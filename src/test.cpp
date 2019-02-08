#include <iostream>
#include <vector>
#include <fort.hpp>
#include <ros/ros.h>
#include <ros/master.h>
#include <rosmon/State.h>
#include <rosmon/StartStop.h>
#include <std_msgs/Empty.h>
#include <yaml-cpp/yaml.h>
#include <shelf-pack.hpp>
#include <regex>
#include <ncurses.h>
#include <locale.h>
#include <term.h>
#include <sys/ioctl.h>

using namespace std;

struct MonitorNode {
    string name;
    string rosmon_server;
    int restarts;
    double ram;
    double load;
    int status;

    string get_status_string() const
    {
        if (status == rosmon::NodeState::RUNNING) {
            return "alive";
        }
        else {
            return "dead";
        }
    }
};

struct MonitorGroup {
    enum class Action { kill, restart, abort, none };

    const static int MAX_INF;

    string name;
    Action on_max;
    Action on_kill;
    int max_restarts;
    double max_ram;
    double max_load;

    regex nodes_regex;
    map<string, MonitorNode> nodes;

    static regex list_to_regex(const vector<string>& strings)
    {
        string s = std::accumulate(strings.begin(), strings.end(), string(),
            [](const string& s1, const string& s2) {
                return s1.empty() ? s2 : s1 + "|" + s2;
            });
        return regex(s);

    }

    static Action string_to_action(const string& s)
    {
        if (s == "kill") {
            return Action::kill;
        }
        if (s == "restart") {
            return Action::restart;
        }
        if (s == "abort") {
            return Action::abort;
        }
        if (s == "none") {
            return Action::none;
        }
        return Action::none;
    }

    static string action_to_string(const Action& a)
    {
        switch(a) {
            case Action::kill:
                return "kill";
            case Action::restart:
                return "restart";
            case Action::abort:
                return "abort";
            case Action::none:
                return "none";
        }
    }

    template <typename T>
    static string bound_to_string(T val)
    {
        return val == MAX_INF? "-" : to_string(val);
    }

    MonitorGroup(const YAML::Node& group)
    {
        name = group["group"].as<string>();

        YAML::Node n = group["nodes"];
        if (n.Type() == YAML::NodeType::Sequence) {
            vector<string> strings = n.as<vector<string> >();
            nodes_regex = MonitorGroup::list_to_regex(strings);
        }
        else {
            nodes_regex = regex(n.as<string>());
        }

        max_ram = group["max_ram"].as<double>(MAX_INF);
        max_load = group["max_load"].as<double>(MAX_INF);
        max_restarts = group["max_restarts"].as<int>(MAX_INF);
        on_kill = string_to_action(group["on_kill"].as<string>("none"));
        on_max = string_to_action(group["on_max"].as<string>("none"));
    }

    MonitorGroup(const string& self_name)
    {
        name = "Self";
        nodes_regex = regex(self_name.substr(1, self_name.size()-1));
        max_ram = MAX_INF;
        max_load = MAX_INF;
        max_restarts = 5; // MAX_INF;
        //on_kill = Action::none;
        on_kill = Action::abort;
        on_max = Action::none;
    }
};

const int MonitorGroup::MAX_INF = 100000;

void match_nodes_to_groups(const vector<MonitorNode>& nodes, vector<MonitorGroup>& groups)
{
    map<string, string> node_map;
    for (const MonitorNode& node : nodes) {
        for (MonitorGroup& group : groups) {
            if (regex_match(node.name, group.nodes_regex)) {
                group.nodes[node.name] = node;
                break;
            }
        }
    }
}

vector<MonitorNode> get_example_nodes()
{
    vector<string> example_names = {"behavior_tree", "uavcan_to_ros_bridge", "ros_to_uavcan_bridge", "stim_driver", "tf", "foo", "bar", "baz"};

    vector<MonitorNode> nodes;
    for (const string& name : example_names) {
        MonitorNode node;
        node.name = name;
        node.restarts = node.ram = node.load = 0;
        node.status = 1; // running
        nodes.push_back(node);
    }

    return nodes;
}


class LayoutServer
{
public:
    struct Window {

        size_t hash;
        size_t y0;
        size_t x0;
        size_t height;
        size_t width;
        uint8_t display_group;
        string window_string;
        WINDOW* window;

    };

    map<string, Window> windows;
    size_t screen_width;
    size_t screen_height;
    WINDOW* status_bar;
    uint8_t number_display_groups;
    uint8_t current_display_group;
    bool switched;

    LayoutServer()
    {
        setlocale(LC_ALL, "");
        initscr();
        cbreak();
        noecho();
        nodelay(stdscr, TRUE);
        keypad(stdscr, TRUE);
        curs_set(0);

        start_color();
        init_pair(1, COLOR_BLACK, COLOR_BLUE);
        init_pair(2, COLOR_BLACK, COLOR_RED);
        init_pair(3, COLOR_BLACK, COLOR_YELLOW);
        init_pair(4, COLOR_BLACK, COLOR_WHITE);
        init_pair(5, COLOR_BLACK, COLOR_GREEN);
        init_pair(6, COLOR_BLACK, COLOR_BLACK);

        screen_width = 0;
        screen_height = 0;

        status_bar = newwin(1, 1, 1, 1);
        wbkgd(status_bar, COLOR_PAIR(5));
        number_display_groups = 0;
        current_display_group = 0;
        switched = false;
    }

    ~LayoutServer()
    {
        nodelay(stdscr, FALSE);
        //getch();
        for (pair<const string, Window>& w : windows) {
            delwin(w.second.window);
        }
        delwin(status_bar);
        endwin();
    }

    static pair<size_t, size_t> get_window_dims(const string& window_string)
    {
        size_t width = window_string.find_first_of('\n');
        //width = 100;

        size_t u = 0;
        string first_row = window_string.substr(0, width);
        const char *c_str = first_row.c_str();
        size_t charCount = 0;
        while (u < width)
        {
            u += mblen(&c_str[u], width - u);
            charCount += 1;
        }
        width = charCount+1;

        size_t height = std::count(window_string.begin(), window_string.end(), '\n');

        //cout << "Height: " << height << ", width: " << width << endl;

        return make_pair(height, width);
    }

    bool compute_layout(const map<string, string>& window_strings, bool force)
    {
        bool recompute = false;
        size_t height, width;
        for (const pair<string, string>& ws : window_strings) {
            tie(height, width) = LayoutServer::get_window_dims(ws.second);
            if (windows.count(ws.first) == 0) {
                Window w;
                w.window_string = ws.second;
                w.height = height;
                w.width = width;
                w.window = NULL;
                windows[ws.first] = w;
                recompute = true;
            }
            else {
                Window& w = windows[ws.first];
                w.window_string = ws.second;
                if (height != w.height || width != w.width) {
                    w.height = height;
                    w.width = width;
                    recompute = true;
                }
            }
        }

        if (!recompute && !force) {
            return false;
        }
        
        vector<string> to_be_placed;
        for (const pair<string, Window>& w : windows) {
            to_be_placed.push_back(w.first);
        }

        uint8_t display_group = 0;
        while (!to_be_placed.empty()) {
            // Initialize the sprite with a width and height..
            
            int start_ind;
            for (start_ind = 0; start_ind < to_be_placed.size()-1; ++start_ind) {
                mapbox::ShelfPack sprite(screen_width, screen_height-1);
                std::vector<mapbox::Bin> bins;
                /*for (const pair<string, Window>& w : windows) {
                    bins.emplace_back(-1, w.second.width, w.second.height);
                }*/
                for (int i = start_ind; i < to_be_placed.size(); ++i) {
                    const Window& w = windows[to_be_placed[i]];
                    bins.emplace_back(-1, w.width, w.height);
                }

                // `pack()` returns a vector of Bin* pointers, with `x`, `y`, `w`, `h` values..
                std::vector<mapbox::Bin*> results = sprite.pack(bins);
                if (results.size() < to_be_placed.size() - start_ind) {
                    continue;
                }

                for (int i = start_ind; i < to_be_placed.size(); ++i) {
                    Window& w = windows[to_be_placed[i]];
                    w.x0 = results[i-start_ind]->x;
                    w.y0 = results[i-start_ind]->y;
                    w.display_group = display_group;
                }

                display_group += 1;
                break;
            }

            if (start_ind == to_be_placed.size()-1) {
                Window& w = windows[to_be_placed[start_ind]];
                w.x0 = 0;
                w.y0 = 0;
                w.display_group = display_group;
                display_group += 1;
            }

            to_be_placed.resize(start_ind);
        }
        number_display_groups = display_group;

        return true;
    }

    bool layout(const map<string, string>& window_strings)
    {
        struct winsize size;
        if (ioctl(0, TIOCGWINSZ, (char *) &size) < 0) {
            printf("TIOCGWINSZ error");
        }
        bool resized = size.ws_row != screen_height || size.ws_col != screen_width;
        screen_height = size.ws_row;
        screen_width = size.ws_col;
        bool changed = compute_layout(window_strings, resized);

        //printf("%d rows, %d columns\n", size.ws_row, size.ws_col);
        if (resized) {
            endwin();
            // Needs to be called after an endwin() so ncurses will initialize
            // itself with the new terminal dimensions.
            refresh();
            clear();
            mvwin(status_bar, screen_height-1, 0);
            wresize(status_bar, 1, screen_width);
        }

        if (current_display_group >= number_display_groups) {
            current_display_group = 0;
        }

        int counter = 0;
        for (pair<const string, Window>& w : windows) {
            if (w.second.window == NULL) {
                w.second.window = newwin(w.second.height, w.second.width, w.second.y0, w.second.x0);
                //box(w.window, '*', '*');
                //wbkgd(w.second.window, COLOR_PAIR(1));
            }
            else if (changed) {
                mvwin(w.second.window, w.second.y0, w.second.x0);
                wresize(w.second.window, w.second.height, w.second.width);
            }

            if (changed || switched) {
                if (w.second.display_group == current_display_group) {
                    wbkgd(w.second.window, COLOR_PAIR(counter%4+1));
                }
                else {
                    wbkgd(w.second.window, COLOR_PAIR(6));
                }
            }

            if (w.second.display_group == current_display_group) {
                mvwaddstr(w.second.window, 0, 0, w.second.window_string.c_str());
            }

            ++counter;
            
            /*else {
                //mvwaddstr(w.second.window, 0, 0, ""); // this will cause redraws every time we switch windows
                werase(w.second.window);
            }*/
            //mvwprintw(window, 1, 1, "Old window");
        }

        size_t offset = 0;
        std::stringstream ss;
        //ss << "Number display groups: " << int(number_display_groups) << " ";
        wbkgd(status_bar, COLOR_PAIR(5));
        for (int d = 0; d < int(number_display_groups); ++d) {
            ss << " ";
            if (d == current_display_group) {
                ss << "*";
            }
            ss << d << ":";
            //wbkgd(status_bar, COLOR_PAIR(5));
            wattron(status_bar, COLOR_PAIR(5));
            mvwaddstr(status_bar, 0, offset, ss.str().c_str());
            wattroff(status_bar, COLOR_PAIR(5));
            offset += ss.str().size();
            ss.str("");
            int counter = 0;
            for (const pair<string, Window>& w : windows) {
                if (w.second.display_group == d) {
                    wattron(status_bar, COLOR_PAIR(5));
                    mvwaddstr(status_bar, 0, offset, " ");
                    wattroff(status_bar, COLOR_PAIR(5));
                    offset += 1;
                    wattron(status_bar, COLOR_PAIR(counter%4+1));
                    mvwaddstr(status_bar, 0, offset, w.first.c_str());
                    wattroff(status_bar, COLOR_PAIR(counter%4+1));
                    offset += w.first.size();
                }
                ++counter;
            }
        }
        //wbkgd(status_bar, COLOR_PAIR(5));
        ss << " q: Quit";
        size_t length = ss.str().size();
        for (int i = length; i < screen_width; ++i) {
            ss << " ";
        }
        wattron(status_bar, COLOR_PAIR(5));
        mvwaddstr(status_bar, 0, offset, ss.str().c_str());
        wattroff(status_bar, COLOR_PAIR(5));

        //int c = getch();
        //char* input;
        /*do {

            for (pair<const string, Window>& w : windows) {
                wrefresh(w.second.window);
            }

            echo();
            noecho();

            //touchwin(INPUT);

            c = getch();
        }
        while (c != KEY_F(1));*/

        
        for (pair<const string, Window>& w : windows) {
            if (w.second.display_group != current_display_group) {
                wrefresh(w.second.window);
            }
        }
        for (pair<const string, Window>& w : windows) {
            if (w.second.display_group == current_display_group) {
                wrefresh(w.second.window);
            }
        }
        wrefresh(status_bar);
        //echo();
        //noecho();
        
        int c = getch();

        if (c < '0' + number_display_groups && c >= '0') {
            current_display_group = uint8_t(c - '0');
            switched = true;
        }

        return c != 27 && c != 'q' && c != KEY_F(1);
    }
};

string print_group(const MonitorGroup& group)
{
    fort::table table;
    table.set_border_style(FT_DOUBLE2_STYLE);

    // Fill table with data
    table << fort::header << group.name << "Status" << "Load" << "RAM" << "Restarts" << fort::endr;

    for (const pair<string, MonitorNode>& p : group.nodes) {
        table << p.first << p.second.get_status_string() << p.second.load << p.second.ram << p.second.restarts << fort::endr;
    }
    table << fort::separator
          << "Max" << "-" << MonitorGroup::bound_to_string(group.max_load) << MonitorGroup::bound_to_string(group.max_ram) << MonitorGroup::bound_to_string(group.max_restarts) << fort::endr
          << "Action" << MonitorGroup::action_to_string(group.on_kill) << MonitorGroup::action_to_string(group.on_max) << MonitorGroup::action_to_string(group.on_max) << MonitorGroup::action_to_string(MonitorGroup::Action::kill) << fort::endr
          << fort::endr;

    table[0][0].set_cell_min_width(20);
    table[0][0].set_cell_text_align(fort::text_align::left);
    //table[2].set_cell_row_type(fort::row_type::header);

    string table_string = table.to_string();
    //get_table_dims(table_string);
    //std::cout << table_string << std::endl;
    return table_string;
}

vector<string> get_published_topics()
{
    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    vector<string> topics;
    for (ros::master::V_TopicInfo::iterator it = master_topics.begin(); it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        //std::cout << "topic_" << it - master_topics.begin() << ": " << info.name << std::endl;
        topics.push_back(info.name);
    }

    return topics;
}

class MonitorServer {

    ros::NodeHandle node;
    map<string, ros::Subscriber> subscribers;
	ros::Publisher abort_pub;
    vector<MonitorGroup> groups;
    map<string, string> group_strings;
    LayoutServer layout_server;

public:

    MonitorServer() : layout_server()
    {
        string config_file;
        if (!ros::param::get("~config_file", config_file)) {
            ROS_ERROR("Need to supply the config file parameter!");
            return;
        }

        YAML::Node config = YAML::LoadFile(config_file);

        if (config.Type() != YAML::NodeType::Sequence) {
            ROS_ERROR("Config yaml file is not a sequence, quitting!");
            return;
        }

        ROS_INFO("Node name: %s", ros::this_node::getName().c_str());
        groups = { MonitorGroup(ros::this_node::getName()) };

        for (const YAML::Node& group : config) {
            if (!group["group"]) {
                ROS_ERROR("Group does not contain name, quitting");
                continue;
            }
            if (!group["nodes"]) {
                ROS_ERROR("Group %s does not contain nodes, quitting", group["group"].as<string>().c_str());
                continue;
            }

            groups.push_back(MonitorGroup(group));
        }

        //vector<MonitorNode> nodes = get_example_nodes();
        //match_nodes_to_groups(nodes, groups);

		abort_pub = node.advertise<std_msgs::Empty>("/abort", 1000);

        ros::Rate r(1); // 1 hz
        bool ok = true;
        while (ros::ok() && ok) {
            maybe_subscribe();
            handle_bounds();
            ok = layout_server.layout(group_strings);
            ros::spinOnce();
            r.sleep();
        }
    }

    void perform_action(const MonitorNode& mn, const MonitorGroup::Action& action)
    {
        //uint8_t node_action;
        rosmon::StartStop srv;
        switch (action) {
            case MonitorGroup::Action::kill:
                srv.request.action = srv.request.STOP;
                break;
            case MonitorGroup::Action::restart:
                srv.request.action = srv.request.RESTART;
                break;
            case MonitorGroup::Action::abort:
                abort_pub.publish(std_msgs::Empty());
                return;
            case MonitorGroup::Action::none:
                return;
        }

        ROS_INFO("Executing action with node %s", mn.name.c_str());

        string service_name = string("/") + mn.rosmon_server + "/start_stop";
        ros::ServiceClient client = node.serviceClient<rosmon::StartStop>(service_name);
        srv.request.node = mn.name;
        if (client.call(srv)) {
            ROS_INFO("ROSMON successfully executed action %s", service_name.c_str());
        }
        else {
            ROS_ERROR("Failed to call service %s", service_name.c_str());
        }
    }

    void handle_bounds()
    {
        for (const MonitorGroup& group : groups) {
            for (const pair<string, MonitorNode>& p : group.nodes) {
                if (p.second.restarts >= group.max_restarts) {
                    perform_action(p.second, MonitorGroup::Action::kill);
                    perform_action(p.second, group.on_kill);
                }
                else if (p.second.load > group.max_load || p.second.ram > group.max_ram) {
                    perform_action(p.second, group.on_max);
                }
                else if (p.second.status == rosmon::NodeState::IDLE || p.second.status == rosmon::NodeState::CRASHED) { // dead
                    perform_action(p.second, group.on_kill);
                }
            }
        }
    }

    static MonitorNode rosmon_to_monitor_node(const rosmon::NodeState& node, const string& rosmon_server)
    {
        MonitorNode mn;
        mn.name = node.name;
        mn.restarts = node.restart_count;
        mn.load = node.user_load;
        mn.ram = 1e-9f*double(node.memory);
        mn.rosmon_server = rosmon_server;
        mn.status = node.state;
        return mn;
    }

    void status_callback(const rosmon::State::ConstPtr& msg, const string rosmon_server)
    {
        vector<MonitorNode> nodes;
        nodes.reserve(msg->nodes.size());
        for (const rosmon::NodeState& state : msg->nodes) {
            nodes.push_back(rosmon_to_monitor_node(state, rosmon_server));
        }
        match_nodes_to_groups(nodes, groups);

        for (const MonitorGroup& group : groups) {
            //print_group(group);
            group_strings[group.name] = print_group(group);
        }
    }

    void maybe_subscribe()
    {

        regex rosmon_regex("\/rosmon_[0-9]*\/state");
        vector<string> topics = get_published_topics();

        for (const string& topic : topics) {
            if (regex_match(topic, rosmon_regex) && subscribers.count(topic) == 0) {
                //subscribers[topic] = node.subscribe(topic, 1000, &MonitorServer::status_callback, this);
				std::string name = topic.substr(1, topic.substr(1, topic.size()-1).find("/"));
                subscribers[topic] = node.subscribe<rosmon::State>(topic, 1000, boost::bind(&MonitorServer::status_callback, this, _1, name));
            }
        }

    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosmon_monitor_node");
    ros::NodeHandle ros_node;

    MonitorServer ms;

    return 0;
}
