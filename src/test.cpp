#include <iostream>
#include <vector>
#include <fort.hpp>
#include <ros/ros.h>
#include <ros/master.h>
#include <rosmon_msgs/State.h>
#include <rosmon_msgs/StartStop.h>
#include <std_msgs/Empty.h>
#include <yaml-cpp/yaml.h>
#include <regex>
#include <streamflood/streamflood.h>
#include <numeric>

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
        if (status == rosmon_msgs::NodeState::RUNNING) {
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
    //map<string, string> group_strings;
    streamflood::Streams streams;

public:

    MonitorServer() : streams()
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
        for (const MonitorGroup& group : groups) {
            streams.add_stream(group.name, 10, 10);
        }

		abort_pub = node.advertise<std_msgs::Empty>("/abort", 1000);

        ros::Rate r(1); // 1 hz
        bool ok = true;
        while (ros::ok() && ok) {
            maybe_subscribe();
            handle_bounds();
            //ok = streams.layout(group_strings);
            ok = streams.spin();
            ros::spinOnce();
            r.sleep();
        }
    }

    void perform_action(const MonitorNode& mn, const MonitorGroup::Action& action)
    {
        //uint8_t node_action;
        rosmon_msgs::StartStop srv;
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
        ros::ServiceClient client = node.serviceClient<rosmon_msgs::StartStop>(service_name);
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
                else if (p.second.status == rosmon_msgs::NodeState::IDLE || p.second.status == rosmon_msgs::NodeState::CRASHED) { // dead
                    perform_action(p.second, group.on_kill);
                }
            }
        }
    }

    static MonitorNode rosmon_to_monitor_node(const rosmon_msgs::NodeState& node, const string& rosmon_server)
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

    void status_callback(const rosmon_msgs::State::ConstPtr& msg, const string rosmon_server)
    {
        vector<MonitorNode> nodes;
        nodes.reserve(msg->nodes.size());
        for (const rosmon_msgs::NodeState& state : msg->nodes) {
            nodes.push_back(rosmon_to_monitor_node(state, rosmon_server));
        }
        match_nodes_to_groups(nodes, groups);

        size_t height, width;
        for (const MonitorGroup& group : groups) {
            //print_group(group);
            //group_strings[group.name] = print_group(group);
            string table = print_group(group);
            tie(height, width) = streamflood::Streams::compute_string_shape(table);
            streams.resize_stream(group.name, height, width);
            streams[group.name] << table;
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
                subscribers[topic] = node.subscribe<rosmon_msgs::State>(topic, 1000, boost::bind(&MonitorServer::status_callback, this, _1, name));
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
