#include <iostream>
#include <vector>
#include <fort.hpp>
#include <yaml-cpp/yaml.h>
#include <regex>

using namespace std;

struct MonitorNode {
    string name;
    int restarts;
    double ram;
    double load;
    int status;
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
        nodes_regex = regex(self_name);
        max_ram = MAX_INF;
        max_load = MAX_INF;
        max_restarts = MAX_INF;
        on_kill = Action::none;
        on_max = Action::none;
    }
};

const int MonitorGroup::MAX_INF = 1000;

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
        nodes.push_back(node);
    }

    return nodes;
}

void print_group(const MonitorGroup& group)
{
    fort::table table;
    table.set_border_style(FT_DOUBLE2_STYLE);

    // Fill table with data
    table << fort::header << group.name << "Active" << "Load" << "RAM" << "Restarts" << fort::endr;

    for (const pair<string, MonitorNode>& p : group.nodes) {
        table << p.first << "Yes" << p.second.load << p.second.ram << p.second.restarts << fort::endr;
    }
    table << fort::separator
          << "Max" << "-" << MonitorGroup::bound_to_string(group.max_load) << MonitorGroup::bound_to_string(group.max_ram) << MonitorGroup::bound_to_string(group.max_restarts) << fort::endr
          << "Action" << MonitorGroup::action_to_string(group.on_kill) << MonitorGroup::action_to_string(group.on_max) << MonitorGroup::action_to_string(group.on_max) << MonitorGroup::action_to_string(MonitorGroup::Action::kill) << fort::endr
          << fort::endr;

    table[0][0].set_cell_min_width(20);
    table[0][0].set_cell_text_align(fort::text_align::left);
    //table[2].set_cell_row_type(fort::row_type::header);

    std::cout << table.to_string() << std::endl;
}

int main()
{
    YAML::Node config = YAML::LoadFile("../example/groups.yaml");

    if (config.Type() != YAML::NodeType::Sequence) {
        cout << "It's not a sequence, quitting!" << endl;
        exit(-1);
    }

    vector<MonitorGroup> groups = { MonitorGroup("node_name") };

    for (const YAML::Node& group : config) {
        if (!group["group"]) {
            cout << "Group does not contain name, quitting" << endl;
            continue;
        }
        if (!group["nodes"]) {
            cout << "Group " << group["group"] << " does not contain nodes, quitting" << endl;
            continue;
        }

        groups.push_back(MonitorGroup(group));
    }

    vector<MonitorNode> nodes = get_example_nodes();

    match_nodes_to_groups(nodes, groups);

    for (const MonitorGroup& group : groups) {
        print_group(group);
    }

    return 0;
}
