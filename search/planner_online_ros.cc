#include "option_parser.h"
#include "search_engine.h"

#include "utils/system.h"
#include "utils/timer.h"
#include "abstract_task.h"

#include <ros/ros.h>
#include <std_msgs/String.h>

#include "fd_ros/Planning.h"
#include "fd_ros/Fact.h"


#include <iostream>
#include <set>

using namespace std;
using utils::ExitCode;

int saved_argc;
const char **saved_argv;
std::map<string, pair<int, int> > pddl_to_fact;
std::vector<int> default_values;


bool planServiceRequestHandler(fd_ros::Planning::Request &req, fd_ros::Planning::Response &res)
{
    ROS_INFO("Got planning request");
    res.unsolvable = false;

    for (size_t i = 0; i < g_initial_state_data.size(); i++) {
        g_initial_state_data[i] = default_values[i];
        cout << i << " = " << default_values[i] << endl;
    }

    for (size_t i = 0; i < req.init_state_facts.size(); i++) {
        string name = "Atom " + req.init_state_facts[i].name + "(";
        for (size_t j = 0; j < req.init_state_facts[i].args.size(); j++) {
            if (j > 0) {
                name.append(", ");
            }
            name.append(req.init_state_facts[i].args[j]);
        }
        name.append(")");
        std::map<string, pair<int, int> >::iterator it = pddl_to_fact.find(name);
        if (it != pddl_to_fact.end()) {
          cout << name << ": " << it->second.first << " = " << it->second.second << endl;
          g_initial_state_data[it->second.first] = it->second.second;

        } else {
          cout << "can't find " << name << endl;
          return false;
        }
    }
    
    for (size_t i = 0; i < g_initial_state_data.size(); i++) {
        if (g_initial_state_data[i] == -1) {
            cout << "No value set for var " << i << endl;
            return false;
        }
    }
    


    bool unit_cost = is_unit_cost();
    SearchEngine *engine = OptionParser::parse_cmd_line(saved_argc, saved_argv, false, unit_cost);

    utils::Timer search_timer;
    engine->search();
    search_timer.stop();
    utils::g_timer.stop();

    //engine->save_plan_if_necessary();
    engine->print_statistics();
    cout << "Search time: " << search_timer << endl;
    cout << "Total time: " << utils::g_timer << endl;

    if (engine->found_solution()) {
        ROS_INFO("Plan found");
        res.solved = true;
        for (size_t i = 0; i < engine->get_plan().size(); i++) {
          res.plan.push_back(engine->get_plan()[i]->get_name());
        }
    } else {
        ROS_INFO("No plan found");
        res.solved = false;
    }

    return true;
}



int main(int argc, char **argv) {
    utils::register_event_handlers();

    if (argc < 2) {
        cout << OptionParser::usage(argv[0]) << endl;
        utils::exit_with(ExitCode::INPUT_ERROR);
    }

    if (string(argv[1]).compare("--help") != 0)
        read_everything(cin);

    default_values.resize(g_fact_names.size(), -1);

    for (size_t i = 0; i < g_fact_names.size(); i++) {
        for (size_t j = 0; j < g_fact_names[i].size(); j++) {
            cout << i << " " << j << " " << g_fact_names[i][j] << endl;
            pddl_to_fact[g_fact_names[i][j]] = make_pair((int) i, (int) j);
            if ((g_fact_names[i][j].size() > 11) && (g_fact_names[i][j].compare(0, 11, "NegatedAtom") == 0)) {
                cout << "got default value: " << i << " = " << j << endl;  
                default_values[i] = j;
            }
        }
    }


    // The command line is parsed twice: once in dry-run mode, to
    // check for simple input errors, and then in normal mode.
    bool unit_cost = is_unit_cost();
    try {
        OptionParser::parse_cmd_line(argc, const_cast<const char**>(argv), true, unit_cost);        
    } catch (ArgError &error) {
        cerr << error << endl;
        OptionParser::usage(argv[0]);
        utils::exit_with(ExitCode::INPUT_ERROR);
    } catch (ParseError &error) {
        cerr << error << endl;
        utils::exit_with(ExitCode::INPUT_ERROR);
    }

    saved_argc = argc;
    saved_argv = const_cast<const char**>(argv);
    
    ros::init(argc, argv, "fd_search_online_ros");
    ros::NodeHandle n;
    ros::ServiceServer service = n.advertiseService("fd_plan", planServiceRequestHandler);

    ros::spin();
}
