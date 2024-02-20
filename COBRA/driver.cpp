#include "Simulation.h"
// #include <algorithm>
#include <boost/program_options.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <iostream>
#include <string>
#include <vector>

namespace po = boost::program_options;
using namespace std;

void validate_string(const boost::any &v, const vector<string> &valid_strings) {
  const string *s = boost::any_cast<string>(&v);

  if (find(valid_strings.begin(), valid_strings.end(), *s) ==
      valid_strings.end())
    throw po::validation_error(po::validation_error::invalid_option_value);
}

int main(int argc, char **argv) {
  try {
    po::options_description desc("Allowed options");
    vector<string> valid_algorithms = {"TP", "TPTS"};
    string algorithm;

    desc.add_options()("help", "produce help message")(
        "map,m", po::value<string>()->required(), "input file for map")(
        "task,t", po::value<string>()->required(),
        "input file for task")("algorithm,a",
                               po::value<string>(&algorithm)
                                   ->default_value("TP")
                                   ->notifier([&](const string &val) {
                                     validate_string(val, valid_algorithms);
                                   }),
                               "algorithm to use (TP or TPTS)")(
        "deadline,l", po::value<unsigned int>()->default_value(1000),
        "deadline for the simulation in ms")(
        "output-path,p", po::value<string>()->default_value("path.txt"),
        "output path file")("output-task,k",
                            po::value<string>()->default_value("task.txt"),
                            "output task file")(
        "verbose,v", po::bool_switch()->default_value(false),
        "print verbose output")("debug,d",
                                po::bool_switch()->default_value(false),
                                "print debug output");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
      cout << desc << "\n";
      return 0;
    }

    po::notify(vm);

    if (algorithm == "TP") {
      Simulation simu(vm["map"].as<string>(), vm["task"].as<string>(),
                      vm["deadline"].as<int>(), vm["debug"].as<bool>());
      simu.run_TOTP(vm["verbose"].as<bool>());
      simu.SavePathUntilTimestep(vm["output-path"].as<string>(),
                                 simu.end_timestep);
      simu.SaveTaskUntilTimestep(vm["output-task"].as<string>(),
                                 simu.end_timestep);
    } else if (algorithm == "TPTS") {
      Simulation simu(vm["map"].as<string>(), vm["task"].as<string>(),
                      vm["deadline"].as<int>(), vm["debug"].as<bool>());
      simu.run_TPTR(vm["verbose"].as<bool>());
      simu.SavePathUntilTimestep(vm["output-path"].as<string>(),
                                 simu.end_timestep);
      simu.SaveTaskUntilTimestep(vm["output-task"].as<string>(),
                                 simu.end_timestep);
    }

  } catch (exception &e) {
    cerr << "Error: " << e.what() << "\n";
    return 1;
  } catch (...) {
    cerr << "Unknown error!"
         << "\n";
    return 1;
  }
  // Simulation simu1(argv[1], argv[2]);
  // simu1.run_TOTP();
  //    simu1.SaveThroughput((string)argv[2] + "_tp_throughput");
  //    simu1.SaveTask((string)argv[2] + "_tp_out", argv[2]);
  // simu1.SavePath((string)argv[2] + "_tp_path");

  // Simulation simu2(argv[1], argv[2]);
  // simu2.run_TPTR();
  //    simu2.SaveThroughput((string)argv[2] + "_tptr_throughput");
  //    simu2.SaveTask((string)argv[2] + "_tptr_out", argv[2]);
  // simu2.SavePath((string)argv[2] + "_tptr_path");

  //    simu1.ShowTask();
  // simu2.ShowTask();
  return 0;
}
