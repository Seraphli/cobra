#pragma once
#include <cassert>
#include <chrono>
#include <climits>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

// #include <float.h>

#include <boost/tokenizer.hpp> //use to read file

#include "Agent.h"
#include "Endpoint.h"

using namespace std;
using Time = std::chrono::steady_clock;

class Simulation {
public:
  Simulation(string map_name, string task_name, unsigned int deadline_time, bool debug);
  ~Simulation();

  // run
  void run_TOTP(bool verbose);
  void run_TPTR(bool verbose);

  // save
  void ShowTask();
  void SavePath(const string &fname);
  void SavePathUntilTimestep(const string &fname, const int timestep);
  void PrintPathUntilTimestep(const int timestep);
  void SaveTask(const string &fname, const string &instance_name);
  void SaveTaskUntilTimestep(const string &fname, const int timestep);
  void PrintTaskUntilTimestep(const int timestep);
  void SaveThroughput(const string &fname);

  unsigned int deadline_time;
  bool debug;
  double computation_time;
  int num_computations;
  unsigned int end_timestep;

private:
  // initialize
  void LoadMap(string fname);
  void LoadTask(string fname);
  double elapsed_ms() const;
  void SaveDebugInfo(const string &fname);
  // test
  bool TestConstraints();

private:
  Time::time_point t_s;
  int row, col;
  Token token;
  vector<list<Task>> tasks;
  vector<Endpoint> endpoints;
  vector<Agent> agents;

  unsigned int maxtime;

  int workpoint_num; // number of endpoints that may have tasks on. Other
                     // endpoints are home endpoints
  int t_task;        // timestep that last task appears
};
