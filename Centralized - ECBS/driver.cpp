#include "map_loader.h"
#include "agents_loader.h"
#include "compute_heuristic.h"
#include "egraph_reader.h"
#include "ecbs_search.h"
#include <string>
#include <cstring>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <vector>
#include "ecbs_node.h"
#include <cstdlib>
#include <cmath>
#include "Timer.hpp"

#include "boost/program_options.hpp"

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
namespace pt = boost::property_tree;

using namespace std;

int main(int argc, char** argv) {

  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("map,m", po::value<std::string>()->required(), "input file for map")
      ("agents,a", po::value<std::string>()->required(), "input file for agents")
      ("highway,w", po::value<std::string>()->required(), "input file for highway or CRISSCROSS")
      ("highway-weight,g", po::value<double>()->default_value(1.0), "highway weight")
      ("focal-weight,f", po::value<double>()->default_value(1.0), "focal weight")
      ("tweakGVal", po::bool_switch()->default_value(false), "tweak g value")
      ("output,o", po::value<std::string>()->required(), "output file for schedule")
  ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);

  if (vm.count("help")) {
      std::cout << desc << std::endl;
      return 1;
  }

  po::notify(vm);

  // read the map file and construct its two-dim array
  MapLoader ml(vm["map"].as<string>());

  // read agents' start and goal locations
  AgentsLoader al(vm["agents"].as<string>());

#ifdef _DEBUG
  for (int i = 0; i < ml.rows*ml.cols; i++) {
	  cout << ml.my_map[i] << " " << i << " ";
  }
#endif

  // read the egraph (egraph file, experience_weight, weigthedastar_weight)
  EgraphReader egr;
  if (vm["highway"].as<string>() == "CRISSCROSS") {
    egr = EgraphReader();
    egr.createCrissCrossHWY(&ml);
  } else {
    egr = EgraphReader(vm["highway"].as<string>());
  }

  bool tweakGVal = vm["tweakGVal"].as<bool>();
  ECBSSearch ecbs(ml, al, egr, vm["highway-weight"].as<double>(), vm["focal-weight"].as<double>(), tweakGVal);


  bool res;

  {
    ScopedTimer timer;
    res = ecbs.runECBSSearch();
  }

  if (res == true) {
    cout << "From Driver: Path found" << endl;
    ecbs.printPaths(ml);
  } else {
    cout << "From Driver: NO Path found" << endl;
  }

  ofstream output;
  output.open("replan-" + vm["output"].as<string>(), ios::app);

  output << ecbs.paths.size() << endl;
  for (size_t ag = 0; ag < ecbs.paths.size(); ag++) {
	  int last_location = -1;
	  for (auto& entry : *ecbs.paths[ag]) {
		  if (entry.location != last_location) {
			  output << entry.location << " ";
			  last_location = entry.location;
		  }		  
	  }
	  output << endl;
  }

  output.close();

  // write output file
  using namespace pt;

  ptree pt;
  ptree agents;
  for (size_t ag = 0; ag < ecbs.paths.size(); ag++) {
    ptree agent;
    std::stringstream sstr;
    sstr << "agent" << ag;
    agent.put("name", sstr.str());
    agent.put("group", ag);
    ptree path;
    for (auto& entry : *ecbs.paths[ag]) {
      ptree pathentry;
      pathentry.put("x", ml.col_coordinate(entry.location));
      pathentry.put("y", ml.row_coordinate(entry.location));
      pathentry.put("z", 0);
      pathentry.put("locationId", entry.location);
      pathentry.put("action", entry.action);
      pathentry.put("orientation", entry.orientation);

      path.push_back(std::make_pair("", pathentry));
    }
    agent.add_child("path", path);
    agents.push_back(std::make_pair("", agent));
  }
  pt.add_child("agents", agents);
  write_json(vm["output"].as<string>(), pt);

  return 0;

}
