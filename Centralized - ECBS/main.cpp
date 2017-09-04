#include <iostream>
#include "Simulation.h"

using namespace std;

void main(int argc, char** argv)
{
	//Simulation simu("kiva-150-1500-100.map", "kiva-150-1500-100.task");
	Simulation simu(argv[1], argv[2]);
	int w = 1;
	simu.run(w);
	simu.SaveThroughput((string)(argv[2]) + "ECBS_w" + to_string(w));
	simu.SaveTask("ECBS_w" + to_string(w) + "_output.txt", argv[2]);
	simu.ShowTask();
	//simu.SaveTask("20analysis1_5.tsk");
	//simu.SavePath("20analysis1_5.path");
	return;
}
