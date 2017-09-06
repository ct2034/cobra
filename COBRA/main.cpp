#include "Simulation.h"

int main(int argc, char** argv)
{
	Simulation simu1(argv[1], argv[2]);
	simu1.run_TOTP();	
	simu1.SaveThroughput((string)argv[2] + "_tp");
	simu1.SaveTask("tp_output.txt", argv[2]);

	Simulation simu2(argv[1], argv[2]);
	simu2.run_TPTR();
	simu2.SaveThroughput((string)argv[2] + "_tptr");
	simu2.SaveTask("tptr_output.txt", argv[2]);
	//simu2.SavePath("20analysis1_5.path");
	//simu2.SaveTask("20analysis1_5.tsk");


	simu1.ShowTask();
	simu2.ShowTask();
    return 0;
}
	
