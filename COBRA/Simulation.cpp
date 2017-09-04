
#include <ctime>

#include "Simulation.h"



Simulation::Simulation(string map_name, string task_name)
{
	computation_time = 0;
	num_computations = 0;
	LoadMap(map_name);
	LoadTask(task_name);

}

Simulation::~Simulation()
{
}

void Simulation::LoadMap(string fname)
{
	string line;
	ifstream myfile(fname.c_str());
	if (!myfile.is_open())
	{
		cerr << "Map file not found." << endl;
		system("PAUSE");
		return;
	}
	//read file
	getline(myfile, line);
	boost::char_separator<char> sep(",");
	boost::tokenizer< boost::char_separator<char> > tok(line, sep);
	boost::tokenizer< boost::char_separator<char> >::iterator beg = tok.begin();
	row = atoi((*beg).c_str()) + 2; // read number of rows
	beg++;
	col = atoi((*beg).c_str()) + 2; // read number of cols
	
	int agent_num;
	stringstream ss;
	getline(myfile, line);
	ss << line;
	ss >> workpoint_num; //number of endpoints that may have tasks on. Other endpoints are home endpoints


	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> agent_num; //agent number

	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> maxtime; //max timestep
	//resize all vectors
	agents.resize(agent_num);
	token.agents.resize(agent_num);
	token.path.resize(agent_num);
	endpoints.resize(workpoint_num + agent_num);
	token.my_map.resize(row*col);
	token.my_endpoints.resize(row*col);

	// read map
	int ep = 0, ag = 0;
	for (int i = 1; i<row - 1; i++)
	{
		getline(myfile, line);
		for (int j = 1; j<col - 1; j++)
		{
			token.my_map[col*i + j] = (line[j - 1] != '@'); // not a block
			token.my_endpoints[col*i + j] = (line[j - 1] == 'e') || (line[j - 1] == 'r'); // is an endpoint
			if (line[j - 1] == 'e') //endpoint
			{
				endpoints[ep++].loc = i*col + j;
			}
			else if (line[j - 1] == 'r') //robot initial location, also regarded as home endpoint
			{
				endpoints[workpoint_num + ag].loc = i*col + j;
				agents[ag].Set(i*col + j, col, row, ag, maxtime);
				token.agents[ag] = &agents[ag];
				token.path[ag].resize(maxtime);
				for (unsigned int k = 0; k < maxtime; k++)
				{
					token.path[ag][k] = i*col + j;
				}
				ag++;
			}
		}
	}
	myfile.close();

	//set a bloack border of the map
	for (int i = 0; i < row; i++)
	{
		token.my_map[i*col] = false;
		token.my_map[i*col + col - 1] = false;
		token.my_endpoints[i*col] = false;
		token.my_endpoints[i*col + col - 1] = false;
	}
	for (int j = 1; j < col - 1; j++)
	{
		token.my_map[j] = false;
		token.my_map[row*col - col + j] = false;
		token.my_endpoints[j] = false;
		token.my_endpoints[row*col - col + j] = false;
	}

	//initial heuristic matrix for each endpoint
	for (unsigned int e = 0; e < endpoints.size(); e++)
	{
		endpoints[e].SetHVal(token.my_map, col);
		endpoints[e].id = e;
		/*
		cout << "Endpoint " << e << endl;
		for (int i = 0; i < row; i++)
		{
			for (int j = 0; j < col; j++)
			{
				cout << endpoints[e].h_val[i*col + j] << " ";
			}
			cout << endl;
		}
		*/
	}
}

void Simulation::LoadTask(string fname)
{

	string line;
	ifstream myfile(fname.c_str());
	if (!myfile.is_open())
	{
		cerr << "Task file not found." << endl;
		system("PAUSE");
		return;
	}
	//read file
	stringstream ss;
	int task_num;
	getline(myfile, line);
	ss << line;
	ss >> task_num;  // number of tasks
	tasks.resize(maxtime);
	for (int i = 0; i < task_num; i++)
	{
		int s, g, ts, tg;
		getline(myfile, line);
		ss.clear();
		ss << line;
		ss >> t_task >> s >> g >> ts >> tg; //time +s tart + goal + time at start + time at goal
		tasks[t_task].push_back(Task(&endpoints[s], &endpoints[g], ts, tg));
	}
	myfile.close();

	if (!tasks[0].empty())
	{
		for (list<Task>::iterator it = tasks[0].begin(); it != tasks[0].end(); it++)
		{
			token.tasks.push_back(&(*it));
		}
	}
	
	/*
	for (int i = 0; i < maxtime; i++)
	{
		if (tasks[i].size() > 0)
		{
			cout << "Timestep " << i<<" : ";
			for (list<Task>::iterator it = tasks[i].begin(); it != tasks[i].end(); it++)
			{
				cout << it->start->loc << "-->" << it->goal->loc << "	";
			}
			cout << endl;
		}
	}
	*/
}

void Simulation::run_TOTP()
{
	cout << endl << "************TOTP************" << endl;

	while (!token.tasks.empty() || token.timestep <= t_task)
	{
		// pick of  the first agent in the waiting line
		Agent* ag = &agents[0];
		for (int i = 1; i < agents.size(); i++)
		{
			if (agents[i].finish_time == token.timestep)
			{
				ag = &agents[i];
				break;
			}
			else if (agents[i].finish_time < ag->finish_time)
			{
				ag = &agents[i];
			}
		}

		//add new tasks
		for (unsigned int i = token.timestep + 1; i <= ag->finish_time; i++)
		{
			if (tasks[i].empty()) continue;
			for (list<Task>::iterator it = tasks[i].begin(); it != tasks[i].end(); it++)
			{
				token.tasks.push_back(&(*it));
			}
		}
		// update timestep
		token.timestep = ag->finish_time;
		ag->loc = ag->path[token.timestep];

		if (token.tasks.empty())//If no new tasks
		{
			ag->finish_time = ag->finish_time + 1; //agent waits for one timestep
			continue;
		}

		

		//***************test*************************
		/*unsigned int i = 0;
		for (; i < endpoints.size(); i++)
		{
			if (endpoints[i].loc == ag->path[token.timestep]) break;
		}
		if (i == endpoints.size()) system("PAUSE");*/
		//***************end test***************
		num_computations++;
		clock_t start = std::clock();
		if (!ag->TOTP(token))//not get a task
		{
			system("PAUSE");

		}
		computation_time += std::clock() - start;
		/*if (!TestConstraints())
		{
			system("PAUSE");
		}*/
	}
}
void Simulation::run_TPTR()
{
	cout << endl << "************TPTR************" << endl;

	while (!token.tasks.empty() || token.timestep <= t_task)
	{
		//pick off the first agent in the waiting line
		Agent* ag = &agents[0];
		for (int i = 1; i < agents.size(); i++)
		{
			if (agents[i].finish_time == token.timestep)
			{
				ag = &agents[i];
				break;
			}
			else if (agents[i].finish_time < ag->finish_time)
			{
				ag = &agents[i];
			}
		}
		//add new tasks to token
		for (unsigned int i = token.timestep + 1; i <= ag->finish_time; i++)
		{
			if (tasks[i].empty()) continue;
			for (list<Task>::iterator it = tasks[i].begin(); it != tasks[i].end(); it++)
			{
				token.tasks.push_back(&(*it));
			}
		}
		// update timestep
		token.timestep = ag->finish_time;
		ag->loc = ag->path[token.timestep];

		// delete finished tasks
		list<Task*>::iterator it = token.tasks.begin();
		while (it != token.tasks.end())
		{
			if (TAKEN == (*it)->state && token.timestep >= (*it)->ag_arrive_start)
			{
				list<Task*>::iterator done = it++;
				token.tasks.erase(done);
				//cout << "Task " << (*done)->start->loc << "-->" << (*done)->goal->loc << " is done at Timestep " << (*done)->ag_arrive_goal << endl;
			}
			else
			{
				it++;
			}
		}

		//***************test*************************
		/*unsigned int i = 0;
		for (; i < endpoints.size(); i++)
		{
			if (endpoints[i].loc == ag->path[token.timestep]) break;
		}
		if (i == endpoints.size()) system("PAUSE");*/
		//**************end test**********************
		num_computations++;
		clock_t start = std::clock();
		if (!ag->TPTR(token))//not get a task
		{
			system("PAUSE");

		}
		computation_time += std::clock() - start;
		/*if (!TestConstraints())
		{
			system("PAUSE");
		}*/
	} 
}


void Simulation::ShowTask()
{
	unsigned int WaitingTime = 0;
	unsigned int LastFinish = 0;
	cout << endl << "TASK" << endl;
	for (unsigned int i = 0; i < tasks.size(); i++)
	{
		if (tasks[i].size() > 0)
		{
			//cout << "Timestep " << i<<" :	";
			for (list<Task>::iterator it = tasks[i].begin(); it != tasks[i].end(); it++)
			{
				//cout << "Agent " << it->ag->id << " delivers package from " << it->start->loc << " to " << it->goal->loc 
				//	<< "	(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")" << endl;
				WaitingTime += it->ag_arrive_goal - i;
				LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
			}
			//cout << "	";
		}
	}
	cout << endl << "Finishing Timestep:	" << LastFinish << endl;
	cout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
}
void Simulation::SaveTask(const string &fname, const string &instance_name)
{
	// write output file
	std::ofstream fout(fname, ios::app);
	if (!fout) return;
	//fout << mPanel->agents.size() << std::endl;
	unsigned int WaitingTime = 0;
	unsigned int LastFinish = 0;
	for (unsigned int i = 0; i < tasks.size(); i++)
	{
		if (tasks[i].size() > 0)
		{
			//fout << "Timestep " << i << " :	";
			for (list<Task>::iterator it = tasks[i].begin(); it != tasks[i].end(); it++)
			{
				//fout << "Agent " << it->ag->id << " delivers package from " << it->start->loc << " to " << it->goal->loc
				//	<< "	(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")" << endl;
				WaitingTime += it->ag_arrive_goal - i;
				LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
			}
			//cout << "	";
		}
	}
	//fout << endl << "Finishing Timestep:	" << LastFinish << endl;
	//fout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
	fout << instance_name << " " << LastFinish << " " << WaitingTime << " " << computation_time / (double)LastFinish << endl;
	fout.close();
}
void Simulation::SaveThroughput(const string &fname)
{
	// write output file
	std::ofstream fout(fname + ".throughput");
	if (!fout) return;
	//fout << mPanel->agents.size() << std::endl;
	vector<int> thpts(5000, 0);
	vector<int> inpts(5000, 0);

	unsigned int WaitingTime = 0;
	unsigned int LastFinish = 0;
	for (unsigned int i = 0; i < tasks.size(); i++)
	{
		if (tasks[i].size() > 0)
		{
			//fout << "Timestep " << i << " :	";
			for (list<Task>::iterator it = tasks[i].begin(); it != tasks[i].end(); it++)
			{
				//fout << "Agent " << it->ag->id << " delivers package from " << it->start->loc << " to " << it->goal->loc
				//	<< "	(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")" << endl;
				for (int time = 0; time < 100; time++) {
					thpts[it->ag_arrive_goal + time]++;
				}
				//WaitingTime += it->ag_arrive_goal - i;
				//LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
			}
			//cout << "	";
		}
		for (int time = 0; time < 100; time++) {
			inpts[i + time] += tasks[i].size();
		}
	}
	//fout << endl << "Finishing Timestep:	" << LastFinish << endl;
	//fout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
	for (int i = 0; i < thpts.size(); i++) {
		fout << thpts[i] << " " << inpts[i] << endl;
	}
	fout.close();
}
void Simulation::SavePath(const string &fname)
{
	// write output file
	std::ofstream fout(fname);
	if (!fout) return;
	for (unsigned int i = 0; i < token.path.size(); i++)
	{
		fout << maxtime << std::endl;
		for (unsigned int j = 0; j < maxtime; j++)
		{
			int x = token.path[i][j] % col - 1;
			int y = token.path[i][j] / col - 1;
			fout << x << "	" << y << endl;
		}
	}
	fout.close();
}

bool Simulation::TestConstraints()// test vertex collision and edge collision
{
	for (unsigned int ag = 0; ag < agents.size(); ag++)
	{
		for (unsigned int i = ag + 1; i < agents.size(); i++)
		{
			for (unsigned int j = token.timestep + 1; j < maxtime; j++)
			{
				if (agents[ag].path[j] == agents[i].path[j])
				{
					cout << "Agent " << ag << " and " << i << " collide at location "
						<< agents[ag].path[j] << " at time " << j << endl;
					return false;
				}
				else if (token.timestep > 0 && agents[ag].path[j] == agents[i].path[j - 1]
					&& agents[ag].path[j - 1] == agents[i].path[j])
				{
					cout << "Agent " << ag << " and " << i << " collide at edge "
						<< agents[ag].path[j - 1] << "-" << agents[ag].path[j] << " at time " << j << endl;
					return false;
				}
			}
		}
	}
	return true;
}