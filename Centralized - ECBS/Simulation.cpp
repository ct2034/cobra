#include "Simulation.h"

#include <ctime>

Simulation::Simulation(string map_name, string task_name)
{

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

	stringstream ss;
	getline(myfile, line);
	ss << line;
	ss >> workpoint_num;

	int agent_num;
	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> agent_num;

	ss.clear();
	getline(myfile, line);
	ss << line;
	ss >> maxtime;

	this->agents.resize(agent_num);
	endpoints.resize(workpoint_num + agent_num);
	my_map.resize(row*col);
	DeliverGoal.resize(row*col, false);
	// read map
	int ep = 0, ag = 0;
	for (int i = 1; i<row - 1; i++)
	{
		getline(myfile, line);
		for (int j = 1; j<col - 1; j++)
		{
			my_map[col*i + j] = (line[j - 1] != '@'); // not a block
			if (line[j - 1] == 'e') //endpoint
			{
				endpoints[ep++].loc = i*col + j;
			}
			else if (line[j - 1] == 'r') //robot rest
			{
				endpoints[workpoint_num + ag].loc = i*col + j;
				agents[ag].Set(i*col + j, col, row, ag, maxtime);
				ag++;
			}
		}
	}
	myfile.close();

	//set the border of the map blocked
	for (int i = 0; i < row; i++)
	{
		my_map[i*col] = false;
		my_map[i*col + col - 1] = false;
	}
	for (int j = 1; j < col - 1; j++)
	{
		my_map[j] = false;
		my_map[row*col - col + j] = false;
	}

	//initial heuristic matrix for each endpoint
	for (unsigned int e = 0; e < endpoints.size(); e++)
	{
		endpoints[e].SetHVal(my_map, col);
		endpoints[e].id = e;
	}
}
void Simulation::LoadTask(string fname)
{

	string line;
	ifstream myfile(fname.c_str());
	if (!myfile.is_open())
	{
		cerr << "Task file not found." << endl;
		return;
	}
	//read file
	stringstream ss;
	int task_num;
	getline(myfile, line);
	ss << line;
	ss >> task_num;  // number of tasks
	tasks_total.resize(maxtime);
	for (int i = 0; i < task_num; i++)
	{
		int s, g, ts, tg;
		getline(myfile, line);
		ss.clear();
		ss << line;
		ss >> t_task >> s >> g >> ts >> tg; //time+start+goal+time at start+time at goal
		tasks_total[t_task].push_back(Task(&endpoints[s], &endpoints[g], ts, tg));
	}
	myfile.close();
}


void Simulation::AssignTasks(vector<Agent*> &agents, const vector<vector<int> > &cons_paths)
{
	//hold goals of delivering tasks
	vector<bool> hold(col*row, false);
	for (list<Task*>::iterator it = tasks_deliver.begin(); it != tasks_deliver.end(); it++)
	{
		hold[(*it)->goal->loc] = true;
	}

	//pick off tasks that starts are not held and different with each other
	vector<Task*> tasks(0);
	vector<Endpoint*> starts(0);
	for (list<Task*>::iterator it = tasks_assign.begin(); it != tasks_assign.end(); it++)
	{
		if (hold[(*it)->start->loc] == false)
		{
			tasks.push_back(*it);
			starts.push_back((*it)->start);
			hold[(*it)->start->loc] = true; // once a task is chosen, hold its start and goal
		}
	}

	//if tasks are less than agents, add non-holding endpoints for each agent
	if (starts.size() < agents.size())
	{
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			//choose nearest endpoint ep for agent i
			int ep = 0, dis = col*row;
			for (unsigned int j = 0; j < endpoints.size(); j++)
			{
				if (hold[endpoints[j].loc] == false && endpoints[j].loc == agents[i]->loc)
				{
					ep = j;
					dis = 0;
					break;

				}
				else if (hold[endpoints[j].loc] == false && 0 < endpoints[j].h_val[agents[i]->loc] && endpoints[j].h_val[agents[i]->loc] < dis)
				{
					ep = j;
					dis = endpoints[j].h_val[agents[i]->loc];
				}
			}
			hold[endpoints[ep].loc] = true;
			starts.push_back(&endpoints[ep]);
		}
	}
	

	
	//compute cost matrix
	//cout << "Cost matrix:" << endl;
	dlib::matrix<int> cost(starts.size(), starts.size());
	for (unsigned int i = 0; i < starts.size(); i++)
	{
		if (i >= agents.size())
		{
			for (unsigned int j = 0; j < starts.size(); j++)
			{
				cost(i, j) = 0;
			}
		}
		else
		{
			for (unsigned int j = 0; j < tasks.size(); j++)
			{
				if (starts[j]->h_val[agents[i]->loc] == -1)
				{
					cost(i, j) = 0;
					system("PAUSE");
				}
				else
				{
					bool* res_table = new bool[my_map.size() * maxtime]();  // initialized to false
					SingleAgentECBS single(cons_paths, starts[j]->h_val, my_map, i, agents[i]->loc, starts[j]->loc, col, timestep, maxtime);
					if (single.findPath(1, NULL, res_table, maxtime) == false)
						cout << "NO SOLUTION EXISTS";
					int path = single.path.size();
					//cout << path << "	";
					cost(i, j) = (2 * col*row - path)*agents.size()*starts.size();
					delete[] res_table;
				}
			}
			for (unsigned int j = tasks.size(); j < starts.size(); j++)
			{
				if (starts[j]->h_val[agents[i]->loc] == -1)
				{
					cost(i, j) = 0;
					system("PAUSE");
				}
				else
				{
					bool* res_table = new bool[my_map.size() * maxtime]();  // initialized to false
					SingleAgentECBS single(cons_paths, starts[j]->h_val, my_map, i, agents[i]->loc, starts[j]->loc, col, timestep, maxtime);
					if (single.findPath(1, NULL, res_table, maxtime) == false)
						cout << "NO SOLUTION EXISTS";
					int path = single.path.size();
					//cout << -path << "	";
					cost(i, j) = col*row*agents.size()*starts.size() - path;
					delete[] res_table;
				}
			}
			//cout << endl;
		}
	}
	
	// To find out the best assignment of people to jobs we just need to call this function.
	vector<long> assignment = max_cost_assignment(cost);

	
	//assign
	for (unsigned int i = 0; i < agents.size(); i++)
	{
		agents[i]->next_ep = starts[assignment[i]];
		if (assignment[i] < tasks.size())
		{
			//cout << "Agent " << agents[i]->id << " tasks Task " << tasks[assignment[i]]->start->loc << "-->" << tasks[assignment[i]]->goal->loc << endl;
			//agents[i]->task = tasks[assignment[i]];
		}
		else
		{
			//cout << "Agent " << agents[i]->id << " go to endpoint " << starts[assignment[i]]->loc << endl;
			agents[i]->task = NULL;
		}
	}
}
bool Simulation::PathFinding(vector<Agent*> &agents, const vector<vector<int> > &cons_paths)
{
	ECBSSearch ecbs(my_map, agents, cons_paths, timestep, col, focal_w);
	if (ecbs.runECBSSearch())
	{
		//update
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			//update searching path
			for (unsigned int j = 0; j < ecbs.paths[i].size(); j++)
			{
				agents[i]->path[timestep + j] = ecbs.paths[i][j];
			}
			
			//hold endpoint
			for (unsigned int j = ecbs.paths[i].size() + timestep; j < maxtime; j++)
			{
				agents[i]->path[j] = agents[i]->next_ep->loc;
			}
			//update task
			if (agents[i]->delivering == true)
			{
				agents[i]->task->ag_arrive_goal= timestep + ecbs.paths[i].size() - 1;
			}
			agents[i]->task = NULL;
		}
		return true;
	}
	else
	{
		cout << "CBS fails" << endl;
		std::system("PAUSE");
		//Recovery. Let robots move along its original paths.
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			if (agents[i]->delivering == true)
			{
				agents[i]->delivering = false;
				DeliverGoal[agents[i]->task->goal->loc] = false;
				agents[i]->task->delivering = false;
				agents[i]->task->ag = NULL;
				tasks_assign.push_back(agents[i]->task);
				tasks_deliver.pop_back();
			}
			agents[i]->task = NULL;
			agents[i]->next_ep = NULL;
		}
		return false;
	}

}

void Simulation::run(double focal_w)
{
	this->focal_w = focal_w;
	for (timestep = 0; timestep <= t_task || !tasks_assign.empty(); timestep++)
	{
		cout << endl << "Timestep " << timestep << endl;
		vector<Agent*> ag_pathfinding;
		vector<Agent*> ag_assign;
		vector<vector<int> > cons_paths;
		// delete FINISH tasks 	and update its agent's state	
		for (list<Task*>::iterator it = tasks_deliver.begin(); it != tasks_deliver.end();)
		{
			if (true == (*it)->delivering && timestep == (*it)->ag_arrive_goal)
			{
				//cout << "Task " << (*it)->start->loc << "-->" << (*it)->goal->loc << " is finished" << endl;
				(*it)->ag->delivering = false;
				DeliverGoal[(*it)->goal->loc] = false;
				list<Task*>::iterator done = it++;
				tasks_deliver.erase(done);
			}
			else
			{
				it++;
			}
		}

		//add new tasks
		for (list<Task>::iterator it = tasks_total[timestep].begin(); it != tasks_total[timestep].end(); it++)
		{
			cout << "New task " << (*it).start->loc << "-->" << (*it).goal->loc << endl;
			tasks_assign.push_back(&(*it));
		}

		//check non-package agents whether it is at a start that isn't held by other agent
		//if it is, assign task to it
		vector<int> ag_loc(my_map.size(), -1);
		vector<int> ag_hold(my_map.size(), -1);
		for (unsigned int i = 0; i < agents.size(); i++)
		{
			agents[i].loc = agents[i].path[timestep];
			ag_hold[agents[i].path[maxtime - 1]] = i; //record every agent's holding point
			if (agents[i].delivering == false)
				ag_loc[agents[i].loc] = i; // record non_package agents current loc
			else
				cons_paths.push_back(agents[i].path); //record package agents' paths
		}
		for (list<Task*>::iterator it = tasks_assign.begin(); it != tasks_assign.end();)
		{
			if (ag_loc[(*it)->start->loc] >= 0 && DeliverGoal[(*it)->goal->loc] == false)// assign agent to  deliver package
			{
				int id = ag_loc[(*it)->start->loc];
				ag_loc[(*it)->start->loc] = -1;
				//cout << "Agent " << id << " takes Task " << (*it)->start->loc << "-->" << (*it)->goal->loc << endl;
				(*it)->ag_arrive_start = timestep;
				//update agent
				agents[id].task = (*it);
				agents[id].next_ep = (*it)->goal;
				agents[id].delivering = true;
				DeliverGoal[(*it)->goal->loc] = true;
				ag_pathfinding.push_back(&agents[id]);
				//update task
				list<Task*>::iterator done = it++;
				(*done)->ag = &agents[id];
				(*done)->delivering = true;
				
				tasks_deliver.push_back(*done);
				tasks_assign.erase(done);

				
			}
			else
			{
				it++;
			}
		}

		num_computations++;
		clock_t start = std::clock();

		if (!ag_pathfinding.empty()) //path finding
		{
			PathFinding(ag_pathfinding, cons_paths);
			for (int i = 0; i < ag_pathfinding.size(); i++)
			{
				cons_paths.push_back(ag_pathfinding[i]->path);
			}
		}
		ag_pathfinding.clear();

		//pick off non-package agents and 				
		//cout << "Non-package agents:	";
		for (int i = 0; i < agents.size(); i++)
		{
			if (agents[i].delivering == false)
			{
				//cout << i << "	";
				ag_assign.push_back(&agents[i]);
			}
		}
		//cout << endl;
		//cout << "Holding goals:	";
		/*
		for (unsigned int i = 0; i < DeliverGoal.size(); i++)
		{
			if (DeliverGoal[i]) cout << i << "	";
		}
		cout << endl;
		*/
		if (!ag_assign.empty()) //assign tasks
		{
			AssignTasks(ag_assign, cons_paths); 
			for (unsigned int i = 0; i < ag_assign.size(); i++)
				ag_pathfinding.push_back(ag_assign[i]);
		}

		if (!ag_pathfinding.empty()) //path finding
		{
			PathFinding(ag_pathfinding, cons_paths);
		}
		
		computation_time += (std::clock() - start);

		if (!TestConstraints()) //test correctness
		{
			std::system("PAUSE");
		}
		
	}
}

void Simulation::ShowTask()
{
	int WaitingTime = 0;
	int LastFinish = 0;
	//cout << "TASK" << endl;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].size() > 0)
		{
			//cout << "Timestep " << i<<" :	";
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
			{
				//cout << "(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")	";
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
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].size() > 0)
		{
			//fout << "Timestep " << i << " :	";
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
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
	vector<int> thpts(2500, 0);
	vector<int> inpts(2500, 0);

	unsigned int WaitingTime = 0;
	unsigned int LastFinish = 0;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].size() > 0)
		{
			//fout << "Timestep " << i << " :	";
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
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
			inpts[i + time] += tasks_total[i].size();
		}
	}
	//fout << endl << "Finishing Timestep:	" << LastFinish << endl;
	//fout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
	for (int i = 0; i < thpts.size(); i++) {
		fout << thpts[i] << " " << inpts[i] << endl;
	}
	fout.close();
}

void Simulation::SavePath(string fname)
{
	// write output file
	std::ofstream fout(fname);
	if (!fout) return;
	//fout << mPanel->agents.size() << std::endl;
	for (unsigned int i = 0; i < this->agents.size(); i++)
	{
		fout << maxtime << std::endl;
		for (unsigned int j = 0; j < maxtime; j++)
		{
			int x = this->agents[i].path[j] % col - 1;
			int y = this->agents[i].path[j] / col - 1;
			fout << x << "	" << y << endl;
		}
	}
	fout.close();
}

bool Simulation::TestConstraints()
{
	for (unsigned int ag = 0; ag < agents.size(); ag++)
	{
		for (unsigned int i = ag + 1; i < agents.size(); i++)
		{
			for (unsigned int j = timestep + 1; j < maxtime; j++)
			{
				if (agents[ag].path[j] == agents[i].path[j])
				{
					cout << "Agent " << ag << " and " << i << " collide at location " 
						<< agents[ag].path[j] << " at time " << j << endl;
					return false;
				}
				else if (timestep > 0 && agents[ag].path[j] == agents[i].path[j - 1]
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
void Simulation::minCost()
{
	int cost = 0;
	for (int i = 0; i < t_task; i++)
	{
		if (!tasks_total[i].empty())
		{
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
			{
				//cout << "(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")	";
				cost += it->start->h_val[it->goal->loc];
			}
		}
	}
	cout << "Min cost" << cost << endl;
}