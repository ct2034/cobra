#include "Simulation.h"
#include <algorithm> 



Simulation::Simulation(int num_agents, int num_tasks, int tasks_per_timestep)
{

	Kiva(20, 8, 10, num_agents, num_tasks, tasks_per_timestep);

}
Simulation::~Simulation()
{
}

void Simulation::Kiva(int podRows, int podCols, int podLength, int num_agents, int num_tasks, int tasks_per_timestep) {
	const int sideWidth = 6;
	row = 1 + 4 * podRows;
	col = 2 * sideWidth + 1 + (1 + podLength) * podCols;
	vector<char> cmap(row * col, '.');
	vector<int> agt_endpoint_hashtable;

	for (int i = 0; i < podRows; i++) {
		int podR = 1 + 4 * i;
		for (int j = 0; j < podCols; j++) {
			for (int k = 0; k < podLength; k++) {				
				int podC = sideWidth + 1 + (1 + podLength) * j + k;
				cmap[podR * col + podC] = 'e';
				cmap[(podR + 1) * col + podC] = '@';
				cmap[(podR + 2) * col + podC] = 'e';
				endpoint_hashtable.push_back(podR * col + podC);
				endpoint_hashtable.push_back((podR + 2) * col + podC);
			}
		}
		for (int j = 0; j < sideWidth; j++) {
			if (j % 3 == 0) {
				continue;
			}
			cmap[podR * col + j] = 'e';
			cmap[(podR + 1) * col + j] = 'e';
			cmap[(podR + 2) * col + j] = 'e';
			cmap[podR * col + (col - 1 - j)] = 'e';
			cmap[(podR + 1) * col + (col - 1 - j)] = 'e';
			cmap[(podR + 2) * col + (col - 1 - j)] = 'e';
			endpoint_hashtable.push_back(podR * col + j);
			endpoint_hashtable.push_back((podR + 1) * col + j);
			endpoint_hashtable.push_back((podR + 2) * col + j);
			endpoint_hashtable.push_back(podR * col + (col - 1 - j));
			endpoint_hashtable.push_back((podR + 1) * col + (col - 1 - j));
			endpoint_hashtable.push_back((podR + 2) * col + (col - 1 - j));

			agt_endpoint_hashtable.push_back(podR * col + j);
			agt_endpoint_hashtable.push_back((podR + 1) * col + j);
			agt_endpoint_hashtable.push_back((podR + 2) * col + j);
			agt_endpoint_hashtable.push_back(podR * col + (col - 1 - j));
			agt_endpoint_hashtable.push_back((podR + 1) * col + (col - 1 - j));
			agt_endpoint_hashtable.push_back((podR + 2) * col + (col - 1 - j));

			if (i > 0) {
				cmap[(podR - 1) * col + j] = 'e';
				cmap[(podR - 1) * col + (col - 1 - j)] = 'e';

				endpoint_hashtable.push_back((podR - 1) * col + j);
				endpoint_hashtable.push_back((podR - 1) * col + (col - 1 - j));
				agt_endpoint_hashtable.push_back((podR - 1) * col + j);
				agt_endpoint_hashtable.push_back((podR - 1) * col + (col - 1 - j));
			}
		}
	}

	string fname;
	std::random_shuffle(agt_endpoint_hashtable.begin(), agt_endpoint_hashtable.end(), [](int n) { return rand() % n; });
	
	for (int real_num_agents = 100; real_num_agents <= num_agents; real_num_agents += 100) {

		for (int i = 0; i < real_num_agents; i++) {
			cmap[agt_endpoint_hashtable[i]] = 'r';
		}
		for (int i = real_num_agents; i < num_agents; i++) {
			cmap[agt_endpoint_hashtable[i]] = '.';
		}

		fname = "kiva-" + to_string(real_num_agents) + "-" + to_string(num_tasks) + "-" + to_string(tasks_per_timestep) + ".map";

		std::ofstream fout(fname);
		if (!fout) return;

		fout << row << "," << col << endl;
		fout << endpoint_hashtable.size() - num_agents << endl;
		//fout << endpoint_hashtable.size() << endl;
		fout << real_num_agents << endl;
		fout << 5000 << endl;
		int index = 0;
		for (int i = 0; i < row; i++) {
			for (int j = 0; j < col; j++) {
				fout << cmap[index];
				index++;
			}
			fout << std::endl;
		}
		fout.close();

	}


	fname = "kiva-" + to_string(num_agents) + "-" + to_string(num_tasks) + "-" + to_string(tasks_per_timestep) + ".task";

	std::ofstream fout_tsk(fname);
	if (!fout_tsk) return;

	fout_tsk << num_tasks << endl;

	int time = -1;
	for (int i = 0; i < num_tasks; i++) {
		if (i%tasks_per_timestep == 0) {
			time++;
		}
		int s = rand() % (endpoint_hashtable.size() - num_agents);
		int g;
		while ((g = rand() % (endpoint_hashtable.size() - num_agents)) == s){};
		fout_tsk << time << " " << s << " " << g << " " << 0 << " " << 0 << endl;
	}
	fout_tsk.close();
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
void Simulation::SaveTask(const string &fname)
{
	// write output file
	std::ofstream fout(fname);
	if (!fout) return;
	//fout << mPanel->agents.size() << std::endl;
	unsigned int WaitingTime = 0;
	unsigned int LastFinish = 0;
	for (unsigned int i = 0; i < tasks_total.size(); i++)
	{
		if (tasks_total[i].size() > 0)
		{
			fout << "Timestep " << i << " :	";
			for (list<Task>::iterator it = tasks_total[i].begin(); it != tasks_total[i].end(); it++)
			{
				fout << "Agent " << it->ag->id << " delivers package from " << it->start->loc << " to " << it->goal->loc
					<< "	(" << it->ag_arrive_start << "," << it->ag_arrive_goal << ")" << endl;
				WaitingTime += it->ag_arrive_goal - i;
				LastFinish = LastFinish > it->ag_arrive_goal ? LastFinish : it->ag_arrive_goal;
			}
			//cout << "	";
		}
	}
	fout << endl << "Finishing Timestep:	" << LastFinish << endl;
	fout << "Sum of Task Waiting Time:	" << WaitingTime << endl;
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