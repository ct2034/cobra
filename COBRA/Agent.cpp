#include "Agent.h"

struct HeuristicNode
{
	HeuristicNode(int loc, Task *task, int h_val) : loc(loc), task(task), h_val(h_val) {};
	int loc;
	Task *task;
	int h_val;
};
struct CompareHeuristic
{
	// returns true if n1 > n2 (note -- this gives us *min*-heap).
	bool operator()(const HeuristicNode &n1, const HeuristicNode &n2) const
	{
		return n1.h_val > n2.h_val;
	}
};
//Token
Token::Token(const Token &token)
{
	my_map.resize(token.my_map.size());
	copy(token.my_map.begin(), token.my_map.end(), my_map.begin());
	
	tasks.resize(token.tasks.size());
	copy(token.tasks.begin(), token.tasks.end(), tasks.begin());
	
	agents.resize(token.agents.size());
	copy(token.agents.begin(), token.agents.end(), agents.begin());
	
	path.resize(token.path.size());
	for (int i = 0; i < path.size(); i++)
	{
		path[i].resize(token.path[i].size());
		for (int j = 0; j < token.path[i].size(); j++)
		{
			path[i][j] = token.path[i][j];
		}
	}
	timestep = token.timestep;
}
void Token::reset(const Token &token)
{
	vector<bool>::const_iterator j = token.my_map.begin();
	for (vector<bool>::iterator i = my_map.begin(); i != my_map.end(); i++, j++)
	{
		(*i) = (*j);
	}
	
	list<Task*>::const_iterator j2 = token.tasks.begin();
	for (list<Task*>::iterator i = tasks.begin(); i != tasks.end(); i++, j2++)
	{
		(*i) = (*j2);
	}
	
	for (int i = 0; i < agents.size(); i++)
	{
		agents[i] = token.agents[i];
	}
	
	path.resize(token.path.size());
	for (int i = 0; i < path.size(); i++)
	{
		path[i].resize(token.path[i].size());
		for (int j = 0; j < token.path[i].size(); j++)
		{
			path[i][j] = token.path[i][j];
		}
	}
	timestep = token.timestep;
}

//Agent
Agent::Agent(int loc, int col, int row, int id, int maxtime)
	:loc(loc), col(col), row(row), id(id), finish_time(0), maxtime(maxtime)
{ 
	for (int i = 0; i < maxtime; i++)
	{
		path.push_back(loc);//hold the initial point
	}
};
Agent::Agent(const Agent &ag)
{
	path.resize(ag.path.size());
	copy(ag.path.begin(), ag.path.end(), path.begin());
	loc = ag.loc;
	id = ag.id;
	maxtime = ag.maxtime;
	finish_time = ag.finish_time;
	task = ag.task;
	row = ag.row;
	col = col;
}
Agent::~Agent()
{
}

void Agent::reset(const Agent &ag)
{
	for (int i = 0; i < path.size(); i++)
	{
		path[i] = ag.path[i];
	}
	loc = ag.loc;
	id = ag.id;
	maxtime = ag.maxtime;
	finish_time = ag.finish_time;
	task = ag.task;
	row = ag.row;
	col = col;
}
void Agent::Set(int loc, int col, int row, int id, unsigned int maxtime)
{
	this->loc = loc;
	this->col = col;
	this->row = row;
	this->id = id;
	this->finish_time = 0;
	this->maxtime = maxtime;
	for (int i = 0; i < maxtime; i++)
	{
		path.push_back(loc);//stay still all the tiem
	}
};

bool Agent::TOTP(Token &token)
{
	//update agent current location
	loc = path[token.timestep];

	vector<bool> hold(col*row, false);
	for (unsigned int i = 0; i < token.path.size(); i++)
	{
		if (i != id) hold[token.path[i][maxtime - 1]] = true;
	}
	//sort tasks by heuristic distances
		
	Task *task = NULL;
	list<Task*>::iterator n;
	for (list<Task*>::iterator it = token.tasks.begin(); it != token.tasks.end();it++)
	{
		if (hold[(*it)->start->loc] || hold[(*it)->goal->loc]) continue;
		else if (NULL == task) task = (*it);
		else if ((*it)->start->h_val[loc] < task->start->h_val[loc])
		{
			task = *it;
			n = it;
		}
	}
	if (NULL == task) // No available tasks
	{
		bool move = false;
		for (list<Task*>::iterator it = token.tasks.begin(); it != token.tasks.end(); it++)
		{
			if ((*it)->goal->loc == loc) //move away
			{
				move = true;
				break;
			}
		}
		if (move)
		{
			if (Move2EP(token))
			{
				for (int i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
				{
					token.path[id][i] = path[i];
				}
				return true;
			}
		}
		else
		{
			//std::cout << "Agent " << id << " wait at timestep " << token.timestep << endl;
			finish_time = token.timestep + 1;
			return true;
		}
		
	}

	else //take this task
	{
		
		int arrive_start = AStar(loc, token.timestep, *task->start, token, id); 
		if (arrive_start < 0)
		{
			system("PAUSE");
		}

		// try to find a path from start to goal
		//if succeed, return the arriving timestep; otherwise, return -1
		int arrive_goal = AStar(task->start->loc, arrive_start + task->start_time, *task->goal, token, id);
		if (arrive_goal < 0) //find a path to goal
		{
			system("PAUSE");
		}
		//update token path
		//positive means deliver package or waiting at goal or home, negative means moving without package				
		for (int i = token.timestep; i < token.path[id].size(); i++) //agent move with package or waiting
		{
			token.path[id][i] = path[i];
		}
		//update agent
		this->finish_time = arrive_goal + task->goal_time; //next available timestep for agent

		//show
		//std::cout << "Agent " << id << " take task " << task->start->loc << " --> " << task->goal->loc;
		//std::cout << "	Timestep " << token.timestep << "-->" << arrive_goal << endl;
							
		//update task
		task->ag = this;
		task->ag_arrive_start = arrive_start;
		task->ag_arrive_goal = arrive_goal;
		//cout << "Task " << task->start->loc << "-->" << task->goal->loc << " is done at Timestep " << task->ag_arrive_goal << endl;
		token.tasks.remove(task);

		return true;				
	}
	
	return false;

}
bool Agent::TPTR(Token &token)
{
	//a copy of token and agent
	Token token_copy(token);
	Agent agent_copy(*this);

	//update agent current location
	loc = path[token.timestep];

	//sort tasks by heuristic distances
	boost::heap::fibonacci_heap< HeuristicNode, boost::heap::compare<CompareHeuristic> > heuristic;
	for (list<Task*>::iterator it = token.tasks.begin(); it != token.tasks.end();  it++)
	{
		heuristic.push(HeuristicNode((*it)->start->loc, (*it), (*it)->start->h_val[loc]));	
	}

	while (!heuristic.empty())
	{
		//try the task with min heuristic
		HeuristicNode n = heuristic.top();
		heuristic.pop();

		if (WAIT == n.task->state          //no agent took this task before
			|| (TAKEN == n.task->state && n.task->ag_arrive_start > token.timestep + n.h_val))  // or the agent may arrive before the original agent
		{
			//check whether the start and goal are or will be occupied 
			bool occupied = false;
			for (unsigned int i = 0; i < token.path.size(); i++)
			{
				if (i == id) continue; //ignore the path of agent itself
				else if (TAKEN == n.task->state && i == n.task->ag->id) continue; //ignore the path of original agent
				else if (token.path[i][maxtime - 1] == n.task->goal->loc || token.path[i][maxtime - 1] == n.task->start->loc) // start or goal is occupied 
				{
					occupied = true;
					break;
				}
			}
			if (occupied) //if occupied, try next
			{
				//cout << "Goal " << n.task->goal->loc << " is occupied" << endl;
				continue;
			}
			
			// try to find a path to the start point
			//if succeed, return the arriving timestep; otherwise, return -1	
			int arrive_start;
			if (TAKEN == n.task->state) //try to swap
				arrive_start = AStar(loc, token.timestep, *n.task->start, token, n.task->ag->id);
			else
				arrive_start = AStar(loc, token.timestep, *n.task->start, token, id);

			if (0 <= arrive_start && (WAIT == n.task->state || arrive_start < n.task->ag_arrive_start))  //find a path to start
			{
				// try to find a path from start to goal
				//if succeed, return the arriving timestep; otherwise, return -1
				int arrive_goal;
				if (TAKEN == n.task->state) //try to swap
					arrive_goal = AStar(n.task->start->loc, arrive_start + n.task->start_time, *n.task->goal, token, n.task->ag->id);
				else
					arrive_goal = AStar(n.task->start->loc, arrive_start + n.task->start_time, *n.task->goal, token, id);

				if (arrive_goal >= 0) //find a path to goal
				{
					//update token path			
					for (int i = token.timestep; i < token.path[id].size(); i++)
					{
						token.path[id][i] = path[i];
					}

					//update agent finish_time
					this->finish_time = arrive_goal + n.task->goal_time; //next available timestep for agent

					if (WAIT == n.task->state) //no agent took this task before
					{
						//show
						//cout << "Agent " << id << " takes task " << n.task->start->loc << " --> " << n.task->goal->loc;
						//cout << "	Timestep " << token.timestep << "-->" << arrive_goal << endl;

						//update task
						n.task->state = TAKEN;
						n.task->ag = this;
						n.task->ag_arrive_start = arrive_start;
						n.task->ag_arrive_goal = arrive_goal;
						return true;
					}
					else  //swap the task
					{
						Agent* old_ag = n.task->ag;
						//show
						//cout << "Agent " << id << " swaps task " << n.task->start->loc << " --> " << n.task->goal->loc << " with Agent "<<old_ag->id;
						//cout << " at Timestep " << token.timestep << "-->" << arrive_goal << endl;

						//update task
						n.task->ag = this;
						n.task->ag_arrive_start = arrive_start;
						n.task->ag_arrive_goal = arrive_goal;

						//pass token
						if (old_ag->TPTR(token)) //swap succeed
						{
							return true;
						}
						else //give up
						{
							//cout << "Swap fails" << endl;
						}
					}

				}
				else
				{
					//cout << "Agent " << id << " fails to move from start " << n.task->start->loc << " to " << n.task->goal->loc << endl;
				}
			}
			else
			{
				/*if (arrive_start < 0)
					cout << "Agent " << id << " fails to move from current " << loc << " to " << n.task->start->loc << endl;
				else
					cout<< "Agent " << id << " fails to rob the task from " << n.task->ag->id << endl;*/
			}
		}
	}
	//agent fails to get a task
	if (token.my_endpoints[loc]) //if agent is at an endpoint now
	{
		//check whether this location is a goal of a task
		bool move = false;
		for (list<Task*>::iterator it = token.tasks.begin(); it != token.tasks.end() && !move; it++)
		{
			if ((*it)->goal->loc == loc) move = true;
		}
		//check whether agent can hold this location
		for (unsigned int t = token.timestep; t < maxtime && !move; t++)
		{
			for(unsigned int i = 0; i < token.agents.size() && !move; i++)
				if (i != id && token.path[i][t] == loc) move = true;
		}
		if (move)
		{
			if (Move2EP(token)) //move to a nearest empty endpoint
			{
				//update token
				for (int i = token.timestep; i < token.path[id].size(); i++)
				{
					token.path[id][i] = path[i];
				}
				return true;
			}
			else
			{
				//cout << "Agent " << id << " returns token" << endl;
				token.reset(token_copy);
				this->reset(agent_copy);
				return false;
			}
		}
		else //wait for one timestep
		{
			//cout << "Agent " << id << " waits at timestep " << token.timestep << endl;
			//update path
			for (int i = token.timestep + 1; i < maxtime; i++)
			{
				path[i] = path[token.timestep];
				token.path[id][i] = path[token.timestep];
			}
			finish_time = token.timestep + 1;
			return true;
		}
			
	}
	else //agent current location is not an endpoint
	{
		if (Move2EP(token))//try to move to a nearest empty endpoint
		{
			for (int i = token.timestep; i < token.path[id].size(); i++) 
			{
				token.path[id][i] = path[i];
			}
			return true;
		}
		else// the agent have no place to go, so give up swapping, return false
		{
			//cout << "Agent " << id << " return token" << endl;
			token.reset(token_copy);
			this->reset(agent_copy);
			return false;
		}
	}
}

void Agent::updatePath(const Node &goal) //update path for agent
{
	//hold the goal
	for (int i = goal.timestep + 1; i < path.size(); i++)
	{
		path[i] = goal.loc;
	}
	//update the path
	const Node* curr = &goal;
	while (curr!=NULL)
	{
		path[curr->timestep] = curr->loc;
		curr = curr->parent;
	}
}
inline void Agent::releaseClosedListNodes(map<unsigned int, Node*> &allNodes_table)
{
	map<unsigned int, Node*>::iterator it;
	for (it = allNodes_table.begin(); it != allNodes_table.end(); it++) 
	{
		delete ((*it).second);
	}
	allNodes_table.clear();
}
inline bool Agent::isConstrained(int curr_id, int next_id, int next_timestep, const Token &token, int ag_hide)
{
	// check block constraints (being in next_id at next_timestep is disallowed)
	if (!token.my_map[next_id]) return true;

	// check path constraints (the move from curr_id to next_id at next_timestep-1 is disallowed)
		for (int ag = 0; ag < token.path.size(); ag++)
		{
			if (ag == id || ag==ag_hide) continue; //ignore its path and the original agent's path
			else if (token.path[ag][next_timestep] == next_id) return true; //vertex collision
			else if (token.path[ag][next_timestep - 1] == next_id && token.path[ag][next_timestep] == curr_id) return true; //edge collision
		}
	
	return false;
}

//return final timestep if find a path, otherwise renturn -1
int Agent::AStar(int start_loc, int begin_time, const Endpoint &goal, const Token &token, int ag_hide)
{
	int goal_location = goal.loc;
	heap_open_t open_list;
	map<unsigned int, Node*> allNodes_table; //key = g_val*map_size+loc

	// generate start and add it to the OPEN list
	Node *start = new Node(start_loc, 0, goal.h_val[start_loc], NULL, begin_time, false);

	open_list.push(start);
	start->in_openlist = true;
	allNodes_table.insert(make_pair(start_loc, start)); //g_val=0 -->key=loc
	//int min_f_val = start->getFVal();


	while (!open_list.empty()) 
	{
		Node *curr = open_list.top(); 
		open_list.pop();
		curr->in_openlist = false;//move to closed list


		// check if the popped node is a goal
		if (curr->loc == goal_location) 
		{
			bool hold = true;
			//test whether the goal can be held
			for (unsigned int i = curr->timestep + 1; i < maxtime; i++)
			{
				for (unsigned int j = 0; j < token.agents.size(); j++)
				{
					if (j != id && j != ag_hide && curr->loc == token.path[j][i])
					{
						hold = false;
						break;
					}
				}
			}
			if (hold) //if it can be held, then return the path
			{
				updatePath(*curr);
				int t = curr->timestep;
				releaseClosedListNodes(allNodes_table);
				return t;
			}
			// else, keep searching
		}

		// check timestep
		if (curr->timestep >= maxtime - 1) continue;


		int next_id;
		// iterator over all possible actions
		int action[5] = {0, 1,-1,col,-col };
		for (int i = 0; i < 5;i++)
		{
			next_id = curr->loc + action[i];			
			int next_timestep = curr->timestep + 1;
			if (!isConstrained(curr->loc, next_id, next_timestep, token, ag_hide))
			{
				//compute cost to next_id via curr node
				int next_g_val = curr->g_val + 1;
				int next_h_val = goal.h_val[next_id];

				//generate (maybe temporary) node
				Node *next=new Node(next_id, next_g_val, next_h_val,curr, next_timestep, false);
				
				//try to retrieve it from the hash table
				map<unsigned int, Node* >::iterator it = allNodes_table.find(next->loc + next->g_val*row*col);
				if (it == allNodes_table.end()) //undiscover
				{  // add the newly generated node to open_list and hash table
					next->in_openlist = true;
					
					allNodes_table.insert(pair<unsigned int, Node*>(next->loc + next->g_val*row*col, next));
					open_list.push(next);
				}
				
				else //discovered
				{  
					delete(next);  // not needed anymore -- we already generated it before			
				} 	
			}  // end if case for grid not blocked
		}// end for loop that generates successors
	}  // end while loop
	// no path found

	releaseClosedListNodes(allNodes_table);
	return -1;
}
// move to an empty endpoint
bool Agent::Move2EP(Token &token)
{
	//BFS algorithm, choose the first empty endpoint to go to
	queue<Node*> Q;
	map<unsigned int, Node*> allNodes_table; //key = g_val * map_size + loc
	int action[5] = { 0, 1,-1,col,-col };
	Node *start = new Node(loc, 0, NULL, token.timestep);
	allNodes_table.insert(make_pair(loc, start)); //g_val = 0 --> key = loc
	Q.push(start);
	while (!Q.empty())
	{
		Node* v = Q.front();
		Q.pop();
		if (v->timestep >= maxtime - 1) continue; // time limit
		if (token.my_endpoints[v->loc]) // if v->loc is an endpoint
		{
			bool occupied = false;
			// check whether v->loc can be held (no collision with other agents)
			for (unsigned int t = v->timestep; t < maxtime && !occupied; t++)
			{
				for (unsigned int ag = 0; ag < token.agents.size() && !occupied; ag++)
				{
					if (ag != id && token.path[ag][t] == v->loc) occupied = true;
				}
			}
			// check whether it is a goal of a task
			for (list<Task*>::iterator it = token.tasks.begin(); it != token.tasks.end() && !occupied; it++)
			{
				if ((*it)->goal->loc == v->loc) occupied = true;
			}
			if (!occupied)// If this endpoint is empty, return path
			{
				updatePath(*v);
				finish_time = v->timestep;
				//cout << "Agent " << id << " moves to endpoint " << v->loc << endl;
				releaseClosedListNodes(allNodes_table);
				return true;
			}
			// Else, keep searching
		}
		for (int i = 0; i < 5; i++) // search its neighbor
		{
			if (!isConstrained(v->loc, v->loc + action[i], v->timestep + 1, token, id))
			{
				//try to retrieve it from the hash table
				map<unsigned int, Node* >::iterator it = allNodes_table.find(v->loc + action[i] + (v->g_val + 1)*row*col);
				if (it == allNodes_table.end()) //undiscover
				{  // add the newly generated node to hash table
					Node *u = new Node(v->loc + action[i], v->g_val + 1, v, v->timestep + 1);
					allNodes_table.insert(pair<unsigned int, Node*>(u->loc + u->g_val*row*col, u));					
					Q.push(u);
				}
			}
		}
	}
	return false;
}
