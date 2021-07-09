//
//  IncrementalBGS.h
//  hog2
//
//  This is an incremental version of Budgeted Graph Search - IBEX on trees with Bounded Dijkstra at the low level.
//  This code isn't meant for production - just to animate how the algorithm works as a comparison to IDA*.
//  See (Helmert et al, IJCAI 2019) for details of the algorithm
//

#ifndef IncrementalBGS_h
#define IncrementalBGS_h

#include "Heuristic.h"
#include "AStarOpenClosed.h"
bool mode = 0;
template <class state, class action>
class IncrementalBGS {
public:
	IncrementalBGS(double initialBound = 0) :bound(initialBound), initialBound(initialBound), nextBound(initialBound)
	{ ResetNodeCount(); previousBound = 0; }
	bool InitializeSearch(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
						  std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep();
	bool DoExponentialSingleSearchStep();
	uint64_t GetNodesExpanded() { return totalnodesExpanded; }
	uint64_t GetIterationNodesExpanded() { return data.nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount()
	{
		nodesExpanded = nodesTouched = 0;
		newNodeCount = newNodesLastIteration = 0;
	}
	void Reset() { bound = initialBound; nextBound = initialBound; solutionPath.clear();
		history.clear(); ResetNodeCount(); previousBound = 0; }
	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	state GetCurrentState() 
	{
		if (q.size() > 0)
			return q.Lookup(q.Peek()).data;
//			return search.back().currState;
//		if (flesseq(solutionCost, data.solutionInterval.lowerBound))
//			return goal;
//		return start;
		
	}
	void GetCurrentPath(std::vector<state> &p) const
	{ p = solutionPath; }
	double GetCurrentFLimit() { return std::min(bound, solutionCost); }
	double GetNextFLimit() { return nextBound; }
	uint64_t GetNewNodesLastIteration() { return newNodesLastIteration; }
	const uint64_t c1 = 2;
	const uint64_t c2 = 8;
	const uint64_t gamma = 2;
	const uint64_t k = 1;
	const int infiniteWorkBound = -1;
	void GetGlobalCostInterval(double &lower, double &upper)
	{ lower = data.solutionInterval.lowerBound; upper = data.solutionInterval.upperBound; }
	void GetNodeInterval(uint64_t &lower, uint64_t &upper)
	{ lower = data.nodeLB*c1; upper = data.workBound; }
	std::string stage;
	std::string fEquation;
private:
	struct costInterval {
		double lowerBound;
		double upperBound;
		costInterval &operator&=(const costInterval &i)
		{
			lowerBound = std::max(lowerBound, i.lowerBound);
			upperBound = std::min(upperBound, i.upperBound);
			return *this;
		}
	};
	struct IBEXData {
		uint64_t nodesExpanded;
		uint64_t workBound;
		uint64_t nodeLB;
		costInterval solutionInterval;
		double delta;
	};
	IBEXData data;
	struct searchData {
		double f_below;
		double f_above;
	};
	searchData sd;
//	enum kSearchStatus {
//		kGoingDown,
//		kGoingAcross
//	};
//	struct currSearchState {
//		state currState;
//		kSearchStatus status;
//	    double pathCost;
//		std::vector<state> succ;
//	};
//	std::vector<currSearchState> search;
	double solutionCost;
	bool IterationComplete() { 
		return q.OpenSize() == 0; }
	bool MainIterationComplete(){ 
		if(fc_next == DBL_MAX){
			return false;
		}
		else{
			return true;
		}
	}
	unsigned long nodesExpanded, nodesTouched;
	unsigned long nodesReexpanded;
	unsigned long totalnodesExpanded;
	unsigned long totalnodesReexpanded;
	void SetupIteration(double cost);
	void SetupExponentialBinaryIteration(double cost);
	bool StepIteration();
	void ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath);

	std::vector<std::pair<state, double>> history;
	std::vector<state> solutionPath;
	state start, goal;
	double previousBound;
	double bound;
	double initialBound;
	double nextBound;
	SearchEnvironment<state, action> *env;
	Heuristic<state> *h;
	std::vector<state> succ;
	uint64_t newNodeCount, newNodesLastIteration;
	double fc;
	double fc_next;
	std::vector <double> f_values;
	int MInodesexpanded;
	int MInodesreexpanded;
	int nodeLB;
	int MODE=0;
	struct BFHSCompare {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g, i2.g));
		}
	};

	struct BFHSCompare_f {
		bool operator()(const AStarOpenClosedData<state> &i1, const AStarOpenClosedData<state> &i2) const
		{
			return (fgreater(i1.g+i1.h, i2.g+i2.h));
		}
	};

	// Data for BFHS
	AStarOpenClosed<state, BFHSCompare> q;
	AStarOpenClosed<state, BFHSCompare_f> q_f;
	std::vector<state> neighbors;
	std::vector<uint64_t> neighborID;
	std::vector<double> edgeCosts;
	std::vector<dataLocation> neighborLoc;
	std::vector<state> solutionStates;
};

template <class state, class action>
void IncrementalBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											Heuristic<state> *h, std::vector<state> &thePath)
{
	if (InitializeSearch(e, from, to, h, thePath))
		return;
	while (!DoSingleSearchStep(thePath))
	{}
}

template <class state, class action>
void IncrementalBGS<state, action>::GetPath(SearchEnvironment<state, action> *e, state from, state to,
											std::vector<action> &thePath)
{
	
}

template <class state, class action>
bool IncrementalBGS<state, action>::InitializeSearch(SearchEnvironment<state, action> *e, state from, state to, Heuristic<state> *h,
													 std::vector<state> &thePath)
{
	printf("Search Initialization\n");
	Reset();
	if (from==to)
	{
		thePath.clear();
		thePath.push_back(from);
		return true;
	}
	start = from;
	goal = to;
	this->env = e;
	start = from;
	this->h = h;

	solutionPath.clear();


	// Setup BTS control data
/*	data.nodesExpanded = 0;
	data.workBound = infiniteWorkBound;
	data.nodeLB = 0;
	data.solutionInterval.lowerBound = h->HCost(start, goal);
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 0;
	fEquation = to_string_with_precision(h->HCost(start, goal), 0);
	solutionCost = DBL_MAX;
	SetupIteration(h->HCost(start, goal));
	stage = "NEW ITERATION";
*/
    q.Reset(env->GetMaxHash());
	q_f.Reset(env->GetMaxHash());
	fc = DBL_MAX;
	fc_next = DBL_MAX;
	f_values.clear();
	MInodesexpanded = 0;
	MInodesreexpanded = 0;
	f_values.push_back(h->HCost(start, goal));
	MODE = 0;
	nodeLB = 0;
	SetupIteration(f_values[f_values.size()-1]);
	solutionCost = DBL_MAX;
	stage = "INITIALIZE SEARCH";
	return false;
}

template <class state, class action>
void IncrementalBGS<state, action>::SetupIteration(double cost)	
{
	double f_next = -1;
	if(q_f.OpenSize() == 0){
		printf("Added the start state with hash value %llu\n",env->GetStateHash(start));
		q.AddOpenNode(start,719, 0.0, 0.0);
	}
	else{
		printf("Populating the queue from q_f\n");
		while(1){
		printf("size of the open list in q_f before is %d\n",q_f.OpenSize());
		uint64_t n = q_f.Peek();
		double g_value = q_f.Lookup(n).g;
		double h_value = q_f.Lookup(n).h;
		printf("g value is %1.5f, h value is %1.5f, total is %1.5f and cost-limit is %1.5f\n",g_value,h_value,g_value+h_value,cost);
		if(flesseq(g_value+h_value,cost)){
			state s = q_f.Lookup(n).data;
			uint64_t parentId = q_f.Lookup(n).parentID;
			printf("state hash %llu is added to q from q_f\n",env->GetStateHash(s));
			q.AddOpenNode(s,
						  env->GetStateHash(s),
						  g_value,
						  h_value,
						  parentId);
			q_f.Remove(env->GetStateHash(s));
			printf("size of the open list in q_f after is %d\n",q_f.OpenSize());
			
			
		}
		else{
			f_next = g_value + h_value;
			printf("f_next is %1.5f\n",f_next);
			break;
		}
		}
	}

	previousBound = bound;
	bound = cost;
	nextBound = f_next;
	printf("Setup iteration bound %1.1f complete\n", bound);
}


template <class state, class action>
void IncrementalBGS<state, action>::SetupExponentialBinaryIteration(double cost)
{
	printf("Setting up Exponential Binary Phase with cost %1.5f \n",cost);
	assert(MODE==1);
	data.nodesExpanded = 0;
	data.workBound = infiniteWorkBound;
	data.nodeLB = nodeLB;
	data.solutionInterval.lowerBound = cost;
	data.solutionInterval.upperBound = DBL_MAX;
	data.delta = 0;
	nodesExpanded = 0;
	nodesReexpanded = 0;
	SetupIteration(cost);
	printf("Setup Exponentail Binary Phase with %1.1 complete\n",cost);
}


template <class state, class action>
void IncrementalBGS<state, action>::ExtractPathToStartFromID(uint64_t node, std::vector<state> &thePath)
{
	thePath.clear();
	do {
		thePath.push_back(q.Lookup(node).data);
		node = q.Lookup(node).parentID;
	} while (q.Lookup(node).parentID != node);
	thePath.push_back(q.Lookup(node).data);
}

template <class state, class action>
bool IncrementalBGS<state, action>::StepIteration()
{
	uint64_t nodeid = q.Close();
	// Check if we are done
	if (env->GoalTest(q.Lookup(nodeid).data, goal))
	{
		printf("Goal has been found\n");
		ExtractPathToStartFromID(nodeid, solutionPath);
		std::reverse(solutionPath.begin(), solutionPath.end());
		solutionCost = env->GetPathLength(solutionPath);
		return true;
	}
	if(q.Lookup(nodeid).reopened == true){
		totalnodesReexpanded++;
		nodesReexpanded++;
	}
	else{
		totalnodesExpanded++;
		nodesExpanded++;
	}
	
	neighbors.resize(0);
	edgeCosts.resize(0);
	neighborID.resize(0);
	neighborLoc.resize(0);
	std::vector<uint64_t> nodes = q.ClosedNodesTillNow();
/*	printf("the closed nodes are \n");
	for(int i = 0 ; i < nodes.size();i++){
		printf("%llu node with hash value %llu\n",nodes[i],env->GetStateHash(q.Lookup(nodes[i]).data));
	}
*/
	std::cout << "Expanding: " << env->GetStateHash(q.Lookup(nodeid).data) <<std::endl;
	
	env->GetSuccessors(q.Lookup(nodeid).data, neighbors);
	
	// 1. load all the children
	for (unsigned int x = 0; x < neighbors.size(); x++)
	{
		uint64_t theID;
		neighborLoc.push_back(q.Lookup(env->GetStateHash(neighbors[x]), theID));
		switch (neighborLoc[x])
		{
		case kClosedList :
			printf("%llu -- %llu -- kclosedlist child\n",env->GetStateHash(neighbors[x]),theID);
			break;
		case kOpenList:
			printf("%llu -- %llu -- kOpenlist child\n",env->GetStateHash(neighbors[x]),theID);
			break;
		case kNotFound :
			printf("%llu -- %llu -- knotFound child\n",env->GetStateHash(neighbors[x]),theID);
			break;
		}
		neighborID.push_back(theID);
		edgeCosts.push_back(env->GCost(q.Lookup(nodeid).data, neighbors[x]));
	}
	
	// iterate again updating costs and writing out to memory
	for (int x = 0; x < neighbors.size(); x++)
	{
		double childGCost = q.Lookup(nodeid).g+edgeCosts[x];
		double childFCost = childGCost+h->HCost(neighbors[x], goal);
		printf("ChildFcost is %1.5f, current bound is %1.5f and next bound is %1.5f\n",childFCost,bound,nextBound);
		dataLocation d_1 ;
		uint64_t id;
		d_1 = q.Lookup(env->GetStateHash(neighbors[x]),id);
		if (fgreater(childFCost, bound))
		{
			if (nextBound == -1)
				nextBound = childFCost;
			else if (fless(childFCost, nextBound) && d_1 != kClosedList)
				nextBound = childFCost;
			else if(d_1 == kClosedList){
				continue;
			}
			sd.f_above = std::min(sd.f_above, childFCost);
				
			dataLocation d ;
			uint64_t id;
			d = q_f.Lookup(env->GetStateHash(neighbors[x]),id);
			if(d == kNotFound){
				q_f.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  h->HCost(neighbors[x], goal),
							  nodeid);
				printf("state hash is %llu is added to q_f\n",env->GetStateHash(neighbors[x]));
			}
			else if(d == kOpenList){
				printf("State is already in q_f\n");
					if (fless(childFCost, q_f.Lookup(id).g+h->HCost(neighbors[x], goal)))
			{
				q_f.Lookup(id).parentID = nodeid;
				q_f.Lookup(id).g = childGCost;
				q_f.KeyChanged(id);
				printf("state hash is %llu is updated in q_f\n",env->GetStateHash(neighbors[x]));
			}
			}
			
			continue;
		}
		
		switch (neighborLoc[x])
		{
			case kClosedList:
				if (fless(childGCost, q.Lookup(neighborID[x]).g))
				{
					q.Lookup(neighborID[x]).parentID = nodeid;
					q.Lookup(neighborID[x]).g = childGCost;
					q.Reopen(neighborID[x]);
				}
				break;
			case kOpenList:
				if (fless(childGCost, q.Lookup(neighborID[x]).g))
				{
					q.Lookup(neighborID[x]).parentID = nodeid;
					q.Lookup(neighborID[x]).g = childGCost;
					q.KeyChanged(neighborID[x]);
				}
				break;
			case kNotFound:
			{
				q.AddOpenNode(neighbors[x],
							  env->GetStateHash(neighbors[x]),
							  childGCost,
							  0,
							  nodeid);
			}
		}
	}
	printf("Step iteration over\n");
	return false;
	
//	if (search.back().status == kGoingDown)
//	{
//		double f = search.back().pathCost+h->HCost(search.back().currState, goal);
//		// exceeded path cost bound
//		if (fgreater(f, bound))
//		{
//			//printf("Above bound: %f/%f\n", bound, f);
//			sd.f_above = std::min(sd.f_above, f);
//			if (nextBound == -1)
//				nextBound = f;
//			else if (fless(f, nextBound))
//				nextBound = f;
//
//			search.pop_back();
//			return false;
//		}
//		if (fgreater(f, solutionCost))
//		{
//			search.pop_back();
//			return false;
//		}
//		if (flesseq(f, bound))
//			sd.f_below = std::max(sd.f_below, f);
//		//			data.solutionInterval.upperBound = std::max(data.solutionInterval.upperBound, f);
//
//		if (fgreater(f, previousBound) && flesseq(f, bound))
//			newNodeCount++;
//
//		// continue search
//		//printf("Generating next set of successors\n");
//		search.back().status = kGoingAcross;
//		env->GetSuccessors(search.back().currState, search.back().succ);
//		nodesExpanded++;
//		for (int x = 0; x < search.back().succ.size(); x++)
//		{
//			if (search.size() > 1 && search.back().succ[x] == search[search.size()-2].currState)
//			{
//				search.back().succ.erase(search.back().succ.begin()+x);
//				x--;
//			}
//		}
//
//		// reverse and then handle them from back to front
//		std::reverse(search.back().succ.begin(), search.back().succ.end());
//		//return false;
//	}
//
//	if (search.back().status == kGoingAcross)
//	{
//		// no more succ to go down - go up
//		if (search.back().succ.size() == 0)
//		{
//			//printf("Out of successors\n");
//			search.pop_back();
//			return false;
//		}
//
//		//printf("Taking next successors\n");
//		// going down - generate next successor
//		search.resize(search.size()+1);
//		auto &s = search[search.size()-2];
//		search.back().currState = s.succ.back();
//		search.back().status = kGoingDown;
//		search.back().pathCost = s.pathCost+env->GCost(s.currState, s.succ.back());
//		s.succ.pop_back();
//		return false;
//	}
//	assert(false);
//	return false;
	
}

template <class state, class action>
bool IncrementalBGS<state, action>::DoSingleSearchStep()
{
	printf("Single Search Step\n");
	bool s;
	if(!MainIterationComplete()){
		if(MODE == 0){
			printf("Reexpansion bound is %llu\n",k*nodeLB);
			if(MInodesreexpanded <= k*nodeLB){
				if(!IterationComplete()){
					int temp1 = nodesExpanded;
					int temp2 = nodesReexpanded;
					printf("Call to StepIteration()\n");
					s = StepIteration();
					MInodesexpanded += (nodesExpanded - temp1);
					MInodesreexpanded += (nodesReexpanded -temp2);
					return s;
				}
				else{
					printf("Iteration is over for %1.5f\n",f_values[f_values.size()-1]);
					if(MInodesexpanded >= (c1-1)*nodeLB){
						fc_next = f_values[f_values.size()-1];
						printf("Main Iteration is over for %1.5f\n",f_values[f_values.size()-1]);
						return false;
					}
					else {
						f_values.push_back(nextBound);
						SetupIteration(f_values[f_values.size()-1]);
						return false;
					}
				}
			}
			else{
				q.Reset(env->GetMaxHash());
				q_f.Reset(env->GetMaxHash());
				MODE = 1;
				double nextCost = f_values[0];
				f_values.clear();
				f_values.push_back(nextCost);
				SetupExponentialBinaryIteration(f_values[f_values.size()-1]);
				return false;
			}
		}
		else{
			return DoExponentialSingleSearchStep();
		}
	}
	else{
		printf("Main iteration is complete\n");
		printf("MInodesExpanded is %d, MInodesReexpanded is %d, total node expansions %d and total re-expansions is %d \n",MInodesexpanded,MInodesreexpanded,nodesExpanded,nodesReexpanded);
		nodeLB = nodesExpanded;
		printf("Total nodes lower bound is %d\n",(c1-1)*nodeLB);
		MInodesexpanded = 0;
		MInodesreexpanded = 0;
		if(fc == DBL_MAX){
			printf("Grew Exponentially between 0 and %1.5f\n",fc_next);
		}
		else{
			printf("Grew Exponentially between %1.5f and %1.5f\n",fc,fc_next);
		}
		
		printf("The vector f_values is as follows :\n");
		for(int i = 0; i < f_values.size();i++){
			printf("%1.5f ",f_values[i]);
		}
		printf("\n");
		fc = fc_next;
		f_values.clear();
		f_values.push_back(nextBound);
		fc_next = DBL_MAX;
		SetupIteration(f_values[f_values.size()-1]);
		return false;
	}
	printf("Single Search Step over\n");
}


template <class state, class action>
bool IncrementalBGS<state, action>::DoExponentialSingleSearchStep()
{
	printf("Exponential Binary Single Search Step\n");
	bool s;
	if (!IterationComplete() && data.nodesExpanded < data.workBound)
	{
		uint64_t tmp = nodesExpanded;
		s = StepIteration();
		data.nodesExpanded += nodesExpanded-tmp;
		return s;
	}
	
	// Invariant - iteration complete *or* exceeded work limit
	// last search completed - set the interval from this search
	costInterval v;
	if (data.nodesExpanded >= data.workBound)
	{
		v.lowerBound = 0;
		v.upperBound = sd.f_below;
	}
	else if (solutionCost != DBL_MAX && fgreatereq(sd.f_below, solutionCost))
	{
		v.lowerBound = solutionCost;
		v.upperBound = solutionCost;
	}
	else {
		v.lowerBound = sd.f_above;
		v.upperBound = DBL_MAX;
	}
	data.solutionInterval &= v;
	
	printf("--> Iteration complete - %llu expanded; target [%llu, %llu)\n", data.nodesExpanded, c1*data.nodeLB, c2*data.nodeLB);
	// Move to next iteration
	if (data.nodesExpanded >= c1*data.nodeLB && data.workBound == infiniteWorkBound)
	{
		printf("Expanded %llu - needed at least %llu\n", data.nodesExpanded, c1*data.nodeLB);
		if (data.solutionInterval.lowerBound == DBL_MAX) // No new nodes in iteration
			printf("[HIT]--Critical f in [%1.5f, %1.5f]\n", solutionCost, solutionCost);
		else
			printf("[HIT]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
	    nodesExpanded = data.nodesExpanded;
		nodesReexpanded = 0;
		data.solutionInterval.upperBound = DBL_MAX;
		fc_next = f_values[f_values.size()-1];
		return false;
	}
		
	// Check if bounds are equal or whether we fell within the node bounds
	if (!
		(fequal(data.solutionInterval.upperBound, data.solutionInterval.lowerBound) ||
		 (data.nodesExpanded >= c1*data.nodeLB && data.nodesExpanded < c2*data.nodeLB)
		 ))
	{
		if (data.solutionInterval.upperBound == DBL_MAX)
			printf("    ]--Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
		else
			printf("    ]--Critical f in [%1.5f, %1.5f]\n", data.solutionInterval.lowerBound, data.solutionInterval.upperBound);
			
		double nextCost;
		if (data.solutionInterval.upperBound == DBL_MAX)
		{
			nextCost = data.solutionInterval.lowerBound+pow(gamma, data.delta);
			fEquation = to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+to_string_with_precision(gamma, 0)+"^"+to_string_with_precision(data.delta, 0)+"="+to_string_with_precision(nextCost, 0);
			stage = "EXP";
		}
		else {
			nextCost = (data.solutionInterval.lowerBound+data.solutionInterval.upperBound)/2.0;
			fEquation = "("+to_string_with_precision(data.solutionInterval.lowerBound, 0)+"+"+ to_string_with_precision(data.solutionInterval.upperBound, 0)+")/2"+"="+to_string_with_precision(nextCost, 0);
			stage = "BIN";
		}
		q.Reset(env->GetMaxHash());
		q_f.Reset(env->GetMaxHash());
		data.delta += 1;
		data.workBound = c2*data.nodeLB;
		f_values.push_back(nextCost);
		data.nodesExpanded = 0;
		SetupIteration(f_values[f_values.size()-1]);
		printf("-> Starting iteration with f: %f; node limit: %llu\n", nextCost, data.workBound);

		return false;
	}

	if (data.solutionInterval.lowerBound == DBL_MAX)
		printf("[HIT]---Critical f in [∞, ∞]\n");
	else
		printf("[HIT]---Critical f in [%1.5f, ∞]\n", data.solutionInterval.lowerBound);
	// finished iteration with current bound - ready to start next IBEX/BTS iteration
    fc_next = f_values[f_values.size()-1];
	nodesExpanded = data.nodesExpanded;
	nodesReexpanded = 0;
	MODE = 0;
	return false;
}

template <class state, class action>
void IncrementalBGS<state, action>::Draw(Graphics::Display &disp) const
{
	double transparency = 1.0;
	if (q.size() == 0)
		return;
	uint64_t top = -1;
	//	double minf = 1e9, maxf = 0;
	if (q.OpenSize() > 0)
	{
		top = q.Peek();
	}
	for (unsigned int x = 0; x < q.size(); x++)
	{
		const auto &data = q.Lookat(x);
		if (x == top)
		{
			env->SetColor(1.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kOpenList) && (data.reopened))
		{
			env->SetColor(0.0, 0.5, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kOpenList)
		{
			env->SetColor(0.0, 1.0, 0.0, transparency);
			env->Draw(disp, data.data);
		}
		else if ((data.where == kClosedList) && (data.reopened))
		{
			env->SetColor(0.5, 0.0, 0.5, transparency);
			env->Draw(disp, data.data);
		}
		else if (data.where == kClosedList)
		{
			//			if (top != -1)
			//			{
			//				env->SetColor((data.g+data.h-minf)/(maxf-minf), 0.0, 0.0, transparency);
			//			}
			//			else {
			if (data.parentID == x)
				env->SetColor(1.0, 0.5, 0.5, transparency);
			else
				env->SetColor(1.0, 0.0, 0.0, transparency);
			//			}
			env->Draw(disp, data.data);
		}
	}
	env->SetColor(1.0, 0.5, 1.0, 0.5);
	env->Draw(disp, goal);}

template <class state, class action>
void IncrementalBGS<state, action>::OpenGLDraw()
{
}


#endif /* IncrementalBGS_h */
