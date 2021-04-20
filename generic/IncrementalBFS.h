//
//  IncrementalBFS.h
//  hog2 glut
//
//  Created by Nathan Sturtevant on 3/24/16.
//  Copyright © 2016 University of Denver. All rights reserved.
//

#ifndef IncrementalBFS_h
#define IncrementalBFS_h

#include <deque>
#define MAX 10000
template <class state, class action>
class IncrementalBFS {
public:
	IncrementalBFS(double initialBound = 0):bound(initialBound),initialBound(initialBound), nextBound(initialBound)
	{ResetNodeCount(); previousBound = 0;}
	bool InitializeSearch(SearchEnvironment<state,action> *env, state from, state to, Heuristic<state> *h,
						 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to, Heuristic<state> *h,
				 std::vector<state> &thePath);
	void GetPath(SearchEnvironment<state, action> *env, state from, state to,
				 std::vector<action> &thePath);
	bool DoSingleSearchStep(std::vector<state> &thePath);
	bool DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
														std::vector<state> &thePath);
    uint64_t GetNodesExpanded() { return nodesExpanded; }
	uint64_t GetNodesTouched() { return nodesTouched; }
	void ResetNodeCount()
	 { 
		nodesExpanded = nodesTouched = 0;
		newNodeCount = newNodesLastIteration = 0;
	 }
	void Reset() {bound = initialBound; nextBound = initialBound; path.clear();
		history.clear(); ResetNodeCount(); previousBound = 0; }
	void OpenGLDraw();
	void Draw(Graphics::Display &display) const;
	state GetCurrentState() const 
	{ 
		if(history.size() > 0)
			return history[1].first.currState;
		return start;
	}
	void GetCurrentPath(std::vector<state> &p) const
	{ p.clear(); for (auto &i : history) p.push_back(i.first.currState); }
	double GetCurrentFLimit() { return bound;}
	double GetNextFLimit() {return nextBound;}
	uint64_t GetNewNodesLastIteration() {
		return newNodesLastIteration;
	}
	bool expanded[MAX] = {false};
private:
	struct currSearchState{
		state currState;
		double pathCost;
		currSearchState(state s,double p) : currState(s), pathCost(p)
		{}
	};
	unsigned long nodesExpanded, nodesTouched;
	
	bool DoIteration(SearchEnvironment<state, action> *env,
					 state parent, state currState,
					 std::vector<state> &thePath, double bound, double g);
	bool DoIteration(SearchEnvironment<state, action> *env,
					 action forbiddenAction, state &currState,
					 std::vector<action> &thePath, double bound, double g);
	
	bool IterationComplete(){ return history.size() == 0;}
	void SetupIteration(double cost);
	bool StepIteration();


	std::deque<std::pair<currSearchState, int>> history;
	state start, goal;
	std::vector<state> path;
	double previousBound;
	double bound;
	double initialBound;
	double nextBound;
	SearchEnvironment<state, action> *env;
	std::vector<state> succ;
	Heuristic<state> *h;
	uint64_t newNodeCount, newNodesLastIteration;
};

template <class state, class action>
void IncrementalBFS<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
											Heuristic<state> *h,std::vector<state> &thePath)
{
	if(InitializeSearch(env,from,to,h,thePath))
		return;
	while (!DoSingleSearchStep(thePath))
	{}
}

template <class state, class action>
void IncrementalBFS<state, action>::GetPath(SearchEnvironment<state, action> *env, state from, state to,
											 std::vector<action> &thePath)
{
	
}

template<class state, class action>
bool IncrementalBFS<state, action>::InitializeSearch(SearchEnvironment<state,action> *env, state from, state to, Heuristic<state> *h,
						 std::vector<state> &thePath)
{
	printf("initialize search here\n");
	Reset();
	if(from == to)
	{
		thePath.clear();
		thePath.push_back(from);
		return true;
	}
	start = from;
	goal = to;
	this->env = env;
	this->h = h;
	SetupIteration(h->HCost(start,goal));
	return false;
}

template <class state, class action>
void IncrementalBFS<state, action>::SetupIteration(double cost)
{
	currSearchState s(start,0);
	history.push_back({s,0});
	history.push_back({s,0});
	previousBound = bound;
	bound = 12.04;
	nextBound = -1;
	path.clear();
	printf("starting iteration bound %1.1f\n",bound);
	newNodesLastIteration = newNodeCount;
	newNodeCount = 0;
}

template <class state, class action>
bool IncrementalBFS<state, action>::StepIteration()
{
	printf("stepIteration is called\n");
	if (env->GoalTest(history[1].first.currState, goal) && flesseq(history[1].first.pathCost, bound))
	{
		path.resize(0);
		for (int x = 0; x < history.size(); x++)
		{
			path.push_back(history[x].first.currState);
		}
		return true;
	}
	int depth = history[1].second;
	
	double f = history[1].first.pathCost+h->HCost(history[1].first.currState, goal);
	printf("f being used %1.1f\n",f);
	if(fgreater(f,bound))
	{
		printf("Above bound: %f/%f\n",bound,f);
		if(nextBound == -1){
			nextBound = f;
		}
		else if(fless(f,nextBound)){
			nextBound = f;
		}

	}
	if (fgreater(f, previousBound) && flesseq(f, bound))
			newNodeCount++;

	printf("Generation next set of ---Successors\n");
	env->GetSuccessors(history[1].first.currState,succ);
	nodesExpanded++;
	expanded[history[1].first.currState] = true;
	for (int x = 0; x < succ.size(); x++)
	{
		if(expanded[succ[x]] == false){
		currSearchState s(succ[x],history[1].first.pathCost+env->GCost(history[1].first.currState,succ[x]));
		history.push_back({history[1].first, depth+1});
		history.push_back({s,depth+1});
		}
	}
	for(int i = 0; i < history.size();i++){
		printf("here is %d -- ",history[i].first.currState);
	}
	history.pop_front();
	history.pop_front();
    return false;


}
template <class state, class action>
bool IncrementalBFS<state, action>::DoSingleSearchStep(std::vector<state> &thePath)
{
	// starting new iteration
	if (IterationComplete())
	{
		printf("iteration is complete, starting a new iteration\n");
		SetupIteration(nextBound);
		// pause between iterations
		return false;
	}
    printf("moving to the next step of the same iteration\n");
	return StepIteration();
}

						 

template <class state, class action>
bool IncrementalBFS<state, action>::DoSingleSearchStep(SearchEnvironment<state, action> *env, state from, state to,
														std::vector<state> &thePath)
{
/*	if (history.size() == 0)
	{
		history.push_back({from, 0});
	}
	
	this->env = env;
	int depth = history.front().second;
	env->GetSuccessors(history.front().first, succ);
	
	// Later than normal - for the purposes of drawing nicely
	if (env->GoalTest(history.front().first, to))
		return true;
	
	history.pop_front();
	for (int x = 0; x < succ.size(); x++)
	{
		history.push_back({succ[x], depth+1});
	}
	return false;
*/
 return false;
}


template <class state, class action>
void IncrementalBFS<state, action>::OpenGLDraw()
{
	for (auto x : history)
		env->OpenGLDraw(x.first.currState);
}

template <class state, class action>
void IncrementalBFS<state, action>::Draw(Graphics::Display &display) const
{
	for (auto x : history)
		env->Draw(display, x.first.currState);
}


#endif /* IncrementalBFS_h */
