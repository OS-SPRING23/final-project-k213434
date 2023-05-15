#include<iostream>
#include<bits/stdc++.h>
#include<pthread.h>
#include<set>
#define pii pair<int,int>
#define ppp pair<pair<int,int>,pair<int,int>>
#include<cmath>
#define forn(a,n) for(int i = a; i < n; ++i)
#define ROW 9
#define COL 10
using namespace std;


void printPath();
pthread_mutex_t openListLock;
pthread_mutex_t myMutex;
class Cell
{
	public:
	int parentI, parentJ;
	double f,g,h;

	Cell()
	{
		f = FLT_MAX;
		g = FLT_MAX;
		h = FLT_MAX;
		parentI = -1;
		parentJ = -1;
	}
};

vector< vector<int> > grid  = {
			{ 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 1 },
            { 1, 1, 1, 0, 1, 1, 0, 1, 0, 1 },
            { 0, 0, 1, 0, 1, 0, 0, 0, 0, 1 },
            { 1, 1, 1, 0, 1, 1, 1, 0, 1, 0 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 0, 0 },
            { 1, 0, 0, 0, 0, 1, 0, 0, 0, 1 },
            { 1, 0, 1, 1, 1, 1, 0, 1, 1, 1 },
            { 1, 1, 1, 0, 0, 0, 1, 0, 0, 1 }};

set<pair<int,pair<int,int> > > openList;
vector<vector<bool> > closedList(ROW, vector<bool>(COL,false));
vector< vector<Cell> > cells(ROW, vector<Cell>(COL));
pair<int,int> start;
pair<int,int> goal;

// Using Euclidean Distance to calculate the heuristic value
double calculateHeuristicValue(const pair<int,int> &current, const pair<int,int> &destination)
{
	double a = current.first - destination.first;
	double b = current.second - destination.second;
	return (double)sqrt(a * a + b * b);
}

bool isValid(int x, int y)
{
	// out of grid
    if( (x < 0 || x >= ROW) || (y < 0 || y >= COL) )
		return false;
	// Cell is in the grid
	return true;
}

bool isObstacle(const vector< vector<int> > &grid, int x, int y)
{
	// cell is an obstacle
	if(grid[x][y] == 0)
		return true;
	// cell is not an obstacle
	return false;
}

bool isDestination(int x, int y, const pair<int,int>& goal)
{
	// reached at goal cell
	if(x == goal.first && y == goal.second)
		return true;
	// not at goal cell	
	return false;
}

// destination found
bool isGoalFound = false;

// thread executes this function
void *runner(void *arg)
{
	ppp pm = *((ppp *)arg);
	pthread_mutex_unlock(&myMutex);	
	pii curr = pm.first;
	pii next = pm.second;
	
	int i = next.first;
	int j = next.second;
	double newF, newG, newH;
	if(isValid(i,j))
	{
		if(isDestination(i,j,goal))
		{
			cells[i][j].parentI = curr.first;
			cells[i][j].parentJ = curr.second;
			cout << "The Goal is found\n\n";
			isGoalFound = true;
			printPath();
			exit(0);
		}
		else if(closedList[i][j] == false && !isObstacle(grid,i,j))
		{
			if(curr.first - next.first == 0 || curr.second - next.second == 0)
				newG = cells[curr.first][curr.second].g + 1.0;
			else
				newG = cells[curr.first][curr.second].g + 1.414;
			newH = calculateHeuristicValue(next,goal);
			newF = newG + newH;

			
			if(cells[i][j].f == FLT_MAX || cells[i][j].f > newF)
			{
				pthread_mutex_lock(&openListLock);
				openList.insert(make_pair(newF,make_pair(i,j)));
				pthread_mutex_unlock(&openListLock);
				cells[i][j].g = newG;
				cells[i][j].h = newH;
				cells[i][j].f = newF;
				cells[i][j].parentI = curr.first;
				cells[i][j].parentJ = curr.second;
			}		
		}
	}
	pthread_exit(0);
}


void printPath()
{
	printf("\nThe Path is \n");
    int row = goal.first;
    int col = goal.second;
 
	cout << "\n\n";
    stack<pii> Path;
    
    while (!(cells[row][col].parentI == row
             && cells[row][col].parentJ == col)) {
        Path.push(make_pair(row, col));
        int temp_row = cells[row][col].parentI;
        int temp_col = cells[row][col].parentJ;
        row = temp_row;
        col = temp_col;
    }
 
    Path.push(make_pair(row, col));
    while (!Path.empty()) {
        pair<int, int> p = Path.top();
        Path.pop();
        printf("-> (%d,%d) ", p.first, p.second);
    }
    return;
}

void aStarSearchAlgorithm(const vector< vector<int> >& grid, const pair<int,int> &start, const pair<int,int> &goal)
{
	// Checking if the start node is valid or not
    if(!isValid(start.first,start.second))
	{
		cout << "Source is not valid it is out of the grid" << endl;
		return;
	}

	// Checking if the destination node is valid or not
	if(!isValid(goal.first,goal.second))
	{
		cout << "Destination is not valid it is out the grid" << endl;
		return;
	}

	// Checking if the source node is blocked or not
	if(isObstacle(grid,start.first,start.second))
	{
		cout << "Source is not valid it is an obstacle" << endl;
		return;
	}

	// Checking if the destination node is blocked or not
	if(isObstacle(grid,goal.first,goal.second))
	{
		cout << "Destination is not valid it is an obstacle" << endl;
		return;
	}

	// Checking if start and goal node are the current
	if(isDestination(start.first,start.second,goal))
	{
		cout << "We are already at the destination" << endl;
		return;
	}

	for(int i = 0;i < ROW; ++i)
	{
		for(int j = 0;j < COL; ++j)
		{
			cells[i][j].f = FLT_MAX;
			cells[i][j].g = FLT_MAX;
			cells[i][j].h = FLT_MAX;
			cells[i][j].parentI = -1;
			cells[i][j].parentJ = -1;
		}
	}
	// (f,(i,j)) f is cost function value of a cell/node and (i,j) is the position of a cell/node
	
	int x = start.first;
	int y = start.second;
	openList.insert(make_pair(0.0,make_pair(x,y)));
	cells[x][y].parentI = x;
	cells[x][y].parentJ = y;
	cells[x][y].f = 0;
	cells[x][y].g = 0;
	cells[x][y].h = 0;

	// 8 moves (E,W,S,N,SE,NE,SW,NW)	
	int movesX[8] = {-1,1,0,0,-1,-1,1,1};
	int movesY[8] = {0,0,1,-1,1,-1,1,-1};
	
	while(!openList.empty())
	{
		pair<int,pii> p = *(openList.begin());
		// popping the node from the set
		openList.erase(openList.begin());
		x = p.second.first;
		y = p.second.second;	
//		cout << "f = " << p.first << " x = " << x << " y = " << y << endl;

		// cell is visited	
		closedList[x][y] = true;
	
		pii curr = make_pair(x,y);
		pthread_t threads[8];
		vector<ppp> arr;
		for(int i = 0;i < 8; ++i)
		{
			int X = x + movesX[i];
			int Y = y + movesY[i];
			pii next = make_pair(X,Y);
			ppp pr = make_pair(curr,next);
			arr.push_back(pr);
			
		}
		
		//Visiting all 8 neighbours of a cell
		for(int i = 0;i < 8; ++i)
		{		
			pthread_mutex_lock(&myMutex);
			pthread_create(&threads[i],0,runner,(void *)(&arr[i]));
		}


		for(int i = 0;i < 8; ++i)
		{
			pthread_join(threads[i],0);
		}
	}
	
	if(!isGoalFound)
	{
		cout << "\nFailed to reach goal state\n";
	}
}



int main()
{
	cout << "The maze is: \n";
	
	for(int i = 0; i < grid.size(); ++i)
	{
		for(int j = 0; j < grid[0].size(); ++j)
		{
			cout << grid[i][j] << " ";
		}
		cout << endl;
	}
	
	
	cout << endl << endl;
	int srcX, srcY;
	int goalX, goalY;
	cout << "Enter the position of source for e.g (8,0): ";
	cin >> srcX >> srcY;
	cout << "Enter the position of goal/destination for e.g (0,0): ";
	cin >> goalX >> goalY;
	
    start = make_pair(srcX,srcY);    
    goal = make_pair(goalX,goalY);
	pthread_mutex_init(&openListLock,0);
	pthread_mutex_init(&myMutex,0);
	
    aStarSearchAlgorithm(grid,start,goal);
	return 0;
}
