#ifndef _GAME_MAP_H
#define _GAME_MAP_H

//basic data structure
struct GM_Point;
typedef struct GM_Point Point;
typedef struct GM_Point * pPoint;

typedef pPoint p_position;
typedef pPoint pList;

struct Road_Info;
typedef struct Road_Info Road;
typedef struct Road_Info * pRdInfo;

typedef pRdInfo ri_position;
typedef pRdInfo riList;

#include <vector>
using std::vector; 

#ifndef _GAME_MAP_DATA_STRUCTURE_IMPLEMENTATION
#define _GAME_MAP_DATA_STRUCTURE_IMPLEMENTATION

struct GM_Point{
	int x, y;
	// meta data
	char _genre_;
	int  f_score;
	int  h_score;
	int  g_score;
	
	p_position father;
	int* father_index; 
	vector<int*> children_Index;
};

struct Road_Info{
	int value;
	vector<int*> road_direction;
};
#endif

//method
//include files
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

//using string just to obtain "const char string" to cater for other standard functions in the future
#include <string>
using std::string;

#include <map>
using std::map;
using std::pair;

#include <set>
using std::set;

#include <ctime>
using std::ctime;

/*
#include <exception>
using std::exception;
*/

class Orienteering{
public:
	// built-in functions
	Orienteering();

	int _init(); 

	int  main();
	int  readmap();
	void showmap();
	void _appendTO(int c[2], Point &a);
	// preprocessing
	int  build_graphic_map();
	int  jduge();
	// core algorithms
	int  heuric_estimation(Point &a, Point &b);
	int  astar_road_direct(Point &star,Point &end, int i, int j);
	void print_road_direct(Road  &r);
	// the main algorithm
	int  route_planning();
	int  generate(int[18]);
	int  routines(int status_set[18], int s, int e, int k, map<long, vector<int>>& search_cache);
	// this function has been deprecated, do not use it for this program
	void print_path_direct();
	// use this function instead
	void print_path_direct(int status_set[18], int e, int k, map<long, vector<int>>& search_cache, vector<int>& path);
private:
	int width;
	int height;
	
	// result

	// graphic information
	map<pair<int,int>, Road> neighbor_map;
	map<pair<int,int>, Point> game_map;
	p_position S;
	p_position G;
	vector<p_position> checkPoints;
};
#endif