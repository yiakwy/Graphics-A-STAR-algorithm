#include "GameMap.h"

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

bool operator < (const Point& a, const Point& b){
	return a.x * 100 + a.y < b.x * 100 + b.y;
}

bool operator ==(const Point& a, const Point& b){
	return a.x == b.x && a.y == b.y;
}

// initialization from standard input/output device
int Orienteering::_init(){
	// set up
	//printf("clearing containers...\n");
	this->game_map.clear();
	this->neighbor_map.clear();
	int W, H;
	scanf_s("%d %d", &W, &H);

	this->width  = W;
	this->height = H;
	// read map
	//printf("begin to read map...\n");
	clock_t start  = clock();
	int status = this->readmap();
	clock_t elapse = clock() - start;

	// captch a potential error
	if (status == -1){
		return -1;
	}

	if (this->checkPoints.size() > 18){
		return -1;
	}
	//printf("finish reading map. %g ms consumed\n", (double)elapse * 1000 / (double)CLOCKS_PER_SEC);
	//this->showmap();

	return 1;
}

void Orienteering::_appendTO(int d[2], Point &p){ 
	// will be deprecated later because of memory safty
	int * c = (int *)malloc(sizeof(int) * 2);

	c[0] = d[0];
	c[1] = d[1];

	if (c[0] >= 0 && c[0] < this->width && c[1] >= 0 && c[1] < this->height){
		p.children_Index.push_back(c);
	}
}

int Orienteering::readmap(){
	// the original position lies at the top left corner.
	for(int y = 0; y < this->height; y++){
		for(int x = 0; x < this->width; x++){
			Point p;
			p.x = x;
			p.y = y;
			
			char type;
			// these variables shoulbe reset at runtime fo A* algorithms
			p.g_score = 0;
			p.h_score = 0;
			p.f_score = 0;
			p._genre_ = type = getchar();

			// insert the game point into the map 
			// real positions where points are kept
			this->game_map.insert(pair<pair<int, int>,Point>(pair<int,int>(x, y), p));
			
			Point &q = this->game_map[std::make_pair(x,y)];
			// make jdugement
			if     (p._genre_ == 'S'){
				this->S = &q;	
			}
			else if(p._genre_ == 'G'){
				this->G = &q;
			}
			else if(p._genre_ == '@'){
				this->checkPoints.push_back(&q);
			}
			else if(p._genre_ == '\n' || p._genre_ == ' '){
				type = getchar();
				q._genre_ = type;
			}
			else if(p._genre_ == '.'  || p._genre_ == '#'){

			}
			else{
				// throw an error
				return -1;			
			}
			// neighbors initialization
			q.father = NULL;
			q.father_index = NULL;
			q.children_Index.clear();
			// tempory storage space
			int coordinates[2];
			
			coordinates[0] = q.x + 1;
			coordinates[1] = q.y;
			this->_appendTO(coordinates, q);

			coordinates[0] = q.x;
			coordinates[1] = q.y + 1;
			this->_appendTO(coordinates, q);

			coordinates[0] = q.x - 1;
			coordinates[1] = q.y;
			this->_appendTO(coordinates, q);

			coordinates[0] = q.x;
			coordinates[1] = q.y - 1;
			this->_appendTO(coordinates, q);
		}
	}
	// OK, I don't like to use c++ as they are not convenient in processing errors handling but use the very low level singnel jdugement
	return 1;
}

void Orienteering::showmap(){
	int counter = 0;
	printf("%d %d\n", this->width, this->height);
	for(int y = 0; y < this->height; y++){
		for(int x = 0; x < this->width; x++){
			char c = this->game_map[std::make_pair(x,y)]._genre_;
			if (c == '@'){
				printf("%d ", counter++);
			}
			else{
				printf("%c ", c);
			}
		}
		printf("%c", '\n');
	}
	printf("%c", '\n');
}
int Orienteering::main(){
	// since exception module depends on versions of OS upon machine your are running
	// this time I use integer to judge situations
	int status = this->_init();

	// building adjent map using A* algorithm with the simplest heuric function
	if (status == -1){
		//printf("error, return\n");
		return -1;
	}

	if (this->build_graphic_map() == -1){
		return -1;
	}

	// route_planing
	int result = this->route_planning();
	// run tsp rountines
	printf("%d", result);
	return result;
}
Orienteering::Orienteering(){

}

int Orienteering::heuric_estimation(Point &a, Point &b){
	return abs(a.x - b.x) + abs(a.y - b.y);
}

int Orienteering::astar_road_direct(Point &start, Point & end, int i, int j){
	// set might not work correctly in moving elements to another container
	set<Point> open__set;
	set<Point> close_set;

	// initialization status
	start.f_score = 0;
	start.g_score = 0;
	start.h_score = this->heuric_estimation(start, end);
	
	open__set.insert(start);

	while(!open__set.empty()){
		p_position to_bestPoint = &start;
		set<Point>::iterator it = open__set.begin();
		
		//printf("open__set:");
		for(set<Point>::iterator curr = open__set.begin(); curr != open__set.end(); curr++){
			//printf("(%d, %d, G:%d, H:%d)", curr->x, curr->y, curr->g_score, curr->h_score);
			if (it->f_score >= curr->f_score){
				it = curr;
			}
		}
		//printf("\n");
		/*
		printf("close_set:");
		for(set<Point>::iterator curr = close_set.begin(); curr != close_set.end(); curr++){
			printf("(%d, %d, G:%d, H:%d)", curr->x, curr->y, curr->g_score, curr->h_score);
		}
		printf("\n");
		*/
		// updating open_set, curr
		to_bestPoint = &this->game_map[std::make_pair(it->x, it->y)];
		open__set.erase(it);

		// updating close_set, copy the point info into the container
		close_set.insert(*to_bestPoint);

		//printf("current point (%d, %d):", to_bestPoint->x,to_bestPoint->y);

		// updating open_set, neighbors
		if (to_bestPoint->x == end.x && to_bestPoint->y == end.y){
			// to do, insert path info into neighbor map
			Road rdf;
			p_position curr = &end;
					
			//printf("\n route ...\n");
			while (curr != &start){
				//printf("->");
				int coordinates[2];
				coordinates[0] = curr->x;
				coordinates[1] = curr->y;

				rdf.road_direction.push_back(coordinates);
				//printf("(%d, %d)",curr->x, curr->y);
				// curr = curr->father;
				curr = &this->game_map[std::make_pair(curr->father_index[0], curr->father_index[1])];
			}
			//printf("->(%d, %d), cost:%d\n", start.x, start.y, to_bestPoint->g_score);

			int coordinates[2];
			coordinates[0] = start.x;
			coordinates[1] = start.y;
			rdf.road_direction.push_back(coordinates);		
			rdf.value = to_bestPoint->g_score;

			// this->print_road_direct(rdf);
			this->neighbor_map[std::make_pair(i,j)] = rdf;
			return rdf.value;
			
		}

		// check legal neibors and updating parents
		for(vector<int*>::iterator to_neighbor  = to_bestPoint->children_Index.begin();
			to_neighbor != to_bestPoint->children_Index.end(); to_neighbor++){
			
				// get a copy of original points
				Point &p = this->game_map[std::make_pair((*to_neighbor)[0],(*to_neighbor)[1])];

				if (p._genre_ == '#' || close_set.find(p) != close_set.end()){
					continue;
				}

				//printf("(%d, %d, G:%d, H:%d)", p.x, p.y, p.g_score, p.h_score);

				// already in the open list
				if (open__set.find(p) != open__set.end()){
					//printf("-in_open_set-");
					if (p.g_score > 1 + to_bestPoint->g_score){
						//printf("father:(%d, %d)->(%d, %d)-", p.father->x, p.father->y, to_bestPoint->x, to_bestPoint->y);
	
						p.father  = to_bestPoint;
						p.father_index[0] = to_bestPoint->x;
						p.father_index[1] = to_bestPoint->y;
						p.g_score = to_bestPoint->g_score + 1;
						p.f_score = p.h_score + p.g_score;
					}
					continue;
				}

				p.father  = to_bestPoint;
				// will be deprecated later because of safety
				p.father_index = (int*)malloc(sizeof(int)*2);
				p.father_index[0] = to_bestPoint->x;
				p.father_index[1] = to_bestPoint->y;
				// **
				p.g_score = to_bestPoint->g_score + 1;
				p.h_score = this->heuric_estimation(p, end);
				p.f_score = p.h_score + p.g_score;

				// copy point info into open__set
				open__set.insert(p);
		}//end-for
		//printf("%c*----------------*\n", '\n');
	}
	return -1;
}

int Orienteering::build_graphic_map(){
	
	if (this->jduge() == -1){
		return -1;
	}

	for(size_t i = 0; i < this->checkPoints.size(); i++){
		for(size_t j = i + 1; j < this->checkPoints.size(); j++){
			Point & a  = *this->checkPoints.at(i);
			Point & b  = *this->checkPoints.at(j);

			int status =  this->astar_road_direct(a, b, i, j);

			this->neighbor_map[std::make_pair(j,i)] = this->neighbor_map[std::make_pair(i,j)];

			if (status == -1){
				return -1;
			}
		}
		Road r;
		r.value = 0;
		this->neighbor_map[std::make_pair(i,i)] = r;
	}
	return 1;
}

int Orienteering::jduge(){
	int status = 0;

	status = this->astar_road_direct(*(this->S), 
									 *(this->G), 
										  -1, 
										  -1);

	if (status == -1){
		return -1;
	}

	for(size_t i  = 0; i < this->checkPoints.size(); i++){

		Point &a = *this->checkPoints.at(i);
		status = this->astar_road_direct(*(this->S), a, -1, i);

		if (status == -1){
			return -1;
		}

		status = this->astar_road_direct(a, *(this->G), i, -1);

		if (status == -1){
			return -1;
		}
	}

	return 1;
}

void Orienteering::print_road_direct(Road &r){
	for(vector<int*>::iterator it = r.road_direction.end(); it >= r.road_direction.begin(); it--){
		int * index = *it;
		printf("->(%d, %d)", index[0], index[1]);
	}
	printf("%c", '\n');
}

// f_k(x_k, s_k): f_k(S,G,s_k) = min {d(S, x_k) + f_(k-1)(x_k, G, s_(k-1))};
int  Orienteering::route_planning(){
	int status_set[18] = {0};

	map<long, vector<int>> search_cache;
	
	int result = this->routines(status_set,-1, -1, int(this->checkPoints.size()), search_cache);

	return result;
}

int  Orienteering::routines(int status_set[18], int s, int e, int k, map<long, vector<int>>& search_cache){
	if (k == 0){
		return this->neighbor_map[std::make_pair(s, e)].value;
	}
	
	long id = this->generate(status_set);

	map<long, vector<int>>::iterator it = search_cache.find(id);
	if (it != search_cache.end()){
		return it->second[0];
	}
	int distance = 1000;
	int position = 0;
	for(int i = 0; i < this->checkPoints.size(); i++){
		if (status_set[i] == 0){
			// change situation
			status_set[i] = 1;
			int d = this->neighbor_map[std::make_pair(s,i)].value + this->routines(status_set, i, e, k - 1, search_cache);
			// revert changes
			status_set[i] = 0;

			if (distance > d){
				distance = d;
				position = i;
			}
		}
	}		
	
	if (s == -1){
		// print path by s, position,status_set iterately:s->0->2->1->e
		vector<int> path;
		// this->print_path_direct(status_set, position, k, search_cache, path);
	}

	vector<int> v;
	v.push_back(distance);
	v.push_back(position);

	search_cache.insert(pair<long, vector<int>>(id, v));
	return distance;
}

int  Orienteering::generate(int status_set[18]){
	long id = 0;
	for (int i = 0; i < this->checkPoints.size(); i++){
		id = (id << 1) + status_set[i];
	}

	return id;
}

// this function has been deprecated, do not use it for this program
void Orienteering::print_path_direct(){
	p_position curr = this->S->father;
	printf("start:(%d, %d)",this->S->x, this->S->y);
	do{
		printf("->(%d, %d)", curr->x, curr->y);
		curr = curr->father;
	}while(curr != this->G);
	printf("->end:(%d, %d)", this->G->x,this->G->y);
	printf("£¬get to the end\n");
}

void Orienteering::print_path_direct(int status_set[18], int e, int k, map<long, vector<int>>& search_cache, vector<int>& path){
	int curr = e;
	printf("S");
	for (int i = 0;i < k - 1; i++){
	status_set[curr] = 1;
	printf("->%d", curr);

	long id = this->generate(status_set);

	map<long, vector<int>>::iterator it = search_cache.find(id);
	
	curr = it->second[1];
	}
	printf("->%d->G", curr);
	printf("\n");	
}

int main(int argc, char* argv[]){
	Orienteering o;
	o.main();
	return 0;
}