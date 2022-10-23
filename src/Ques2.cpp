#include <iostream>
#include <deque>
#include <cmath>
#include <vector>
#include <algorithm>
#include <string>
#include <unordered_map>
#include <queue>
#include <set>
#include <random>
#include <fstream>
#include <chrono>
#include "matplotlibcpp.h"

#define _USE_MATH_DEFINES

namespace plt = matplotlibcpp;

struct pointVec{
    std::vector<float> x;
    std::vector<float> y;
};
struct point{
    float x = 0.0;
    float y = 0.0;
};

class node{
    public:
    int val;
    float g = 0.0;
    float cost = INFINITY;
    float time = 0.0;
    node* parent;
    std::vector<node*> child;
    point coordinate;
    bool operator==(node rhs) const {return this->val == rhs.val;}
};

class unionNode{
    public:
    std::vector<node*> ExtendedSS;
};

bool equal(node* lhs, node* rhs){
    return lhs -> val == rhs -> val?true:false;
}

struct compare{
    bool operator()(node* a, node* b){
        return a->cost > b->cost;
    }
};

struct pathInfo{
    std::vector<node*> path;
    float length;
};

class graph{
    public:
        std::unordered_map<node*, std::vector<std::pair<node*, float>>> adjList;
        std::unordered_map<node*, node*> backpointer;
        graph(){};
        void add_edge(node* a, node* b, float c);
};

class obstacle{
    public:
        // The obstacle class is supposed to represent one obstacle 
        // Assumed that points given in acw direction as required by obstacle definitions
        std::vector<std::pair<float,float>> obs_points;

        obstacle(std::vector<std::pair<float,float>> points){
            for(std::pair<float,float> p : points){
                obs_points.push_back(p);
            }
        }
        void plot();
        bool CheckIntersectionWObs(point pos);
};

class RRT{
    public:
        std::unordered_map<int, std::unordered_map<float,node*>> storage;
        std::vector<obstacle>   Union_obstacle;
        std::vector<node*>      sampledNodes;
        float                   goalBias;
        float                   deltaT;
        float                   goalRegion;
        float                   interAgentDist;
        float                   tStep = 0.1;
        point                   min_vals;
        point                   max_vals;
        node                    start;
        node                    goal;
        int                     treeCount = 0;
        int                     maxIters;
        int                     rCount;

        RRT(float min_x, float min_y, float max_x, float max_y, std::vector<obstacle> uObs, point start, point goal, float goalBias, float deltaT, float goalRegion, int maxIters, std::unordered_map<int, std::unordered_map<float,node*>> storage){
            this->min_vals.x = min_x; this->min_vals.y = min_y;
            this->max_vals.x = max_x; this->max_vals.y = max_y;
            this->goalBias   = goalBias; this->goalRegion = goalRegion;
            this->deltaT     = deltaT; 
            this->Union_obstacle = uObs;
            this->start.coordinate = start; this->start.val = 0; this->sampledNodes.push_back(&this->start);
            this->goal.coordinate = goal; this->goal.val = 1;
            this->maxIters = maxIters;
            this->storage  = storage;
        }   

        bool ExtendRRT();
        pathInfo RRTpath();
        void plotGraph();
        bool checkEdge(point p1, point p2);
        bool checkInObstacle(point p);
        bool checkAgents(std::unordered_map<int, std::unordered_map<float,node*>> storage, float t, point p);
};


float angle_wrap(float angle);
void part_a(int n_agents);
void part_a_hundred(int n_agents);
float dist(pointVec p1, pointVec p2);
float dist(point p1, point p2);
std::unordered_map<int, std::unordered_map<float,node*>> insertPath(std::vector<node*> path, std::unordered_map<int, std::unordered_map<float,node*>> storage, int robotNo);

int main(){
    for(int i = 2; i < 7; i++){
        part_a(i);
    }
    for(int  i = 2; i < 7; i++){
        part_a_hundred(i);
    }
    return 0;
}

void part_a(int n_agents){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{3.5,5.5},std::pair<float,float>{6.5,5.5},std::pair<float,float>{6.5,7.5},std::pair<float,float>{3.5,7.5},std::pair<float,float>{3.5,5.5}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{3.5,5.5},std::pair<float,float>{5.5,5.5},std::pair<float,float>{5.5,10.5},std::pair<float,float>{3.5,10.5},std::pair<float,float>{3.5,5.5}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{3.5,8.5},std::pair<float,float>{6.5,8.5},std::pair<float,float>{6.5,10.5},std::pair<float,float>{3.5,10.5},std::pair<float,float>{3.5,8.5}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{9.5,5.5},std::pair<float,float>{12.5,5.5},std::pair<float,float>{12.5,7.5},std::pair<float,float>{9.5,7.5},std::pair<float,float>{9.5,5.5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{10.5,5.5},std::pair<float,float>{12.5,5.5},std::pair<float,float>{12.5,10.5},std::pair<float,float>{10.5,10.5},std::pair<float,float>{10.5,5.5}};
    std::vector<std::pair<float,float>> p5  = {std::pair<float,float>{9.5,8.5},std::pair<float,float>{12.5,8.5},std::pair<float,float>{12.5,10.5},std::pair<float,float>{9.5,10.5},std::pair<float,float>{9.5,8.5}};

    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);
    obstacle obs5(p5);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    Union_obstacle.push_back(obs5);

    pointVec start; start.x = std::vector<float>{2,2,8,2,11,11}; start.y = std::vector<float>{2,14,14,8,2,14};
    pointVec goal ; goal.x  = std::vector<float>{14,14,8,14,5,5}; goal.y  = std::vector<float>{14,2,2,8,14,2};

    std::unordered_map<int, std::unordered_map<float,node*>> pathMaps;
    std::vector<std::vector<node*>> paths;
    std::vector<RRT> rrtPlanners;

    for(int i = 0; i < n_agents; i++){
        point s; s.x = start.x[i]; s.y = start.y[i];
        point g; g.x = goal.x[i];  g.y = goal.y[i];
        rrtPlanners.push_back(RRT(0, 0, 16, 16, Union_obstacle, s, g, 0.05, 0.5, 0.25, 7500, pathMaps));

        bool pathFound = rrtPlanners[i].ExtendRRT();

        if(pathFound){
            std::cout << "----------------Path Found for: " << i << std::endl;
            pathInfo path = rrtPlanners[i].RRTpath();
            paths.push_back(path.path);
            for(node* n:path.path){
                std::cout << n->coordinate.x << " " << n->coordinate.y << std::endl;
            }
            pathMaps = insertPath(path.path, pathMaps, i);
        }
    }



    for(obstacle o:Union_obstacle){
        o.plot();
    }
    
    std::vector<float> px, py;
    std::vector<std::string> s{"k","b","g","r","c","y"};
    int cc = 0;
    for(std::vector<node*> n: paths){
        for(node* nn: n){
            px.push_back(nn->coordinate.x); py.push_back(nn->coordinate.y);
        }
        plt::plot(px, py, "*" + s[cc%6]);
        px.clear(); py.clear();
        cc++;
    }

    for(int i = 0; i < n_agents; i++){
        plt::named_plot("Starting Position of Agent " + std::to_string(i),std::vector<float>{start.x[i]},std::vector<float>{start.y[i]},"^"+s[i%6]);
        plt::named_plot("Goal Position of Agent " + std::to_string(i),std::vector<float>{goal.x[i]},std::vector<float>{goal.y[i]},"v"+s[i%6]);
    }

    plt::legend();
    plt::xlabel("X axes(m)");
    plt::ylabel("Y axes(m)");
    plt::show();
}

void part_a_hundred(int n_agents){
    std::vector<std::pair<float,float>> p0  = {std::pair<float,float>{3.5,5.5},std::pair<float,float>{6.5,5.5},std::pair<float,float>{6.5,7.5},std::pair<float,float>{3.5,7.5},std::pair<float,float>{3.5,5.5}};
    std::vector<std::pair<float,float>> p1  = {std::pair<float,float>{3.5,5.5},std::pair<float,float>{5.5,5.5},std::pair<float,float>{5.5,10.5},std::pair<float,float>{3.5,10.5},std::pair<float,float>{3.5,5.5}};
    std::vector<std::pair<float,float>> p2  = {std::pair<float,float>{3.5,8.5},std::pair<float,float>{6.5,8.5},std::pair<float,float>{6.5,10.5},std::pair<float,float>{3.5,10.5},std::pair<float,float>{3.5,8.5}};
    std::vector<std::pair<float,float>> p3  = {std::pair<float,float>{9.5,5.5},std::pair<float,float>{12.5,5.5},std::pair<float,float>{12.5,7.5},std::pair<float,float>{9.5,7.5},std::pair<float,float>{9.5,5.5}};
    std::vector<std::pair<float,float>> p4  = {std::pair<float,float>{10.5,5.5},std::pair<float,float>{12.5,5.5},std::pair<float,float>{12.5,10.5},std::pair<float,float>{10.5,10.5},std::pair<float,float>{10.5,5.5}};
    std::vector<std::pair<float,float>> p5  = {std::pair<float,float>{9.5,8.5},std::pair<float,float>{12.5,8.5},std::pair<float,float>{12.5,10.5},std::pair<float,float>{9.5,10.5},std::pair<float,float>{9.5,8.5}};

    obstacle obs(p0);
    obstacle obs1(p1);
    obstacle obs2(p2);
    obstacle obs3(p3);
    obstacle obs4(p4);
    obstacle obs5(p5);

    std::vector<obstacle> Union_obstacle;

    Union_obstacle.push_back(obs);
    Union_obstacle.push_back(obs1);
    Union_obstacle.push_back(obs2);
    Union_obstacle.push_back(obs3);
    Union_obstacle.push_back(obs4);
    Union_obstacle.push_back(obs5);

    pointVec start; start.x = std::vector<float>{2,2,8,2,11,11}; start.y = std::vector<float>{2,14,14,8,2,14};
    pointVec goal ; goal.x  = std::vector<float>{14,14,8,14,5,5}; goal.y  = std::vector<float>{14,2,2,8,14,2};

    std::ofstream length; length.open("rrt_size_" + std::to_string(n_agents) + ".csv");
    std::ofstream time; time.open("time_rrt_" + std::to_string(n_agents) + ".csv");
    std::ofstream valid; valid.open("valid_rrt_" + std::to_string(n_agents) + ".csv");

    for(int i = 0; i < 100; i++){
        std::cout << i << std::endl;
        std::unordered_map<int, std::unordered_map<float,node*>> pathMaps;
        std::vector<std::vector<node*>> paths;
        std::vector<RRT> rrtPlanners;
        bool check = true;
        auto expStart = std::chrono::high_resolution_clock::now();
        for(int i = 0; i < n_agents; i++){
            point s; s.x = start.x[i]; s.y = start.y[i];
            point g; g.x = goal.x[i];  g.y = goal.y[i];
            rrtPlanners.push_back(RRT(0, 0, 16, 16, Union_obstacle, s, g, 0.05, 0.5, 0.25, 7500, pathMaps));

            bool pathFound = rrtPlanners[i].ExtendRRT();

            if(pathFound){
                std::cout << "----------------Path Found for: " << i << std::endl;
                pathInfo path = rrtPlanners[i].RRTpath();
                paths.push_back(path.path);
                pathMaps = insertPath(path.path, pathMaps, i);
            }
            check = check&pathFound;
        }

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - expStart);
        time << duration.count()<< ",";
        valid << check << ",";
        // length << "\n";
        time << "\n";
        valid << "\n";
    }
    length.close();
    time.close();
    valid.close();
}


void graph::add_edge(node* a, node* b, float weight){
    this->adjList[a].push_back(std::pair<node*,float>{b,weight});
    this->adjList[b].push_back(std::pair<node*,float>{a,weight});
}

void obstacle::plot(){
    std::vector<float> x_vals, y_vals;
    for(int i = 0; i < this->obs_points.size(); i++){
        x_vals.push_back(this->obs_points[i].first);
        y_vals.push_back(this->obs_points[i].second);
    }
    plt::plot(x_vals,y_vals,"*-");
}

bool obstacle::CheckIntersectionWObs(point pos){
    // Sum of angles of the point with each vertex point sums to 360 degrees if inside the obstacle
    int n                       = this->obs_points.size();
    float my_sum                = 0;
    bool intersection           = false;
    float prev_min              = INFINITY;
    float dist_from_line        = 0;
    int line_cnt                = 0;

    for(int i = 0; i < this->obs_points.size()-1; i++){
        // my_sum = sum of all interior angles; interior angles = angle of pos.point vec  - angle of pos.next point vec
        float ang           = std::atan2(this->obs_points[(i+1)%n].second - pos.y, this->obs_points[(i+1)%n].first - pos.x) 
                            - std::atan2(this->obs_points[i].second - pos.y, this->obs_points[i].first - pos.x);
        ang                 = angle_wrap(ang);
        my_sum              += ang;
    }
    if (std::abs(my_sum)    >= M_PI){
        intersection        = true;
    } 
    return intersection;
}

bool RRT::ExtendRRT(){
    std::random_device rd;  // Will be used to obtain a seed for the random number engine
    std::mt19937 gen(rd()); // Standard mersenne_twister_engine seeded with rd()

    std::uniform_real_distribution<float> disx(this->min_vals.x, this->max_vals.x);
    std::uniform_real_distribution<float> disy(this->min_vals.y, this->max_vals.y);
    std::uniform_real_distribution<float> disGoal(0, 1);

    int count = 0; 
    while(count < this->maxIters){
        // Generate a node (with bias to deltaT)
        // Find nearest node in the sampledNodes
        // Find point at deltaT distance from nearest node 
        // Add node to tree(Update parent and child pointer in nodes) if edge and node is free else skip entirely
        float goalSelect = disGoal(gen);
        node* temp;
        node* temp1;
        float minDist = INFINITY;
        if(goalSelect < this->goalBias){
            temp          = &goal;
        }
        else{
            node* nNew = new node; nNew->coordinate.x = disx(gen); nNew->coordinate.y = disy(gen); 
            temp = nNew;
        }
        
        for(node* n: this->sampledNodes){
            float d = dist(n->coordinate, temp->coordinate);
            if(d < minDist){
                minDist = d;
                temp1 = n;
            }
        } 

        float angle = std::atan2(temp->coordinate.y - temp1->coordinate.y, temp->coordinate.x - temp1->coordinate.x); 
        node* nn = new node;
        nn->coordinate.x = this->deltaT*std::cos(angle) + temp1->coordinate.x;
        nn->coordinate.y = this->deltaT*std::sin(angle) + temp1->coordinate.y;
        nn->val          = count + 2;
        nn->time         = temp1->time + this->tStep;

        if(this->checkEdge(temp->coordinate, nn->coordinate) || this->checkInObstacle(nn->coordinate) || checkAgents(this->storage, nn->time, nn->coordinate)){
            count++;
            continue;
        }
        else{
            nn->parent = temp1;
            temp1->child.push_back(nn);
            this->sampledNodes.push_back(nn);
        }

        if(dist(nn->coordinate, this->goal.coordinate) < this->goalRegion){
            return true;
        }
        count++;
    }
    return false;
}

pathInfo RRT::RRTpath(){
    node* goalr; // Node in goal region
    node* temp;
    float len = 0.0;
    std::vector<node*> ansVec;

    for(node* n: this->sampledNodes){
        float d = dist(n->coordinate, this->goal.coordinate);
        if(d < this->goalRegion){
            goalr = n;
            break;
        }
    }
    temp = goalr;
    while(temp->parent->val != 0){
        temp = temp->parent;
        ansVec.push_back(temp);
        len += dist(temp->parent->coordinate, temp->coordinate);
    }
    ansVec.push_back(temp->parent);
    len += dist(temp->parent->coordinate, temp->coordinate);
    pathInfo path; path.path = ansVec; path.length = len;
    return path;
}


bool RRT::checkEdge(point p1, point p2){ // Update this
    float delta_t = 0.01; 
    // Check line parametrically
    float t = 0.0; 
    while(t <= 1.0){
        point p; 
        p.x = (1-t)*(p1.x) + t*(p2.x); 
        p.y = (1-t)*p1.y + t*p2.y;

        for(obstacle o: this->Union_obstacle){
            if(o.CheckIntersectionWObs(p)){
                return true;
            }
        }

        t += delta_t;
    }
    return false;
}

bool RRT::checkInObstacle(point p){ //Update this
    for(obstacle o: this->Union_obstacle){
        point pi; pi.x = p.x; pi.y = p.y;
        if(o.CheckIntersectionWObs(pi)){
            std::cout << "here:" << pi.x << " " << pi.y << std::endl;
            return true;
        }
    }
    return false;
}

bool RRT::checkAgents(std::unordered_map<int, std::unordered_map<float,node*>> storage, float t, point p){
    for(auto r: storage){

        if(r.second.count(t) == 0){
            continue;
        }

        point p1; p1.x = r.second[t]->coordinate.y; p1.y = r.second[t]->coordinate.y; //r.second[t]->coordinate.y

        if(dist(p,p1) < 0.5){
            return true;
        }
    }

    return false;
}

// void RRT::plotGraph(){
//         std::vector<std::string> s{"k","b","g","r","c","y"};
//         for(node* p:this->sampledNodes){
//             for(node* pc: p->child){
//                 for(int i = 0; i < pc->coordinate.x.size(); i++){
//                     plt::plot(std::vector<float>{pc->coordinate.x[i], p->coordinate.x[i]}, std::vector<float>{pc->coordinate.y[i], p->coordinate.y[i]},"o-" + s[i%6]);
//                 }
//             }
//         }
// }

float angle_wrap(float angle){
    if(angle > M_PI){
        return -1*(2*M_PI-angle);
    }
    else if(angle < -M_PI){
        return -1*(-2*M_PI-angle);
    }
    return angle;
}

float dist(pointVec p1, pointVec p2){ // added lines
    float accum = 0;
    for(int i = 0; i < p1.x.size(); i++){
        accum += std::pow(p2.y[i] - p1.y[i],2) + std::pow(p2.x[i] - p1.x[i],2);
    }
    return std::sqrt(accum);
}

float dist(point p1, point p2){
    return std::pow(p2.y - p1.y,2) + std::pow(p2.x - p1.x,2);
}

std::unordered_map<int, std::unordered_map<float,node*>> insertPath(std::vector<node*> path, std::unordered_map<int, std::unordered_map<float,node*>> storage, int robotNo){
    if(path.empty()){
        return storage;
    }

    for(node* n: path){
        storage[robotNo][n->time] = n;
    }
    return storage;
}