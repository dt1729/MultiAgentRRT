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
    node* parent;
    std::vector<node*> child;
    pointVec coordinate;
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
        std::vector<obstacle>   Union_obstacle;
        std::vector<node*>      sampledNodes;
        std::vector<bool>       reach;
        float                   goalBias;
        float                   deltaT;
        float                   goalRegion;
        float                   interAgentDist;
        point                   min_vals;
        point                   max_vals;
        node                    start;
        node                    goal;
        int                     treeCount = 0;
        int                     maxIters;
        int                     rCount;

        RRT(float min_x, float min_y, float max_x, float max_y, std::vector<obstacle> uObs, pointVec start, pointVec goal, float goalBias, float deltaT, float goalRegion, int maxIters, float interAgentDist, int rCount){
            this->min_vals.x = min_x; this->min_vals.y = min_y;
            this->max_vals.x = max_x; this->max_vals.y = max_y;
            this->goalBias   = goalBias; this->goalRegion = goalRegion;
            this->deltaT     = deltaT; 
            this->Union_obstacle = uObs;
            this->start.coordinate = start; this->start.val = 0; this->sampledNodes.push_back(&this->start);
            this->goal.coordinate = goal; this->goal.val = 1;
            this->maxIters = maxIters;
            this->interAgentDist = interAgentDist;
            this->rCount     = rCount;
            for(int i = 0; i < rCount; i++) reach.push_back(false);
        }   

        bool ExtendRRT();
        pathInfo RRTpath();
        void plotGraph();
        bool checkEdge(pointVec p1, pointVec p2);
        bool checkInObstacle(pointVec p);
        bool checkAgents(pointVec p);
};


float angle_wrap(float angle);
void part_a(int n_agents);
void part_a_hundred(int n_agents);
float dist(pointVec p1, pointVec p2);
float dist(point p1, point p2);

int main(){
    for(int  i = 1; i < 7; i++){
        part_a(i);
    }
    for(int  i = 1; i < 7; i++){
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

    RRT rrt(0, 0, 16, 16, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 7500, 0.5, n_agents); //float goalBias, float deltaT, float goalRegion, int maxIters, float interAgentDist for 6 agents 30,000 works fine
    bool pathFound = rrt.ExtendRRT();
    std::cout << rrt.treeCount << std::endl;
    for(obstacle o:Union_obstacle){
        o.plot();
    }
    std::vector<float> px, py;
    std::vector<std::string> s{"k","b","g","r","c","y"};
    if(pathFound){
        pathInfo path = rrt.RRTpath();
        for(node* p: path.path){
            for(int i = 0; i < rrt.rCount; i++){
                px.push_back(p->coordinate.x[i]); py.push_back(p->coordinate.y[i]);
                plt::plot(px, py, "o-" + s[i%6]);
                px.clear(); py.clear();
            }
        }
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
        RRT rrt(0, 0, 16, 16, Union_obstacle, start, goal, 0.05, 0.5, 0.25, 7500, 0.5, n_agents); //float goalBias, float deltaT, float goalRegion, int maxIters, float interAgentDist for 6 agents 30,000 works fine
        auto start = std::chrono::high_resolution_clock::now();
        bool pathFound = rrt.ExtendRRT();
        length << rrt.treeCount << ",";
        if(pathFound){
            pathInfo path = rrt.RRTpath();
        }
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        time << duration.count()<< ",";
        valid << pathFound << ",";
        length << "\n";
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
        std::cout << "Count: " << count << std::endl;
        if(goalSelect < this->goalBias){
            temp          = &this->goal;
        }
        else{
            node* nNew = new node; 
            for(int i = 0; i < this->rCount; i++){// added lines
                if(this->reach[i]){
                    nNew->coordinate.x.push_back(this->goal.coordinate.x[i]); nNew->coordinate.y.push_back(this->goal.coordinate.y[i]); 
                }
                else{
                    nNew->coordinate.x.push_back(disx(gen)); nNew->coordinate.y.push_back(disy(gen)); 
                }
                std::cout << nNew->coordinate.x[i] << " " << nNew->coordinate.y[i] << " " << reach[i] << std::endl;
            }
            temp = nNew;
        }
        
        for(node* n: this->sampledNodes){
            float d = dist(n->coordinate, temp->coordinate);
            if(d < minDist){
                minDist = d;
                temp1 = n;
            }
        }
        node* nn = new node;
        for(int i = 0; i < this->rCount; i++){ // added lines
            float angle = std::atan2(temp->coordinate.y[i] - temp1->coordinate.y[i], temp->coordinate.x[i] - temp1->coordinate.x[i]); 
            nn->coordinate.x.push_back(this->deltaT*std::cos(angle) + temp1->coordinate.x[i]);
            nn->coordinate.y.push_back(this->deltaT*std::sin(angle) + temp1->coordinate.y[i]);
        }
        nn->val          = count + 2;
        if(this->checkEdge(temp->coordinate, nn->coordinate) || this->checkInObstacle(nn->coordinate) || this->checkAgents(nn->coordinate)){ // added lines
            count++;
            continue;
        }
        else{
            this->treeCount++;
            nn->parent = temp1;
            temp1->child.push_back(nn);
            this->sampledNodes.push_back(nn);
        }

        bool overallGoal = true;
        for(int i = 0; i < this->rCount; i++){
            point p1, p2; p1.x = nn->coordinate.x[i]; p1.y = nn->coordinate.y[i]; 
            p2.x = this->goal.coordinate.x[i]; p2.y = this->goal.coordinate.y[i];
            if(dist(p1,p2) < this->goalRegion){
                this->reach[i] = true;
                std::cout << "in Goal" << std::endl;
            }
            overallGoal &= this->reach[i];
        }

        if(overallGoal){
            std::cout <<"------------Reached Goal---------------" << std::endl;
            this->goal = *nn;
            return true;
            break;
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


bool RRT::checkEdge(pointVec p1, pointVec p2){ // Update this
    float delta_t = 0.01; 
    // Check line parametrically
    for(int i = 0; i < this->rCount; i++){
        float t = 0.0; 
        while(t <= 1.0){
            point p; 
            p.x = (1-t)*(p1.x[i]) + t*(p2.x[i]); 
            p.y = (1-t)*p1.y[i] + t*p2.y[i];

            for(obstacle o: this->Union_obstacle){
                if(o.CheckIntersectionWObs(p)){
                    return true;
                }
            }

            t += delta_t;
        }
    }
    return false;
}

bool RRT::checkInObstacle(pointVec p){ //Update this
    for(int i = 0; i < this->rCount; i++){
        for(obstacle o: this->Union_obstacle){
            point pi; pi.x = p.x[i]; pi.y = p.y[i];
            if(o.CheckIntersectionWObs(pi)){
                std::cout << "here:" << pi.x << " " << pi.y << std::endl;
                return true;
            }
        }
    }
    return false;
}

bool RRT::checkAgents(pointVec p){
    for(int i = 0; i < this->rCount; i++){
        for(int j = i+1; j < this->rCount; j++){
            if(std::sqrt(std::pow(p.x[i] - p.x[j],2) + std::pow(p.y[i] - p.y[j],2)) < this->interAgentDist){
                std::cout << "here" << std::endl;
                return true;
            }
        }
    }
    return false;
}

float angle_wrap(float angle){
    if(angle > M_PI){
        return -1*(2*M_PI-angle);
    }
    else if(angle < -M_PI){
        return -1*(-2*M_PI-angle);
    }
    return angle;
}

void RRT::plotGraph(){
        std::vector<std::string> s{"k","b","g","r","c","y"};
        for(node* p:this->sampledNodes){
            for(node* pc: p->child){
                for(int i = 0; i < pc->coordinate.x.size(); i++){
                    // std::cout << pc->coordinate.x[i] << " " << p->coordinate.x[i] << std::endl;
                    plt::plot(std::vector<float>{pc->coordinate.x[i], p->coordinate.x[i]}, std::vector<float>{pc->coordinate.y[i], p->coordinate.y[i]},"o-" + s[i%6]);
                }
            }
        }
}