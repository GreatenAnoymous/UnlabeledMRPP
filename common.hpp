/**
 * @file common.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once

#include<vector>
#include<iostream>
#include<unordered_map>
#include<unordered_set>
#include<iostream>
#include<memory>
#include<unordered_map>
#include<unordered_set>
#include<vector>
#include<queue>
#include<fstream>
#include <chrono>
#include <cmath>
#include <regex>
#include <boost/functional/hash.hpp>
#include <stack>
#include <limits>

typedef std::chrono::high_resolution_clock Time;
typedef std::chrono::milliseconds ms;
typedef std::chrono::duration<float> fsec;
typedef std::pair<int,int> point2d;

///////////////////////////////////////////////////////////////////////////////
class Location{
public:
    Location(int _id,int _x,int _y):x(_x),y(_y),id(_id){}
    int x,y,id;
    int manhattan_dist(Location *other){
        return abs(x-other->x)+abs(y-other->y);
    }
    std::string print(){
        return "("+std::to_string(x)+","+std::to_string(y)+")";
    }
    // std::vector<Location3d*> neighbors;
};
//////////////////////////////////////////////////////////////////////////////

class Grids{
public:

    Grids(std::string file_name);
    ~Grids();
    std::vector<Location *> getNeighbors(Location *);
    Location*getVertex(int x,int y);
    Location*getVertex(int id);
    int xmax,ymax;
    int getNodesSize(){
        return nodes.size();
    }
    std::vector<Location*> getNodes(){
       return nodes; 
    }

    void add_obstacles(Location *obs){
        this->obstacles.insert(obs);
    }

    bool isBlocked(Location *v){
        return obstacles.find(v)!=obstacles.end();
    }

protected:
    std::vector<Location *> nodes;
    std::set<Location *> obstacles;
};
////////////////////////////////////////////////////////////////////////////////
using Configs=std::vector<Location*>;
using Path=std::vector<Location*>;
using Paths=std::vector<Path>;

////////////////////////////////////////////////////////////////////////////////
// void read_graph(std::string file_name,Grids *&graph);
void read_scen(std::string file_name,Configs&starts,Configs &goals,Grids *graph);
void save_solutions(std::string file_name,Paths&paths,double runtime,bool save_paths);
void check_feasible_bruteForce(Paths &);
void format_paths(Paths &paths);
void evaluate_result(Paths &,int &makespan,int &makespanLB,int &soc,int &socLB);   
void fill_paths(Paths &,int makespan=-1);
void shrink_paths(Paths&paths);
// void precompute_dist(Grids*graph,Configs&starts,Configs&goals,std::vector<std::tuple<int,int,double>>&costEdges);
//////////////////////////////////////////////////////////////////////////////////