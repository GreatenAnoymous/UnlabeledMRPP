/**
 * @file flow.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include "flow.hpp"
#include <ortools/graph/max_flow.h>


using namespace operations_research;

/**
 * @brief Construct a new Flow MAPF:: Flow M A P F object
 * 
 * @param starts 
 * @param goals 
 * @param graph 
 */
FlowMAPF::FlowMAPF(Configs&starts,Configs &goals,Grids*graph):starts(starts),goals(goals),graph(graph){

}

/**
 * @brief 
 * 
 * @return Path 
 */
Path FlowMAPF::solve(){
    evaluateLB();
    int timeStep=makespanLB;
    bool solved=true;
    Path result;
    while(true){
        solved=prepare_and_solve(timeStep,result);
        if(solved) break;
    }
    return result;
}


void FlowMAPF::insert_node(int &id,flowNode &node){
    if(node_id.find(node)==node_id.end()){
        node_id[node]=id;
        id_node[id]=node;
        id++;
    }
}

/**
 * @brief 
 * 
 */
void FlowMAPF::evaluateLB(){
    makespanLB=0;
    for(int i=0;i<starts.size();i++){
        makespanLB=std::min(makespanLB,starts[i]->manhattan_dist(goals[i]));
    }
}

/**
 * @brief 
 * 
 * @param timeStep 
 * @param result 
 * @return true 
 * @return false 
 */
bool FlowMAPF::prepare_and_solve(int timeStep,Path &result){
    // id_node.cl
    std::set<Location*> reachable_vertices;
    int id=0;
    for(int i=0;i<starts.size();i++) reachable_vertices.insert(starts[i]);
    for(int t=0;t<timeStep;t++){
        std::set<Location*> next_reachable_vertices;
        for(auto &v1:reachable_vertices){
            for(auto &v2:reachable_vertices){
                flowNode node1={v1->id,-1,t};
                insert_node(id,node1);
                std::pair<int,int> edge;
                if(v1->id<v2->id) edge={v1->id,v2->id};
                else edge={v2->id,v1->id};
                flowNode node2,node3,node4;
            }
        }
    }

}













