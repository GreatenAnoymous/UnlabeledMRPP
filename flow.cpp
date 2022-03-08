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
        prepare(timeStep,result);
        solved=solve_model();
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
void FlowMAPF::prepare(int timeStep,Path &result){
    // id_node.cl
    // startIndexes.clear();
    // goalIndexes.clear();

    std::set<Location*> reachable_vertices;
    int id=0;
    for(int i=0;i<starts.size();i++) reachable_vertices.insert(starts[i]);
    for(int t=0;t<timeStep;t++){
        std::set<Location*> next_reachable_vertices;
        for(auto &v1:reachable_vertices){
            flowNode node1={v1->id,-1,t};
            insert_node(id,node1);
            for(auto &v2:reachable_vertices){
               
                std::pair<int,int> edge;
                if(v1->id<v2->id) edge={v1->id,v2->id};
                else edge={v2->id,v1->id};
                flowNode node2,node3,node4;
                node2={edge.first,edge.second,t+0.3};
                insert_node(id,node2);
                node3={edge.first,edge.second,t+0.6};
                insert_node(id,node3);
                node4={v2->id,-1,t+0.9};
                insert_node(id,node4);
                add_edge(node1,node2);
                add_edge(node2,node3);
                add_edge(node3,node4);
                next_reachable_vertices.insert(v2);
                // add_edge(node_id[node1],node_id[node2]);
                // add_edge(node_id[node])
            }
            flowNode node5,node6;
            node5=std::make_tuple(v1->id,-1,t+0.9);
            node6=std::make_tuple(v1->id,-1,t+1);
            insert_node(id,node5);
            insert_node(id,node6);
            add_edge(node1,node5);
            add_edge(node5,node6);
        }
        reachable_vertices=next_reachable_vertices;
    }
    flowNode source={-211,211,0};
    flowNode sink={-985,-985,0};
    insert_node(id,source);
    insert_node(id,sink);
    for(int r=0;r<starts.size();r++){
        flowNode snode={starts[r]->id,-1,0};
        flowNode gnode={goals[r]->id,-1,timeStep};
        add_edge(source,snode);
        add_edge(gnode,sink);
    }

}













