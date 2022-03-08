/**
 * @file flow.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include "common.hpp"

#include"lp.hpp"




class FlowMAPF{
public:
    using flowNode=std::tuple<int,int,double>;
    // const int source=-985;
    // const int goal=-211;
    FlowMAPF(Configs&starts,Configs &goals,Grids*graph);
    Path solve();

private:
    Configs starts,goals;
    Grids *graph;
    void prepare(int timestep,Path &);
    bool solve_model();
    void add_edge(flowNode & u,flowNode &v);
    void insert_node(int &id,flowNode &node);
    void find_arc_by_head();

    std::vector<int>startIndexes;
    std::vector<int>goalIndexes;
    // std::vector<int>capacities; capacity is 1

    std::unordered_map<int,flowNode> id_node;
    std::unordered_map<flowNode,int,boost::hash<flowNode>> node_id;

    void evaluateLB();

    int makespanLB;

};