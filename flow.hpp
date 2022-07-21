/**
 * @file flow.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * max-flow based mrpp min-makespan
 * @copyright Copyright (c) 2022
 * 
 */

#pragma once

#include "common.hpp"
#include"lp.hpp"
#include <ortools/graph/max_flow.h>
#include <ortools/graph/min_cost_flow.h>


class FlowMAPF{
public:
    // const int None=-1;
    using flowNode=std::tuple<int,int,int>;// u, t, in or out
    // const int source=-985;
    // const int goal=-211;
    FlowMAPF(Configs&starts,Configs &goals,Grids*graph);
    ~FlowMAPF();

    //solve maxflow
    Paths solve();

    //solve min-cost maxflow
    Paths solveWeighted();

private:
    Configs starts,goals;
    Grids *graph;
    void prepare(int timestep);


    void add_edge(flowNode & u,flowNode &v);
    void insert_node(int &id,flowNode &node);

    void resolveEdgeConflicts(Paths &result);
    void retrievePaths(operations_research::MaxFlow &flow,Paths &result);
    void retrievePaths(operations_research::MinCostFlow &flow,Paths &result);

    void switchPaths(int i,int j,int t,Paths &result);

    std::vector<int>startNodes;
    std::vector<int>endNodes;
    std::vector<int>capacities; //capacity is 1

    std::unordered_map<int,flowNode> id_node;
    std::unordered_map<flowNode,int,boost::hash<flowNode>> node_id;

    void evaluateLB();
    void evaluateLBfast();
    int source_id;
    int sink_id;

    int makespanLB;

};