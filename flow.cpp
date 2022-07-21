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
// #define DEBUG true
#define IN 0
#define OUT 1

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




FlowMAPF::~FlowMAPF(){
    
}

/**
 * @brief 
 * 
 * @return Paths 
 */
Paths FlowMAPF::solve(){
    evaluateLB();
    int timeStep=makespanLB;
    bool solved=true;
    Paths result;
    while(true){
        // std::cout<<"preparing model for timestep="<<timeStep<<std::endl;
        prepare(timeStep);
        // std::cout<<"prepared model for timestep="<<timeStep<<std::endl;
        StarGraph gs(node_id.size(), startNodes.size());
        MaxFlow max_flow(&gs,source_id,sink_id);
        for(int i=0;i<startNodes.size();i++){
            ArcIndex arc=gs.AddArc(startNodes[i],endNodes[i]);
            max_flow.SetArcCapacity(arc,1); //unit capacity
        }
        max_flow.Solve();
        FlowQuantity total_flow = max_flow.GetOptimalFlow();
        std::cout<<"the max flow quantity="<<total_flow<<"   num of robots="<<starts.size()<<std::endl;
        if(total_flow==starts.size()) {     //solved
            retrievePaths(max_flow,result);
            resolveEdgeConflicts(result);
            // std::cout<<"edge conflicts resolved!"<<std::endl;
            return result;
        }
        timeStep++;
    }
    return {};  //failed
    
    
}

/**
 * @brief 
 * 
 * @param id 
 * @param node 
 */
void FlowMAPF::insert_node(int &id,flowNode &node){
    if(node_id.find(node)==node_id.end()){
        node_id[node]=id;
        id_node[id]=node;
        id++;
    }
}



void FlowMAPF::add_edge(flowNode & u,flowNode &v){
    startNodes.push_back(node_id[u]);
    endNodes.push_back(node_id[v]);
}

/**
 * @brief 
 * 
 */
void FlowMAPF::evaluateLB(){
    makespanLB=0;
    for(int i=0;i<starts.size();i++){
        int mki=9999996;
        for(int j=0;j<starts.size();j++){
            mki=std::min(mki,starts[i]->manhattan_dist(goals[j]));
        }
        makespanLB=std::max(mki,makespanLB);
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
void FlowMAPF::prepare(int timeStep){
    startNodes.clear();
    endNodes.clear();
    node_id.clear();
    id_node.clear();


    //out=0 in=1
    std::set<Location*> reachable_vertices;
    int id=0;
    for(int i=0;i<starts.size();i++) reachable_vertices.insert(starts[i]);
    for(int t=0;t<timeStep;t++){
        std::set<Location*> next_reachable_vertices;
        for(auto v1: reachable_vertices){
            flowNode node1={v1->id,t,OUT};
            insert_node(id,node1);
            auto nbrs=graph->getNeighbors(v1);
            // std::cout<<"neighor size="<<nbrs.size()<<"  "<<v1->print()<<std::endl;
            nbrs.push_back(v1);
            for(auto v2: nbrs){
                flowNode node2={v2->id,t+1,IN};
                insert_node(id,node2);
                add_edge(node1,node2);
                next_reachable_vertices.insert(v2);
            }
            next_reachable_vertices.insert(v1);
            // flowNode node2={v1->id,t,IN}
        }
        for(auto &v:next_reachable_vertices){
            flowNode node2={v->id,t+1,IN};
            flowNode node3={v->id,t+1,OUT};
            insert_node(id,node3);
            add_edge(node2,node3);
        }
        reachable_vertices=next_reachable_vertices;
    }
    
    // add source and sink
    flowNode source={-985,-985,OUT};
    flowNode sink={-211,-211,IN};
    insert_node(id,source);
    source_id=node_id[source];
    insert_node(id,sink);
    sink_id=node_id[sink];
    for(int r=0;r<starts.size();r++){
        flowNode sr={starts[r]->id,0,OUT};
        flowNode gr={goals[r]->id,timeStep,OUT};
        // assert(node_id.find(sr)!=node_id.end());
        // assert(node_id.find(gr)!=node_id.end());
        add_edge(source,sr);
        add_edge(gr,sink);
    }
}   


/**
 * @brief 
 * 
 * @param max_flow 
 * @param result 
 */
void FlowMAPF::retrievePaths(MaxFlow &max_flow,Paths &result){
    std::map<int,int> adj_list;
    auto starGraph=max_flow.graph();
    for(int i=0;i<startNodes.size();i++){
        if(fabs(max_flow.Flow(i)-1)<1e-2){
            auto head_i=starGraph->Head(i);
            auto tail_i=starGraph->Tail(i);
            
            auto head_node=id_node[head_i];
            auto tail_node=id_node[tail_i];
            // std::cout<<" head : ["<<graph->getVertex(head_node.first)->print()<<","<<head_node.second<<"] ";
            // std::cout<<"tail: ["<<graph->getVertex(tail_node.first)->print()<<","<<tail_node.second<<"] ";
            // std::cout<<"flow quatity: "<<max_flow.Flow(i)<<std::endl;
            adj_list.insert({tail_i,head_i});
        }
    }
    result=Paths(starts.size(),Path());
    for(int i=0;i<starts.size();i++){
        // std::cout<<"robot "<<i<<std::endl;
        flowNode si={starts[i]->id,0,OUT};
        flowNode sink={-211,-211,IN};
        int current_id=node_id[si];
        while(current_id!=node_id[sink]){
            // int timestep=id_node[current_id].second;
            auto flownode=id_node[current_id];
            int vid=std::get<0>(flownode);
            int in_or_out=std::get<2>(flownode);
            if(in_or_out==OUT) result[i].push_back( graph->getVertex(vid));
            // std::cout<<"current id="<< current_id<<"  timestep="<<timestep<<"  "<<graph->getVertex(id_node[current_id].first)->print()<<"   "<<current_id<<"  -->   "<<adj_list[current_id] <<std::endl;
            current_id=adj_list[current_id];
        }
    }
    // for(int i=0;i<starts.size();i++){
    //     std::cout<<result[i].size()<<std::endl;
    // }
    
    // for(int i=0;i<startNodes.size();i++){
    //     goals[i]=result[i].back();
    // }
}



/**
 * @brief 
 * find if there is any edge conflict and resolve them by switching tails
 * @param result 
 */
void FlowMAPF::resolveEdgeConflicts(Paths &result){
    for(int t=1;t<result[0].size();t++){
        for(int i=0;i<result.size();i++){
            for(int j=i+1;j<result.size();j++){
                if(result[i][t]==result[j][t-1] and result[i][t-1]==result[j][t]){
                    // std::cout<<"find edge conflict! resolving...."<<std::endl;
                    switchPaths(i,j,t,result);
                }
            }
        }
    }

    //update the goals
    for(int i=0;i<starts.size();i++){
        goals[i]=result[i].back();
    }
}


/**
 * @brief 
 * 
 * @param i 
 * @param j 
 * @param t 
 * @param result 
 */
void FlowMAPF::switchPaths(int i,int j,int t,Paths &result){
    Path pj=result[j];
    assert(result[j].size()==result[i].size());
    assert(result[0].size()==result[i].size());
    // std::cout<<result[i].size()<<"    "<<result[j].size()<<std::endl;
    for(int k=t;k<result[0].size();k++){
        result[j][k]=result[i][k];
        result[i][k]=pj[k];
    }
}


/**
 * @brief 
 * 
 * @return Paths 
 */
Paths FlowMAPF::solveWeighted(){
    auto getWeight=[&](int head,int tail){
        auto node1=id_node[head];
        auto node2=id_node[tail];
        if(std::get<0>(node1)==std::get<0>(node2)) return 0;
        return 1;
    };
    evaluateLB();
    int timeStep=makespanLB;
    bool solved=true;
    Paths result;
    while(true){
        // std::cout<<"preparing model for timestep="<<timeStep<<std::endl;
        prepare(timeStep);
        // std::cout<<"prepared model for timestep="<<timeStep<<std::endl;
        StarGraph gs(node_id.size(), startNodes.size());
        MinCostFlow min_cost_flow(&gs);
        std::vector<int> sink_arcs;
        for(int i=0;i<startNodes.size();i++){
            ArcIndex arc=gs.AddArc(startNodes[i],endNodes[i]);
            min_cost_flow.SetArcCapacity(arc,1); //unit capacity
            min_cost_flow.SetArcUnitCost(arc,getWeight(startNodes[i],endNodes[i]));
            if(endNodes[i]==sink_id) sink_arcs.push_back(i);
        }
        int numRobots=starts.size();
        min_cost_flow.SetNodeSupply(source_id,numRobots);
        min_cost_flow.SetNodeSupply(sink_id,-numRobots);

        min_cost_flow.Solve();
        int total_flow=0;
        for(auto arc_id:sink_arcs){
            total_flow+=min_cost_flow.Flow(arc_id);
        }
        // FlowQuantity total_flow = max_flow.GetOptimalFlow();
        std::cout<<"the max flow quantity="<<total_flow<<"   num of robots="<<starts.size()<<std::endl;
        if(total_flow==numRobots) {     //solved
            retrievePaths(min_cost_flow,result);
            resolveEdgeConflicts(result);
            // std::cout<<"edge conflicts resolved!"<<std::endl;
            return result;
        }
        timeStep++;
    }
    return {};  //failed
}

/**
 * @brief 
 * 
 * @param max_flow 
 * @param result 
 */
void FlowMAPF::retrievePaths(MinCostFlow &max_flow,Paths &result){
    std::map<int,int> adj_list;
    auto starGraph=max_flow.graph();
    for(int i=0;i<startNodes.size();i++){
        if(fabs(max_flow.Flow(i)-1)<1e-2){
            auto head_i=starGraph->Head(i);
            auto tail_i=starGraph->Tail(i);
            
            auto head_node=id_node[head_i];
            auto tail_node=id_node[tail_i];
            // std::cout<<" head : ["<<graph->getVertex(head_node.first)->print()<<","<<head_node.second<<"] ";
            // std::cout<<"tail: ["<<graph->getVertex(tail_node.first)->print()<<","<<tail_node.second<<"] ";
            // std::cout<<"flow quatity: "<<max_flow.Flow(i)<<std::endl;
            adj_list.insert({tail_i,head_i});
        }
    }
    result=Paths(starts.size(),Path());
    for(int i=0;i<starts.size();i++){
        // std::cout<<"robot "<<i<<std::endl;
        flowNode si={starts[i]->id,0,OUT};
        flowNode sink={-211,-211,IN};
        int current_id=node_id[si];
        while(current_id!=node_id[sink]){
            // int timestep=id_node[current_id].second;
            auto flownode=id_node[current_id];
            int vid=std::get<0>(flownode);
            int in_or_out=std::get<2>(flownode);
            if(in_or_out==OUT) result[i].push_back( graph->getVertex(vid));
            // std::cout<<"current id="<< current_id<<"  timestep="<<timestep<<"  "<<graph->getVertex(id_node[current_id].first)->print()<<"   "<<current_id<<"  -->   "<<adj_list[current_id] <<std::endl;
            current_id=adj_list[current_id];
        }
    }
    // for(int i=0;i<starts.size();i++){
    //     std::cout<<result[i].size()<<std::endl;
    // }
    
    // for(int i=0;i<startNodes.size();i++){
    //     goals[i]=result[i].back();
    // }
}