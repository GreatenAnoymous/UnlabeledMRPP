/**
 * @file dou.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-19
 * Distance Optimal Unlabled MPP
 * @copyright Copyright (c) 2022
 * 
 */

#include"common.hpp"

/**
 * @brief 
 * distance -optimal 
 */
class DU_MAPF{
public:
    using point2d=std::pair<int,int>;   //(vertex, weight)
    using DAG=std::unordered_map<int,std::vector<point2d>>;
    DU_MAPF(Grids*graph,Configs &starts,Configs &goals):starts(starts),goals(goals),graph(graph){}
    void solve();
    Paths get_result();

protected:
    Configs starts;
    Configs goals;
    Grids *graph;
    void find_initial_paths(Paths &);
    void find_initial_paths_faster(Paths &);
    void update_paths(Paths &);
    void schedule(Paths &);
    Paths result;
    std::function<Configs(Location& v)> getNeighbors;
    void formDAG(const Paths &,DAG& dag_graph);
    void BFS(Location &s,Path& path);   //run BFS on DAG 
    void astar_search(Location &start,Location &goal,Path &path);

};






