/**
 * @file dou.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-19
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include"dou.hpp"
#include"search.hpp"
#include"lp.hpp"


/**
 * @brief using BFS to find the initial paths
 * 
 * @param paths 
 */
void DU_MAPF::find_initial_paths_faster(Paths & paths){
    paths.clear();
    using costMatrix=std::vector<std::vector<double>>;
    using costEdges=std::vector<std::tuple<int,int,double>>;
    auto m=starts.size();
    std::vector<int> rowsol(m);

   
    int threshold=7;        //if there is a lap algorithm for sparse matrix, that would be great
    bool feasible=false;
    // costMatrix costs(m,std::vector<double>(m,BIG));
    auto diff=[](Location *s,Location *g){
        return s->manhattan_dist(g);
    };
    costEdges costs;
    for(int i=0;i<m;i++){
        for(int j=0;j<m;j++){
            double dist=starts[i]->manhattan_dist(goals[j]);
            if(dist<threshold) {
                dist=diff(starts[i],goals[j]);
                costs.push_back({i,j,dist}); 
            }
        }
    }

    auto best_cost=lsa_sparse(costs,rowsol);

    Configs new_goals;
    for(int i=0;i<m;i++){
        new_goals.push_back(goals[rowsol[i]]);
    }

    goals.swap(new_goals);
    for(int i=0;i<m;i++){
        BFS_solver solver(starts[i]);
        solver.isGoal=[&](Location *v){
            return v==goals[i];
        };
        solver.getNeighbors=[&](Location*v){
            return graph->getNeighbors(v); 
        };
        auto pi=solver.solve();
        paths.push_back(pi);
    }
}

/**
 * @brief 
 * 
 * @param paths 
 */
void DU_MAPF::find_initial_paths(Paths & paths){
    int threshold=20;
    int numAgents=starts.size();
    std::vector<std::tuple<int,int,double>>costEdges;
    std::vector<std::vector<Path>> pathTable(numAgents,std::vector<Path>(numAgents));
    
    auto BFS_with_threshold=[&](Location*start,int id){
        std::map<int,int> depth_map;
        std::map<int,int> parent_map;
        std::map<int,int> cost_map;
        std::set<int> goal_set;

        for(auto &gi:goals) goal_set.insert(gi->id);
        std::queue<Location*> open;
        open.push(start);
        cost_map[start->id]=0;
        depth_map[start->id]=0;

        while(open.empty()==false){
            auto tp=open.front();
            open.pop();
            if(goal_set.find(tp->id)!=goal_set.end()){
                goal_set.erase(tp->id);
            }
            if(goal_set.empty()) break;
            for(auto &c:graph->getNeighbors(tp)){
                if(cost_map.find(c->id)!=cost_map.end()) continue;
                depth_map[c->id]=depth_map[tp->id]+1;
                parent_map[c->id]=tp->id;
                if(depth_map[c->id]>threshold) continue;
                cost_map[c->id]=depth_map[c->id]+1;
                open.push(c);
            }
        } 
        for(int j;j<numAgents;j++){
            auto current=goals[j];
            Path pij;
            while(current!=start){
                pij.push_back(current);
                current=graph->getVertex(parent_map[current->id]);
            }
            pij.push_back(start);
            std::reverse(pij.begin(),pij.end());
            pathTable[id][j]=pij;
            if(pij.size()<threshold) costEdges.push_back({id,j,pij.size()});
        }
    };
    for(int i=0;i<numAgents;i++){
         BFS_with_threshold(starts[i],i);
    }
    std::vector<int>assignment(numAgents);
    lsa_sparse(costEdges,assignment);
    Configs new_goals;
    for(int i=0;i<numAgents;i++){
        new_goals.push_back(goals[assignment[i]]);
    }

    goals.swap(new_goals);
    for(int i=0;i<numAgents;i++){
        paths.push_back(pathTable[i][assignment[i]]);
    }
    pathTable.clear();

}

/**
 * @brief 
 * 
 * @param old_paths 
 */

void DU_MAPF::update_paths(Paths &old_paths){
    int num_agents=old_paths.size();
    auto toPathSet=[](Path &p){
        std::unordered_set<Location*> path_set;
        for(auto &vs:p) path_set.insert(vs);
        return path_set;
    };
    using LocationSet=std::unordered_set<Location*>;
    LocationSet goalSet,startSet;
    std::unordered_map<int,int> degrees;

    auto findStandAloneGoal=[&](){
        for(auto &goal:goalSet){
            if(degrees[goal->id]<=1) return goal;
        }
        throw std::runtime_error("no standlone goal!");
    };

    for(int i=0;i<num_agents;i++){
        goalSet.insert(old_paths[i].back());
        startSet.insert(old_paths[i][0]);
        degrees[old_paths[i].back()->id]=0;
    }

    std::vector<LocationSet> path_sets;
    for(int i=0;i<num_agents;i++){
        LocationSet pi=toPathSet(old_paths[i]);
        path_sets.push_back(pi);
    }

    for(int i=0;i<num_agents;i++){
        for(auto &v:path_sets[i]) if(degrees.find(v->id)!=degrees.end()) degrees[v->id]++;
    }

    path_sets.clear();
    DAG dag;
    formDAG(old_paths,dag);
    Paths new_paths;
    while(not startSet.empty()){
        auto standAloneGoal=findStandAloneGoal();
        BFS_solver searcher(standAloneGoal);
        searcher.getNeighbors=[&](Location *v){
            auto possible_n=dag[v->id];
            Configs tmp;
            for(auto &n:possible_n){
                if(n.second>0) tmp.push_back(graph->getVertex(n.first));
            }
            return tmp;
        };
        searcher.isGoal=[&](Location*v){
            return startSet.find(v)!=startSet.end();
        };
        Path pi=searcher.solve();
        if(pi.size()==0){
            std::cout<<standAloneGoal->print()<<" bug happend"<<std::endl;
        }
        else{
            auto si=pi.back();
            // std::cout<<si->print()<<" "<<standAloneGoal->print()<<std::endl;
        }
        
        auto si=pi.back();
        startSet.erase(si);
        goalSet.erase(standAloneGoal);
        std::reverse(pi.begin(),pi.end());
        new_paths.push_back(pi);
        degrees.erase(standAloneGoal->id);
        auto new_pathSet=toPathSet(pi);

        //remove the weight of used edges
        for(int t=pi.size()-1;t>0;t--){
            auto it1=dag[pi[t]->id].begin();
            auto it2=dag[pi[t]->id].end();
            auto cond=[&](point2d &v){
                return v.first==pi[t-1]->id;
            };
            auto it=std::find_if(it1,it2,cond);
            it->second=it->second-1;
        }

        for(auto &v:new_pathSet){
            if(degrees.find(v->id)!=degrees.end()) degrees[v->id]--;
        }
    }
    
    // std::cout<<"updated : num_agents="<<new_paths.size()<<std::endl;
    old_paths.swap(new_paths);    
}

/**
 * @brief schedule the paths to time parametrized
 * 
 * @param old_paths 
 */

void DU_MAPF::schedule(Paths &old_paths){
    auto num_agents=old_paths.size();
    using timeObstacle=std::tuple<int,int>;
    Paths timed_paths(num_agents);
    std::unordered_set<timeObstacle,boost::hash<timeObstacle>> reserveTable;
    for(auto &p:old_paths){
        Path pi;
        int t=0,k=0;
        while(k<p.size()){
            timeObstacle obsk={p[k]->id,t};
            if(reserveTable.find(obsk)==reserveTable.end()){
                pi.push_back(p[k]);
                reserveTable.insert(obsk);
                t++;
                k++;
            }else{
                //wait
                pi.push_back(pi.back());
                auto v=pi.back();
                reserveTable.insert({v->id,t});
                t++;
            }          
        }  
        auto si=pi[0];
        auto itr=std::find(starts.begin(),starts.end(),si);
        int index=std::distance(starts.begin(), itr);
        timed_paths[index]=pi;
        // timed_paths.push_back(pi);
    }
    old_paths.swap(timed_paths);
}

/**
 * @brief 
 * 
 * @param paths 
 * @param dag_graph 
 */
void DU_MAPF::formDAG(const Paths &paths,DAG&dag_graph){
    for(auto &p:paths){
        for(int t=p.size()-1;t>=1;t--){
            auto it1=dag_graph[p[t]->id].begin();
            auto it2=dag_graph[p[t]->id].end();
            auto cond=[&](point2d & v){
                return v.first==p[t-1]->id;
            };
            auto it=std::find_if(it1,it2,cond);
            if(it==it2) dag_graph[p[t]->id].push_back({p[t-1]->id,1});
            else it->second=it->second+1;
            
        }
    }
}


void DU_MAPF::solve(){
    find_initial_paths(result);
    update_paths(result);
    schedule(result);
}


Paths DU_MAPF::get_result(){
    return result;
}