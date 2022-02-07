/**
 * @file common.cpp
 * @author your name (you@domain.com)
 * @brief
 * @version 0.1
 * @date 2022-02-07
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "common.hpp"

///////////////////////////////////////////////////////////////////// util functions //////////////////////////////////////////////////

/**
 * @brief
 *
 * @param file_name
 * @param starts
 * @param goals
 * @param graph
 */

/**
 * @brief Construct a new Grids:: Grids object
 * 
 * @param file_name 
 */
Grids::Grids(std::string file_name)
{
    std::string line;
    std::smatch results;
    std::regex r_height = std::regex(R"(height\s(\d+))");
    std::regex r_width = std::regex(R"(width\s(\d+))");
    std::regex r_map = std::regex(R"(map)");
    std::ifstream map_file(file_name);
    // fundamental graph params
    while (getline(map_file, line))
    {
        // for CRLF coding
        if (*(line.end() - 1) == 0x0d)
            line.pop_back();

        if (std::regex_match(line, results, r_height)){
            ymax = std::stoi(results[1].str());
        }
        if (std::regex_match(line, results, r_width)){
            xmax = std::stoi(results[1].str());
        }
        if (std::regex_match(line, results, r_map))
            break;
    }
    if (!(xmax> 0 && ymax > 0))
        throw std::runtime_error("incorrect graph size!");

    // create nodes
    nodes = Configs(xmax*ymax, nullptr);
    int y=0;
    while (getline(map_file, line))
    {
        // for CRLF coding
        if (*(line.end() - 1) == 0x0d)
            line.pop_back();

        if ((int)line.size() != ymax)
            throw std::runtime_error("incorrect graph size!");
        for (int x = 0; x < xmax; ++x)
        {
            char s = line[x];
            if (s == 'T' or s == '@')
                continue; // object
            int id = xmax * y + x;
            Location *v = new Location(id, x, y);
            // std::cout<<*v<<std::endl;
            nodes[id] = v;
        }
        ++y;
    }
}

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @return Location* 
 */
Location* Grids::getVertex(int x,int y){
    return nodes[x+xmax*y];
}


/**
 * @brief 
 * 
 * @param id 
 * @return Location* 
 */
Location *Grids::getVertex(int id){
    return nodes[id];
}


/**
 * @brief 
 * 
 * @param node 
 * @return Configs 
 */
Configs Grids::getNeighbors(Location *node){
    Configs neighbors;
    if(node->x-1 >=0 ){
        auto nc=getVertex(node->x-1,node->y);
        if(nc!=nullptr) neighbors.push_back(nc);
    } 
    if(node->y-1>=0) {
        auto nc=getVertex(node->x,node->y-1);
        if(nc!=nullptr) neighbors.push_back(nc);
    }
    if(node->y+1<ymax) {
        auto nc=getVertex(node->x,node->y+1);
        if(nc!=nullptr) neighbors.push_back(nc);
    }
    if(node->x+1<xmax){
        auto nc=getVertex(node->x+1,node->y);
        if(nc!=nullptr) neighbors.push_back(nc);
    }
}




////////////////////////////////////////////////////////////////////////////


/**
 * @brief 
 * 
 * @param file_name 
 * @param starts 
 * @param goals 
 * @param graph 
 */
void read_scen(std::string file_name,Configs&starts,Configs &goals,Grids *graph){
    starts.clear();
    goals.clear();
    // int xmax=-1,ymax=-1,zmax=-1;
    std::string line;
    std::smatch results;
    std::ifstream scen_file(file_name);
    // std::regex r_xmax=std::regex(R"(xmax=(\d+))");
    // std::regex r_ymax=std::regex(R"(ymax=(\d+))");
    // std::regex r_zmax=std::regex(R"(zmax=(\d+))");
    std::regex r_sg=std::regex(R"((\d+),(\d+),(\d+),(\d+),(\d+),(\d+))");

   
    if(!scen_file){
        std::cout<<"File not found!"<<std::endl;
        exit(0);
    }
    int id=0;
    while(getline(scen_file,line)){
        //CRLF
        if(*(line.end()-1)==0x0d) line.pop_back();
        
        if(std::regex_match(line,results,r_sg)){
            
            int x_s=std::stoi(results[1].str());
            int y_s=std::stoi(results[2].str());
         
            int x_g=std::stoi(results[3].str());
            int y_g=std::stoi(results[4].str());
            starts.emplace_back(graph->getVertex(x_s,y_s));
            goals.emplace_back(graph->getVertex(x_g,y_g));
            // Robot *ri=new Robot(id,graph->getVertex(x_s,y_s,z_s),graph->getVertex(x_g,y_g,z_g));
            id++;
            continue;
        } 
    } 
}


//////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief 
 * 
 * @param file_name 
 * @param paths 
 * @param runtime 
 * @param save_paths 
 */
void save_solutions(std::string file_name,Paths&paths,double runtime,bool save_paths){
    std::ofstream out(file_name);
    int makespanLB,socLB,makespan,soc;
    evaluate_result(paths,makespan,makespanLB,soc,socLB);
    out<<"soc="<<soc<<std::endl;
    out<<"lb_soc="<<socLB<<std::endl;
    out<<"makespan="<<makespan<<std::endl;
    out<<"lb_makespan="<<makespanLB<<std::endl;
    out<<"comp_time="<<(int)(runtime*1000)<<std::endl;
    if(save_paths==false) return;
    out<<"solutions="<<std::endl;
    for(int i=0;i<paths.size();i++){
        out<<i<<":";
     
        for(const auto &v:paths[i]){
           
            out<<'('<<v->x<<","<<v->y<<"),";
        }
        out<<std::endl;
    }
}



/**
 * @brief 
 * 
 * @param paths 
 */
void check_feasible_bruteForce(Paths & paths){
    int makespan=0;
    for(auto &p:paths){
        makespan=std::max(makespan,(int)p.size());
    }

    fill_paths(paths);

    for(int t=1;t<makespan;t++){
        for(int i=0;i<paths.size();i++){
            for(int j=i+1;j<paths.size();j++){
                if(paths[i][t]==paths[j][t]) {
                    printf("Vertex collision (%d,%d,%d,%d)\n",i,j,t,paths[i][j]->id);
                    assert(false);
                }
                if(paths[i][t-1]==paths[j][t] and paths[i][t]==paths[j][t-1]){
                    printf("Edge collision (%d,%d,%d, %d->%d)\n",i,j,t,paths[i][t-1]->id,paths[i][t]->id);
                    assert(false);
                }
            }
        }
    }
    std::cout<<"no collision!"<<std::endl;
}


/**
 * @brief 
 * 
 * @param paths 
 * @param makespan 
 */
void fill_paths(Paths &paths,int makespan){
    if(makespan==-1){
        for(auto &p:paths)makespan=std::max((int)p.size(),makespan);
    }
    for(auto &p:paths){
        while(p.size()<makespan) p.emplace_back(p.back());
    }
}


/**
 * @brief 
 * 
 * @param paths 
 * @param makespan 
 * @param makespanLB 
 * @param soc 
 * @param socLB 
 */
void evaluate_result(Paths &paths,int &makespan,int &makespanLB,int &soc,int &socLB){
    makespan=makespanLB=soc=socLB=0;
    for(auto &p:paths){
        makespan=std::max(makespan,(int)p.size());
        soc+=p.size()-1;
        socLB+=p[0]->manhattan_dist(p.back());
        makespanLB=std::max(makespanLB,p[0]->manhattan_dist(p.back()));
    }
}   

/**
 * @brief 
 * 
 * @param paths 
 */
void shrink_paths(Paths&paths){
    for(auto &p:paths){
        if(p.size()<=1) continue;
        while(p.back()==p.end()[-2]) p.pop_back();
    }
}