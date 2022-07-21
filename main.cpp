/**
 * @file main.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include"common.hpp"
#include"flow.hpp"
// #include "dou.hpp"


void test1(){
    Grids * graph=new Grids("./demo/30x30.map");
    Configs starts,goals;
    read_scen("./demo/30x30_agent100.scen",starts,goals,graph);
    std::cout<<"scen file read!  num of robots="<<starts.size()<<std::endl;
    FlowMAPF solver(starts,goals,graph);
    Paths solution=solver.solve();
    check_feasible_bruteForce(solution);
    save_result_as_json("./demo30x30.json",solution,0,true);
    return;
}



int main(){
    test1();
    return 0;
}





