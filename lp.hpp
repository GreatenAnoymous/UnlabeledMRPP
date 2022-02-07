/**
 * @file lp.hpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-02-07
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#pragma once
#include<algorithm>
#include<iostream>
#include<vector>
#include<unordered_map>


double lba_sparse(std::vector<std::tuple<int,int,double>>&costEdges, std::vector<int>& Assignment); //linear bottleneck assignment
double lsa_sparse(std::vector<std::tuple<int,int,double>>&costEdges, std::vector<int>& Assignment); //linear sum assignment




