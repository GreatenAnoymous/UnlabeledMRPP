cmake_minimum_required(VERSION 3.16)
project(amapf CXX)

#find_package(yaml-cpp REQUIRED)
#add_subdirectory(./graph)
set(CMAKE_THREAD_PREFER_PTHREAD TRUE)
# set(THREADS_PREFER_PTHREAD_FLAG TRUE)
# find_package(Threads REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options system filesystem)
include_directories( ${Boost_INCLUDE_DIRS} )


set(CMAKE_BUILD_TYPE Debug)
IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE "RELEASE")
ENDIF()
set(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0  -g2 -ggdb")
set(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3")



add_executable(main common.cpp lp.cpp search.cpp schedule.cpp flow.cpp)
target_compile_features(main PUBLIC cxx_std_17)
# set(GUROBI_INC /opt/gurobi912/linux64/include/)
# set(GUROBI_LIB "/opt/gurobi912/linux64/lib/libgurobi_c++.a" "/opt/gurobi912/linux64/lib/libgurobi91.so")
target_include_directories(main  PRIVATE)# ${GUROBI_INC})
target_link_libraries(main  ${Boost_LIBRARIES} /usr/local/lib/libortools.so)

# target_link_libraries(main yaml-cpp ${Boost_LIBRARIES})# ${GUROBI_LIB})
