/**
   \file main.cpp
   \brief Main entry point of the program, starts an instance of Planner
*/

//###################################################
//  HYBRID A* ALGORITHM
//  AUTHOR:   Karl Kurzer
//  WRITTEN:  2015-03-02
//###################################################

#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"

//###################################################
//  COUT STANDARD MESSAGE
//###################################################
/**
   \fn message(const T& msg, T1 val = T1())
   \brief Convenience method to display text
*/
template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
    if (!val) {
        std::cout << "### " << msg << std::endl;
    } else {
        std::cout << "### " << msg << val << std::endl;
    }
}

//###################################################
//  MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
int main(int argc, char** argv) {
    // author message
    message<string, int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");

    message("cell size: ", HybridAStar::Constants::cellSize);

    // if statement use a const variable
    if (HybridAStar::Constants::manual) {
        message("mode: ", "manual");
    } else {
        message("mode: ", "auto");
    }

    // init ros node
    ros::init(argc, argv, "a_star");

    // main function
    HybridAStar::Planner hy;
    hy.plan();  // main process

    ros::spin();
    return 0;

}
