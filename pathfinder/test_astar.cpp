#include <iostream>
#include <vector>
#include <list>
#include <memory>
#include <math.h>
#include <typeinfo>
#include "Astar.cpp"

int main() {
    Astar A;
    Map map1;

    std::vector<int> mapArray {1, 1, 1, 1, 0, 1, 0,
                        0, 0, 0, 1, 1, 0, 0,
                        0, 0, 1, 1, 1, 1, 1,
                        1, 1, 1, 1, 1, 1, 0};

    int startPoint = 2, endPoint = 13;
    map1.setMap(7, 4, mapArray);
    std::cout << "Astar Path Planning using C++" << std::endl << "Display map:" << std::endl;
    map1.displayMap();
    std::cout << "Start point: "<< startPoint << std::endl;
    std::cout << "End point: "<< endPoint << std::endl;
    
    A.setEndPoint(endPoint);
    A.setStartPoint(startPoint);
    A.createNodeList(map1, startPoint, endPoint);
   
    int numOfSteps = A.planPath();
    std::cout << "Number of Steps: "<< numOfSteps << std::endl;
    std::vector<int> path1 = A.getPath();
    std::cout << "Shortest Path: ";
    if (!path1.empty()) for (int i : path1) std::cout << i << ", ";
    else std::cout << "None "<< std::endl;
    
    return 0;
}