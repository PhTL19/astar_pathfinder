// #include <stdio.h>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/core/core.hpp>
#include <iostream>
#include <vector>
#include <list>
#include <memory>
#include <math.h>
#include <typeinfo>
#include "LayoutNodes.cpp"
#include "Map.cpp"

#define OBSTACLE 9999
// #define testing

class Astar {
    private:
        int startPoint;
        int endPoint;
        int mapColumn;
        int mapRow;
        std::list<LayoutNodes> openList;
        std::list<LayoutNodes> closedList;
        std::vector<LayoutNodes> nodeList;
        std::vector<double> neighbourNodes;
        std::list<LayoutNodes> path;

    public:
        Astar() {
            startPoint = 0;
            endPoint = 0;
            mapColumn = 0;
            mapRow = 0;
        }
        
        void setStartPoint(int startIndex);
        void setEndPoint(int endIndex);
        
        // calculates path cost between current node and parent node.
        void calcPathCost(int id);
        
        //  * The following function calculates the straight line cost between current node and end(goal) point.
        void calcHeuristicCost(int node, LayoutNodes goal);
        
        //  * The following function creates a list of possible nodes (neglecting obstacles) for the given Map.
        bool createNodeList(Map mapLayout, int startPt, int endPt);
        
        //  * The following function implements the A* algorithm and identifies the shortest path.
        int planPath();
        
        //  * The following function identifies the node index from the row and column values of the node.
        int identifyNode(int x, int y);
        
        //  * The following function returns true if a node of the given is in the openList, else returns false.
        bool inOpenList(int id);
        
        //  * The following function returns true if a node of the given is in the closeList, else returns false.
        bool inClosedList(int id);
        
        //  * Function displayMap of type Mat (OpenCV Matrix)
        // cv::Mat displayMap();

        std::vector<int> getPath();
};

bool priority(LayoutNodes &node1, LayoutNodes &node2) {
    if (node1.getCost() < node2.getCost())
        return true;
    else
        return false;
}

bool Astar::createNodeList(Map mapLayout, int startPt, int endPt) {
    std::vector<int> map = mapLayout.getMap();
    mapColumn = mapLayout.returnColumn();
    mapRow = mapLayout.returnRow();
    int index = 0;
    LayoutNodes node;
    for (int i = 0; i < mapRow; i++) {
        for (int j = 0; j < mapColumn; j++) {
            if (map[i*mapColumn + j] == 1) {
                node.setNodeIndex(index++, j, i);
                nodeList.emplace_back(node);
            }
        }
    }
    #ifdef testing
    for (int i =0; i < index; i++) {
        nodeList[i].output();
    }
    #endif
    if (startPt >= 0 && endPt >= 0 && (unsigned)startPt < nodeList.size() && (unsigned)endPt < nodeList.size()) {
        setStartPoint(startPt);
        setEndPoint(endPt);
        return true;
    } else {
        return false;
    }
}

void Astar::setStartPoint(int startIndex) {
    startPoint = startIndex;
}

void Astar::setEndPoint(int endIndex) {
    endPoint = endIndex;
}

int Astar::planPath() {
    nodeList[startPoint].setHeuristicCost(1);
    nodeList[startPoint].setPathCost(0);
    nodeList[startPoint].setTotalCost();

    for (auto a : nodeList) {
        calcHeuristicCost(a.getIndex(), nodeList[endPoint]);
    }

    openList.emplace_back(nodeList[startPoint]);
    #ifdef testing
    for (auto i : openList) {
        std::cout << "cost" << i.getCost() << std::endl;
    }
    // std::cout << "openList: " << openList.size() << std::endl;
    #endif

    Map map;
    auto directions = map.returnDirection();
    int finalFoundFlag = 0;
    while (!openList.empty()) {
        openList.sort(priority);
        LayoutNodes currentNode = openList.front();
        closedList.emplace_back(currentNode);
        // std::cout << "after transfer: " << currentNode.getCost() << std::endl;
        openList.pop_front();

        if (currentNode.getIndex() == nodeList[endPoint].getIndex()) {
            // std::cout << "final found";
            finalFoundFlag = 1;
            #ifdef testing
            for (auto i : closedList) {
                std::cout << "\n Node Index: "<< i.getIndex() << " hcost: " << i.returnHCost() << "  totalcost: " << i.getCost() << " Parent: " << i.getParentIndex();
            }
            #endif
            break;
        } else {
        // neighbour
        int cRow = currentNode.getRowIndex();
        int cCol = currentNode.getColumnIndex();
        // std::cout << "\ncrow " << cRow << "\tccol " << cCol;
        std::vector<int> neighbourID;
        for (int i = 0; i < 4; i++) {
            int x = directions[2*i] + cRow;
            int y = directions[2*i +1] + cCol;
            // std::cout << "\nx: " << x << "y: " << y << std::endl;
            if (x < 0 || y < 0 || x > 3 || y > 6) {
            continue;
            } else {
                int id = identifyNode(x, y);
                if (id != OBSTACLE) {
                    bool closed = inClosedList(id);
                    if (closed == true) {
                        continue;
                    } else {
                        bool open1 = inOpenList(id);
                        if (open1 == false) {
                            neighbourID.emplace_back(id);
                        }
                    }
                }
            }
        }
        for (auto i : neighbourID) {
            nodeList[i].setParentIndex(currentNode.getIndex());
            nodeList[i].setPathCost(1);
            nodeList[i].setTotalCost();
            openList.push_back(nodeList[i]);
            #ifdef testing
            std::cout << "\nThe openlist size is: " << openList.size();
            std::cout << "\nThe closedlist size is: " << closedList.size();
            #endif
        }
        }
    }
    std::list<LayoutNodes> pathBuffer = closedList;
    pathBuffer.reverse();
    LayoutNodes frontNode = pathBuffer.front();
    path.emplace_back(frontNode);
    pathBuffer.pop_front();
    for (auto i : pathBuffer) {
        LayoutNodes temp = path.back();
        if (temp.getParentIndex() == i.getIndex()) {
            path.emplace_back(i);
        } else {
            continue;
        }
    }
    #ifdef testing
    for (auto i : path) {
        std::cout << "\nPath list: " << i.getIndex();
    }
    #endif
    // cv::Mat bufferMat = displayMap();
    int shortestPathLength = 0;
    if (finalFoundFlag == 0) {
        shortestPathLength = -1;
    } else {
        shortestPathLength = (closedList.size()-1);
    }
    return shortestPathLength;
}

bool Astar::inOpenList(int id) {
    int openFlag = 0;
    for (auto l : openList) {
        if (l.getIndex() == id) {
            if (l.getCost() > nodeList[id].getCost()) {
                l.setCost(nodeList[id].getCost());
            }
            openFlag = 1;
        }
    }
    if (openFlag == 1) {
        return true;
    } else {
        return false;
    }
}

int Astar::identifyNode(int x, int y) {
    int found = 0;
    int index = 0;
    for (auto n : nodeList) {
        // std::cout<<"\n row col"<<n.getRowIndex()<<" "<<n.getColumnIndex();
        if (n.getRowIndex() == x && n.getColumnIndex() == y) {
            found = 1;
            index = n.getIndex();
        }
    }
    if (found == 1) {
        return index;
    } else {
        return OBSTACLE;
    }
}

bool Astar::inClosedList(int id) {
    int closedFlag = 0;
    for (auto l : closedList) {
        if (l.getIndex() == id)
           closedFlag = 1;
    }
    if (closedFlag == 1)
        return true;
    else
        return false;
}

void Astar::calcHeuristicCost(int node, LayoutNodes goal) {
    int x1 = nodeList[node].getColumnIndex();
    int y1 = nodeList[node].getRowIndex();
    int x2 = goal.getColumnIndex();
    int y2 = goal.getRowIndex();
    double distance = sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    nodeList[node].setHeuristicCost(distance);
}

std::vector<int> Astar::getPath(){
    std::vector<int> pathIdx;
    int index = 0;
    std::list<LayoutNodes> pathBuffer = closedList;
    pathBuffer.reverse();
    LayoutNodes frontNode = pathBuffer.front();
    path.emplace_back(frontNode);
    pathBuffer.pop_front();
    for (auto i : pathBuffer) {
        LayoutNodes temp = path.back();
        if (temp.getParentIndex() == i.getIndex()) {
            path.emplace_back(i);
        } else {
            continue;
        }
    }
    for (auto i : path) {
        std::cout << i.getIndex() << ", ";
    }
    return pathIdx;
}

// cv::Mat Astar::displayMap() {
//     int sqW = 50;
//     int iSqW = 45;
//     int xLength = mapColumn*sqW, yLength = mapRow*sqW+40;
//     drawing = Mat::zeros(Size(xLength, yLength), CV_8UC3);
//     for (int i = 0; i < mapRow; i++) {
//         for (int j = 0; j < mapColumn; j++) {
//             int pathFlag = 0;
//             int blockFlag = 1;
//             int firstPt = 0, lastPt = 0;
//             for (auto n : path) {
//                 if (n.getRowIndex() == i && n.getColumnIndex() == j) {
//                     pathFlag = 1;
//                     if (n.getIndex() == startPoint)
//                        firstPt = 1;
//                     else if (n.getIndex() == endPoint)
//                        lastPt = 1;
//                 }
//             }
//             for (auto n : nodeList) {
//                 if (n.getRowIndex() == i && n.getColumnIndex() == j) {
//                     blockFlag = 0;
//                 }
//             }
//             if (pathFlag == 1) {
//                 rectangle(drawing, Point(j*sqW, i*sqW), Point(j*sqW+sqW, i*sqW+sqW), Scalar(255, 255, 255), 1, CV_AA, 0);
//                 if (firstPt == 1)
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(0, 255, 255), -1, CV_AA, 0);
//                 else if (lastPt == 1)
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(0, 0, 255), -1, CV_AA, 0);
//                 else
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(0, 255, 0), -1, CV_AA, 0);
//             } else if (blockFlag == 1) {
//                 rectangle(drawing, Point(j*sqW, i*sqW), Point(j*sqW+sqW, i*sqW+sqW), Scalar(255, 255, 255), 1, CV_AA, 0);
//                 rectangle(drawing, Point(j*sqW+2, i*sqW+2), Point(j*sqW+2+iSqW, i*sqW+2+iSqW), Scalar(128, 128, 128), -1, CV_AA, 0);
//             } else {
//                 rectangle(drawing, Point(j*sqW, i*sqW), Point(j*sqW+sqW, i*sqW+sqW), Scalar(255, 255, 255), 1, CV_AA, 0);
//               }
//        }
//     }
//     rectangle(drawing, Point(10, yLength-35), Point(20, yLength-25), Scalar(0, 255, 255), -1, CV_AA, 0);
//     putText(drawing, "- Start Point", Point(25, yLength-25), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
//     rectangle(drawing, Point(100, yLength-35), Point(110, yLength-25), Scalar(0, 0, 255), -1, CV_AA, 0);
//     putText(drawing, "- End Point", Point(125, yLength-25), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
//     rectangle(drawing, Point(10, yLength-15), Point(20, yLength-5), Scalar(0, 255, 0), -1, CV_AA, 0);
//     putText(drawing, "- Path", Point(25, yLength-10), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
//     rectangle(drawing, Point(100, yLength-15), Point(110, yLength-5), Scalar(128, 128, 128), -1, CV_AA, 0);
//     putText(drawing, "- Obstacle", Point(125, yLength-10), FONT_HERSHEY_SIMPLEX, 0.3, Scalar(255, 255, 255), 1, 16);
// return drawing;
// }

