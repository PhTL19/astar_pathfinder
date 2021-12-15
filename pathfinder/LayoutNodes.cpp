#include <iostream>

class LayoutNodes {
    private:
        int nodeIndex;
        int parentIndex;
        int rowIndex;
        int columnIndex;
        double heuristicCost;
        double pathCost;
        double totalCost;

    public:
        LayoutNodes() {
            nodeIndex = 999;
            parentIndex = 9999;
            rowIndex = 0;
            columnIndex = 0;
            heuristicCost = 0;
            pathCost = 0;
            totalCost = 0;
        }
        void setNodeIndex(int index, int column, int row);
        void setParentIndex(int pIndex);
        int getParentIndex();
        void setHeuristicCost(double heuristic);
        void setPathCost(double cost);
        void setTotalCost();
        void setCost(double cost);
        void output();
        double getCost();
        int getRowIndex();
        int getColumnIndex();
        int getIndex();
        double returnHCost();
};

void LayoutNodes::setNodeIndex(int index, int column, int row) {
    nodeIndex = index;
    columnIndex = column;
    rowIndex = row;
}

void LayoutNodes::setParentIndex(int pIndex) {
    parentIndex = pIndex;
}

int LayoutNodes::getParentIndex() {
    return parentIndex;
}

void LayoutNodes::setHeuristicCost(double heuristic) {
    heuristicCost = heuristic;
}

void LayoutNodes::setPathCost(double cost) {
    pathCost = cost;
}

void LayoutNodes::setTotalCost() {
    totalCost = pathCost + 0.1*heuristicCost;
}

void LayoutNodes::setCost(double cost) {
    totalCost = cost;
}

void LayoutNodes::output() {
    std::cout << rowIndex << " " << columnIndex << std::endl;
}

double LayoutNodes::getCost() {
    return totalCost;
}

int LayoutNodes::getRowIndex() {
    return rowIndex;
}

int LayoutNodes::getColumnIndex() {
    return columnIndex;
}

int LayoutNodes::getIndex() {
    return nodeIndex;
}

double LayoutNodes::returnHCost() {
    return heuristicCost;
}