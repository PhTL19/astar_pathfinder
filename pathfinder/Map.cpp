#include <vector>
#include <iostream>
#include <string>

class Map {
    private:
        std::vector<int> currentMap;
        int column;
        int row;
        int moveDirection[8] = {-1, 0,    // top
                                0, 1,    // right
                                1, 0,    // bottom
                                0, -1};  // left

    public:
        Map() {}
        
        void displayMap();
        void setMap(int col, int row, std::vector<int> maparr);
        std::vector<int> getMap();
        void deleteMap();
        int returnColumn();
        int returnRow();
        void setColumn(int colCount);
        void setRow(int rowCount);
        int* returnDirection();
};

void Map::displayMap() {
    std::vector<int> displayLayout = currentMap;
    int node = 1;
    for (int i = 0; i < row; i++) {
        for (int j = 0; j < column; j++) {
            if (displayLayout[i*column + j] == 1) {
                std::cout << node++ <<"\t";
            } else {
                std::cout << "x" << "\t";
            }
        }
        std::cout << std::endl;
    }
}

void Map::setMap(int col, int row, std::vector<int> maparr) {
    int rowCount = 0;
    int colCount = 0;
    // Map array: line = 1; obstacle = 0;
    if (!maparr.empty() && col > 0 && row > 0) {
        setColumn(col);
        setRow(row);
        currentMap = maparr;
    } else {
        std::cout << "The map property entered is not correct";
        setColumn(0);
        setRow(0);
    }
}

std::vector<int> Map::getMap() {
    return currentMap;
}

int Map::returnColumn() {
    return column;
}

int Map::returnRow() {
    return row;
}

void Map::setColumn(int colCount) {
    column = colCount;
}

void Map::setRow(int rowCount) {
    row = rowCount;
}

int* Map::returnDirection() {
    return moveDirection;
}