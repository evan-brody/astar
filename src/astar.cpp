/// @file   astar.cpp
/// @author Evan Brody
/// @brief  Uses the A* algorithm to pathfind through a given maze

#include "../inc/vector.hpp"
#include "../inc/map.hpp"

#include "../inc/node.hpp"

#include <fstream>
#include <iostream>

using namespace AStar;

const std::string solutionSuffix = "_solution.txt";

int main() {
    std::string inputFilename;
    std::cout << "Enter the input file\'s name: ";
    std::cin >> inputFilename;

    // Open input file
    std::ifstream mapData(inputFilename);
    if (!mapData) {
        std::cerr << "Failed to open input file.\n";
        exit(1);
    }

    // Read start pos
    int x, y;
    mapData >> x >> y;
    Vector startPos(x, y);

    // Read goal pos
    mapData >> x >> y;
    Vector goalPos(x, y);

    double k;
    std::cout << "Enter a value for k: ";
    std::cin >> k;

    // Initialize map
    Map map(startPos, goalPos, mapData, k);
    mapData.close();

    map.pathFind();

    // Create solution file
    size_t lastDotPos = inputFilename.find_last_of('.'); // Remove file extension
    std::ofstream solutionFile(
        inputFilename.substr(0, lastDotPos) + solutionSuffix
    );
    // Output solution info to file
    map.writeToFile(solutionFile);
    solutionFile.close();

    return 0;
}