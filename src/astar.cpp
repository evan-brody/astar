﻿/// @file   astar.cpp
/// @author Evan Brody
/// @brief  Uses the A* algorithm to pathfind through a given maze

#include "../inc/vector.hpp"
#include "../inc/map.hpp"
#include "../inc/node.hpp"

#include <fstream>
#include <iostream>
#include <format> // C++20
#include <cmath>

using namespace AStar;

int main(int argc, char* argv[]) {
    if (2 != argc) {
        std::cerr << "ERROR:\nUsage: astar <filename>\n";
        exit(1);
    }
    std::string inputFilename(argv[1]);

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

    const std::string solutionSuffix = std::format("_k={}_solution.txt", round(k * 100) / 100.0);

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