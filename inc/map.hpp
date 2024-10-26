/// @file   map.hpp
/// @author Evan Brody
/// @brief  Outlines the Map class

#ifndef MAP_HPP
#define MAP_HPP

#include "vector.hpp"

#include <vector>
#include <iostream>
#include <fstream>

// MACROS
#define GRID_HEIGHT 30
#define GRID_WIDTH 50
#define WALL 1
#define START 2
#define SOLUTION_PATH 4
#define GOAL 5

namespace AStar {
    class Node;

    class Map {
        friend std::ostream& operator<<(std::ostream& os, const Map& map);

    public:
        Map(const Vector& startPos, const Vector& goalPos,
            std::ifstream& inputFile, const double k);

        Map(const Map& rhs);

        Map& operator=(const Map& rhs);

        ~Map();

        void pathFind();

        void writeToFile(std::ofstream& outputFile);

        double getK() const;

        const Vector& getGoalPos() const;

    private:

        /// @brief Fills a vector with the neighbours of a Node
        ///        in unsorted order
        /// @param parent   Node        the Node to find the neighbours of
        /// @param toFill   std::vector the vector to fill
        void getNeighbours(const Node& parent, std::vector<const Node*>& toFill) const;

        /// @brief Expands a particular node from the frontier
        /// @param toExpand     Node
        /// @return             bool Whether we're expanding the goal node
        bool expand(const Node& toExpand);

        int grid[GRID_WIDTH][GRID_HEIGHT];
        bool visited[GRID_WIDTH][GRID_HEIGHT];
        std::vector<const Node*> frontier;
        std::vector<const Node*> generated;
        std::vector<const Node*> solutionPath;
        Vector startPos;
        Vector goalPos;
        double k;
    };
}

#endif