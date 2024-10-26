/// @file   map.cpp
/// @author Evan Brody
/// @brief  Implements the Map class

#include "../inc/map.hpp"
#include "../inc/vector.hpp"
#include "../inc/node.hpp"

#include <iostream>
#include <unordered_set>
#include <vector>
#include <fstream>
#include <cmath>

namespace AStar {
    std::ostream& operator<<(std::ostream& os, const Map& map) {
        // We need to output this way because the grid is stored 
        // in memory as follows:
        // 0 1 2 ... (GRID_HEIGHT - 1)
        // 1
        // 2
        // ...
        // (GRID_WIDTH - 1)
        // Which is rotated 90 degrees clockwise from our visualization
        for (size_t j = GRID_HEIGHT; j > 0; j--) {
            for (size_t i = 0; i < GRID_WIDTH; i++) {
                os << map.grid[i][j - 1] << ' ';
            }
            os << '\n';
        }

        return os;
    }

    Map::Map(const Vector& startPos, const Vector& goalPos,
        std::ifstream& inputFile, const double k)
        : startPos(startPos), goalPos(goalPos), k(k) {
        // Use the input file to initialize the grid
        for (size_t j = GRID_HEIGHT; j > 0; j--) {
            for (size_t i = 0; i < GRID_WIDTH; i++) {
                inputFile >> grid[i][j - 1];
            }
        }
    }

    Map::Map(const Map& rhs)
    : previousStates(rhs.previousStates), startPos(rhs.startPos),
    goalPos(rhs.goalPos), k(rhs.k) {
        // Deep copy grid
        for (size_t i = 0; i < GRID_WIDTH; i++) {
            for (size_t j = 0; j < GRID_HEIGHT; j++) {
                grid[i][j] = rhs.grid[i][j];
            }
        }

        // Deep copy generated and frontier nodes
        for (const Node* nodePtr : rhs.frontier) {
            frontier.push_back(new Node(*nodePtr));
        }

        for (const Node* nodePtr : rhs.generated) {
            generated.push_back(new Node(*nodePtr));
        }
    }

    Map& Map::operator=(const Map& rhs) {
        if (this == &rhs) { return *this; }

        // Clear out our own information
        for (const Node* nodePtr : generated) {
            delete nodePtr;
        }
        
        generated.clear();
        frontier.clear();
        solutionPath.clear();
        previousStates.clear();

        // Copy all fields, deeply where necessary
        startPos = rhs.startPos;
        goalPos = rhs.goalPos;
        k = rhs.k;
        previousStates = rhs.previousStates;

        for (size_t i = 0; i < GRID_HEIGHT; i++) {
            for (size_t j = 0; j < GRID_WIDTH; j++) {
                grid[i][j] = rhs.grid[i][j];
            }
        }

        for (const Node* nodePtr : rhs.generated) {
            generated.push_back(new Node(*nodePtr));
        }

        for (const Node* nodePtr : rhs.frontier) {
            frontier.push_back(new Node(*nodePtr));
        }

        for (const Node* nodePtr : rhs.solutionPath) {
            solutionPath.push_back(new Node(*nodePtr));
        }

        return *this;
    }

    Map::~Map() {
        // Generated is a superset of frontier and solutionPath
        for (const Node* nodePtr : generated) {
            delete nodePtr;
        }

        generated.clear();
        frontier.clear();
        solutionPath.clear();
        previousStates.clear();
    }

    void Map::pathFind() {
        // In this case, no work to do
        if (startPos == goalPos) {
            solutionPath.push_back(new Node(*this, startPos));
            return;
        }

        // Expand the root node to begin
        const Node* startNode = new Node(*this, startPos);
        generated.push_back(startNode);
        expand(*startNode);

        const Node* next = nullptr;
        bool reachedGoal = false;

        // Continually expand the lowest cost node
        // from the frontier until it runs out or we reach the goal node
        while (!(frontier.empty() || reachedGoal)) {
            next = frontier.back();
            frontier.pop_back();

            reachedGoal = expand(*next);
        }

        if (reachedGoal) {
            const Node* backtrack = next; // Next is currently the goal node
            while (backtrack->getPos() != backtrack->getParentNode().getPos()) {
                solutionPath.push_back(backtrack);
                backtrack = &backtrack->getParentNode();
            }
            // Solution path will hold the root and goal nodes
            solutionPath.push_back(backtrack);
            // Put solution path in correct order (start at index 0)
            std::reverse(solutionPath.begin(), solutionPath.end());
        }
        else {
            std::cerr << "Pathfinding failed.\n";
            return;
        }
    }

    void Map::writeToFile(std::ofstream& outputFile) {
        size_t depth = solutionPath.size() - 1;
        size_t numGenerated = generated.size();

        // Write depth
        outputFile << depth << '\n';

        // Write N
        outputFile << numGenerated << '\n';

        // Write actions
        for (size_t i = 1; i < solutionPath.size(); i++) {
            outputFile << solutionPath[i - 1]->getMoveTo(*solutionPath[i]) << ' ';
        }
        outputFile << '\n';

        // Write costs
        for (size_t i = 0; i < solutionPath.size(); i++) {
            // Round to two decimal places
            double rounded = std::ceil(solutionPath[i]->getCost() * 100.0) / 100.0;
            outputFile << rounded << ' ';
        }
        outputFile << '\n';

        // Change grid before writing
        // Exclude start and goal nodes
        for (size_t i = 1; i < solutionPath.size() - 1; i++) {
            const Vector& pos = solutionPath[i]->getPos();
            grid[pos.getX()][pos.getY()] = SOLUTION_PATH;
        }

        // Write grid
        for (size_t j = GRID_HEIGHT; j > 0; j--) {
            for (size_t i = 0; i < GRID_WIDTH; i++) {
                outputFile << grid[i][j - 1] << ' ';
            }
            outputFile << '\n';
        }
    }

    double Map::getK() const { return k; }

    const Vector& Map::getGoalPos() const {
        return goalPos;
    }

    /// @brief Fills a vector with the neighbours of a Node
    ///        in unsorted order
    /// @param parent   Node        the Node to find the neighbours of
    /// @param toFill   std::vector the vector to fill
    void Map::getNeighbours(const Node& parent, std::vector<const Node*>& toFill) const {
        const Vector& parentPos(parent.getPos());

        // Left, middle, and right columns of the 3x3 square
        for (int i = -1; i <= 1; i++) {
            int maybeNeighbourX = parentPos.getX() + i;

            // Check that we're inside the (horizontal) bounds of the grid
            if (maybeNeighbourX < 0 || maybeNeighbourX > GRID_WIDTH - 1) {
                continue;
            }

            // Bottom, middle, and top rows of the 3x3 square
            for (int j = -1; j <= 1; j++) {
                int maybeNeighbourY = parentPos.getY() + j;

                // Check that we're inside the (vertical) bounds of the grid
                if (maybeNeighbourY < 0 || maybeNeighbourY > GRID_HEIGHT - 1) {
                    continue;
                }

                // Check that we're not looking at a wall
                if (WALL == grid[maybeNeighbourX][maybeNeighbourY]) {
                    continue;
                }

                const Node* neighbour = new Node(
                    *this,
                    parent,
                    maybeNeighbourX,
                    maybeNeighbourY
                );

                // Check that we haven't generated this state already
                // Requires C++ 20
                if (previousStates.contains(neighbour->getStateHash())) {
                    delete neighbour;
                    continue;
                }

                // Add to our neighbours vector (unsorted)
                toFill.push_back(neighbour);
            }
        }
    }

    /// @brief Expands a particular node from the frontier
    /// @param toExpand     Node
    /// @return             bool Whether we're expanding the goal node
    bool Map::expand(const Node& toExpand) {
        // If we're expanding the goal node, the algorithm has terminated.
        if (goalPos == toExpand.getPos()) {
            return true;
        }

        // Find all neighbors
        std::vector<const Node*> neighbors;
        getNeighbours(toExpand, neighbors);

        // Store neighbours
        for (const Node* nodePtr : neighbors) {
            generated.push_back(nodePtr);
            previousStates.insert(nodePtr->getStateHash());
        }

        // Insert the neighbors into the frontier while maintaining sorted order
        // (most costly node at index 0)
        for (const Node* nodePtr : neighbors) {
            std::vector<const Node*>::const_iterator it = frontier.begin();

            // Find where it should be in the sorted frontier
            while (it != frontier.end() && **it > *nodePtr) { it++; }
            frontier.insert(it, nodePtr);
        }
        neighbors.clear();

        return false;
    }
}