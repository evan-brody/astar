/// @file   astar.cpp
/// @author Evan Brody
/// @brief  Implements the A* algorithm

#include <fstream>
#include <iostream>
#include <vector>
#include <unordered_set>
#include <numbers>
#include <algorithm>
#include <cmath>

// MACROS
#define GRID_HEIGHT 30
#define GRID_WIDTH 50
#define WALL 1
#define START 2
#define SOLUTION_PATH 4
#define GOAL 5

// CONSTANTS
const std::string inputFilename = "sample_input.txt";

// Represents a position in the 2D plane
class Vector {
    friend std::ostream& operator<<(std::ostream& os, const Vector& rhs);
public:
    Vector(int x, int y);

    Vector(const Vector& rhs);

    friend bool operator==(const Vector& lhs, const Vector& rhs);
    friend Vector& operator+=(Vector& lhs, const Vector& rhs);
    friend Vector operator-(const Vector& rhs);
    friend double operator*(const Vector& lhs, const Vector& rhs);

    Vector abs() const;

    int getX() const;

    int getY() const;

    int getManhattanDistance(const Vector& other) const;

    double getMagnitude() const;

    // In degrees
    double getAngleBetween(const Vector& other) const;

    double dist(const Vector& other) const;

private:
    int x, y;
};

bool operator!=(const Vector& lhs, const Vector& rhs);
Vector operator+(Vector lhs, const Vector& rhs);
Vector operator-(Vector lhs, const Vector& rhs);

class Map;

// Represents a node in the search tree
class Node {
    friend std::ostream& operator<<(std::ostream& os, const Node& rhs);
public:
    // This constructor is only used for the starting node
    Node(const Map& parentMap, const Vector& pos);

    Node(const Map& parentMap, const Node& parentNode, const Vector& pos);

    Node(const Map& parentMap, const Node& parentNode, int x, int y);

    friend bool operator==(const Node& lhs, const Node& rhs);
    friend bool operator<(const Node& lhs, const Node& rhs);

    const Vector& getPos() const;

    const Vector& getOrientation() const;

    double dist(const Vector& destination) const;

    double dist(const Node& destination) const;

    int getMoveTo(const Node& destination) const;

    const Node& getParentNode() const;

    // f(n) = g(n) + h(n)
    double getCost() const;

    /// @brief Calculates the local action cost of moving from this node to destination
    /// @param destination  Node
    /// @return             double The calculated action cost
    double getActionCost(const Node& destination) const;

    /// @brief Determines whether the argument node is our neighbour.
    ///        Our neighbours are defined as the eight adjacent positions on the grid.
    /// @param other    Node
    /// @return         bool
    bool isNeighbor(const Node& other) const;
private:
    /// @brief Calculates the costs of moving to this node from the starting node
    void calculateCost();

    const Map& parentMap;
    const Node& parentNode;
    Vector pos;
    Vector orientation;
    double pathCost;
    double heuristicCost;
};

bool operator!=(const Node& lhs, const Node& rhs);
bool operator>(const Node& lhs, const Node& rhs);
bool operator<=(const Node& lhs, const Node& rhs);
bool operator>=(const Node& lhs, const Node& rhs);

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

int main() {
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

    // Output solution info to file
    std::ofstream solutionFile("solution.txt");
    map.writeToFile(solutionFile);
    solutionFile.close();

    return 0;
}

// VECTOR

std::ostream& operator<<(std::ostream& os, const Vector& rhs) {
    return os << '(' << rhs.x << ", " << rhs.y << ')';
}

Vector::Vector(int x, int y) : x(x), y(y) {}

Vector::Vector(const Vector& rhs) : x(rhs.x), y(rhs.y) {}

bool operator==(const Vector& lhs, const Vector& rhs) {
    return lhs.x == rhs.x && lhs.y == rhs.y;
}

bool operator!=(const Vector& lhs, const Vector& rhs) {
    return !(lhs == rhs);
}

// Overloading necessary arithmetic operators
Vector& operator+=(Vector& lhs, const Vector& rhs) {
    lhs.x += rhs.x; lhs.y += rhs.y;
    return lhs;
}

// Negation
Vector operator-(const Vector& rhs) {
    return Vector(-rhs.x, -rhs.y);
}

// Plus and minus
Vector operator+(Vector lhs, const Vector& rhs) {
    return lhs += rhs;
}

Vector operator-(Vector lhs, const Vector& rhs) {
    return lhs += (-rhs);
}

// Dot product
double operator*(const Vector& lhs, const Vector& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

// Absolute value
Vector Vector::abs() const {
    return Vector(std::abs(x), std::abs(y));
}

int Vector::getX() const { return x; }

int Vector::getY() const { return y; }

int Vector::getManhattanDistance(const Vector& other) const {
    return std::abs(this->x - other.x) + std::abs(this->y - other.y);
}

double Vector::getMagnitude() const {
    return sqrt(x * x + y * y);
}

// In degrees
double Vector::getAngleBetween(const Vector& other) const {
    double denominator = getMagnitude() * other.getMagnitude();
    if (0 == denominator) { return 0; }

    double radians = acos((*this * other) / denominator);

    return radians * 180.0 / std::numbers::pi;
}

double Vector::dist(const Vector& other) const {
    double xDiff = this->x - other.x;
    double yDiff = this->y - other.y;

    return sqrt(xDiff * xDiff + yDiff * yDiff);
}

// NODE

std::ostream& operator<<(std::ostream& os, const Node& rhs) {
    return os << "Node: " << rhs.getPos();
}

// This constructor is only used for the starting node
Node::Node(const Map& parentMap, const Vector& pos)
    : parentMap(parentMap), parentNode(*this), pos(pos), orientation(0, 0),
    pathCost(0), heuristicCost(dist(parentMap.getGoalPos())) {}

Node::Node(const Map& parentMap, const Node& parentNode, const Vector& pos)
    : parentMap(parentMap), parentNode(parentNode), pos(pos),
    orientation(this->pos - parentNode.pos) {
    calculateCost();
}

Node::Node(const Map& parentMap, const Node& parentNode, int x, int y)
    : parentMap(parentMap), parentNode(parentNode), pos(x, y),
    orientation(this->pos - parentNode.pos) {
    calculateCost();
}

bool operator==(const Node& lhs, const Node& rhs) {
    return lhs.getCost() == rhs.getCost();
}

bool operator!=(const Node& lhs, const Node& rhs) {
    return !(lhs == rhs);
}

bool operator<(const Node& lhs, const Node& rhs) {
    return lhs.getCost() < rhs.getCost();
}

bool operator>(const Node& lhs, const Node& rhs) {
    return !(lhs == rhs || lhs < rhs);
}

bool operator<=(const Node& lhs, const Node& rhs) {
    return !(lhs > rhs);
}

bool operator>=(const Node& lhs, const Node& rhs) {
    return !(lhs < rhs);
}

const Vector& Node::getPos() const { return pos; }

const Vector& Node::getOrientation() const { return orientation; }

double Node::dist(const Vector& destination) const {
    return this->pos.dist(destination);
}

double Node::dist(const Node& destination) const {
    return this->pos.dist(destination.pos);
}

int Node::getMoveTo(const Node& destination) const {
    // 3 2 1
    // 4 . 0
    // 5 6 7
    const Vector dist = destination.getPos() - pos;
    switch (dist.getX()) {
    case -1: // _W
        switch (dist.getY()) {
        case -1: // (-1, -1) == SW
            return 5;
        case 0: // (-1, 0) == W
            return 4;
        case 1: // (-1, 1) == NW
            return 3;
        }
        break;
    case 0:
        switch (dist.getY()) {
        case -1: // (0, -1) == S
            return 6;
        case 0: // Moving to yourself ?
            return -1;
        case 1: // (0, 1) == N
            return 2;
        }
        break;
    case 1: // _E
        switch (dist.getY()) {
        case -1: // (1, -1) == SE
            return 7;
        case 0: // (1, 0) == E
            return 0;
        case 1: // (1, 1) == NE
            return 1;
        }
    }

    return -1;
}

const Node& Node::getParentNode() const {
    return parentNode;
}

// f(n) = g(n) + h(n)
double Node::getCost() const { return pathCost + heuristicCost; }

/// @brief Calculates the local action cost of moving from this node to destination
/// @param destination  Node
/// @return             double The calculated action cost
double Node::getActionCost(const Node& destination) const {
    Vector distance = destination.pos - this->pos;
    double distCost = distance.getMagnitude();

    // In degrees
    double deltaAngle = orientation.getAngleBetween(distance);
    double angleCost = deltaAngle * parentMap.getK() / 180.0;

    return distCost + angleCost;
}

/// @brief Determines whether the argument node is our neighbour.
///        Our neighbours are defined as the eight adjacent positions on the grid.
/// @param other    Node
/// @return         bool
bool Node::isNeighbor(const Node& other) const {
    Vector dist((this->pos - other.pos).abs());
    return (dist.getX() <= 1) && (dist.getY() <= 1);
}

/// @brief Calculates the costs of moving to this node from the starting node
void Node::calculateCost() {
    this->pathCost = parentNode.pathCost + parentNode.getActionCost(*this);
    this->heuristicCost = this->dist(parentMap.getGoalPos());
}

// MAP

std::ostream& operator<<(std::ostream& os, const Map& map) {
    for (size_t j = GRID_HEIGHT; j > 0; j--) {
        for (size_t i = 0; i < GRID_WIDTH; i++) {
            os << map.grid[i][j - 1] << ' ';
        }
        os << '\n';
    }

    os << '\n';

    for (size_t j = GRID_HEIGHT; j > 0; j--) {
        for (size_t i = 0; i < GRID_WIDTH; i++) {
            os << map.visited[i][j - 1] << ' ';
        }
        os << '\n';
    }

    return os;
}

Map::Map(const Vector& startPos, const Vector& goalPos,
    std::ifstream& inputFile, const double k)
    : startPos(startPos), goalPos(goalPos), k(k) {
    for (size_t j = GRID_HEIGHT; j > 0; j--) {
        for (size_t i = 0; i < GRID_WIDTH; i++) {
            inputFile >> grid[i][j - 1];
            visited[i][j - 1] = false;
        }
    }
}

Map::Map(const Map& rhs) : startPos(rhs.startPos), goalPos(rhs.goalPos), k(rhs.k) {
    // Deep copy grid
    for (size_t i = 0; i < GRID_WIDTH; i++) {
        for (size_t j = 0; j < GRID_HEIGHT; j++) {
            grid[i][j] = rhs.grid[i][j];
            visited[i][j] = rhs.visited[i][j];
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

    // Copy all fields, deeply where necessary
    startPos = rhs.startPos;
    goalPos = rhs.goalPos;
    k = rhs.k;

    for (size_t i = 0; i < GRID_HEIGHT; i++) {
        for (size_t j = 0; j < GRID_WIDTH; j++) {
            grid[i][j] = rhs.grid[i][j];
            visited[i][j] = rhs.visited[i][j];
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
    double parentCost = parent.getCost();
    const Vector& parentOrientation = parent.getOrientation();

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

            // Check that we're not looking at a wall or
            // previously visited position
            if (WALL == grid[maybeNeighbourX][maybeNeighbourY] ||
                visited[maybeNeighbourX][maybeNeighbourY]) {
                continue;
            }

            const Node* neighbour = new Node(
                *this,
                parent,
                maybeNeighbourX,
                maybeNeighbourY
            );

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

    // If there are no neighbors, we have no work to do
    if (neighbors.empty()) { return false; }

    // Store neighbours
    for (const Node* nodePtr : neighbors) {
        const Vector& nodePos = nodePtr->getPos();
        visited[nodePos.getX()][nodePos.getY()] = true;
        generated.push_back(nodePtr);
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