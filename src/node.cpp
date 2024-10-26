/// @file   node.cpp
/// @author Evan Brody
/// @brief  Implements the Node class

#include "../inc/node.hpp"
#include "../inc/vector.hpp"
#include "../inc/map.hpp"

#include <iostream>

namespace AStar {
    std::ostream& operator<<(std::ostream& os, const Node& rhs) {
        os << "Node:\n";
        os << "\tPosition:\t" << rhs.getPos() << '\n';
        return os << "\tOrientation:\t" << rhs.getOrientation();
    }

    // This constructor is only used for the starting node
    // Its orientation being 0 prevents a rotation cost being included in the first action
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

    bool Node::sameStateAs(const Node& rhs) const {
        return pos == rhs.pos && orientation == rhs.orientation;
    }

    /// @brief Bitwise-appends four signed 16-bit numbers to create a unique
    /// 64-bit hash value representing the state of the node
    /// The state of the node is defined as the tuple of its position and orientation
    uint64_t Node::getStateHash() const {
        uint64_t hashRes = 0;
        uint16_t posX, posY, oriX, oriY;
        posX = pos.getX(); posY = pos.getY();
        oriX = orientation.getX(); oriY = orientation.getY();

        hashRes |= *(uint64_t*)(&posX);

        hashRes <<= sizeof(oriX) * 8;
        hashRes |= *(uint64_t*)(&posY);

        hashRes <<= sizeof(posY) * 8;
        hashRes |= *(uint64_t*)(&oriX);

        hashRes <<= sizeof(oriX) * 8;
        return hashRes |= *(uint64_t*)(&oriY);
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
}