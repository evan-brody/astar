/// @file   node.hpp
/// @author Evan Brody
/// @brief  Outlines the Node class

#ifndef NODE_HPP
#define NODE_HPP

#include "vector.hpp"

#include <iostream>

namespace AStar {
    class Map;

    // Represents a node in the search tree
    class Node {
        friend std::ostream& operator<<(std::ostream& os, const Node& rhs);
    public:
        // This constructor is only used for the starting node
        Node(const Map& parentMap, const Vector& pos);

        Node(const Map& parentMap, const Node& parentNode, const Vector& pos);

        Node(const Map& parentMap, const Node& parentNode, int x, int y);

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

    bool operator==(const Node& lhs, const Node& rhs);
    bool operator<(const Node& lhs, const Node& rhs);
    bool operator!=(const Node& lhs, const Node& rhs);
    bool operator>(const Node& lhs, const Node& rhs);
    bool operator<=(const Node& lhs, const Node& rhs);
    bool operator>=(const Node& lhs, const Node& rhs);
}

#endif