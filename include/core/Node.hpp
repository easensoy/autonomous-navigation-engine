#pragma once
#include <string>
#include <iostream>

/*
- Core Node class representing a location in our navigation system. 
- This class follows the single responsibility principle - it only manages the
data and basic operations for a single point in space.
- This implementation is split across multiple files to demonstrate how complex
classes can be organized in large-scale projects.  
*/

class Node 
{
    private:
        int id;
        std::string name;
        double x,y;

    
    public:
        // Constructor operations (implemented in node_constructor.cpp)
        Node(int nodeId, const std::string& nodeName, double xPos = 0.0, double yPos = 0.0);
        Node(const Node& other);
        Node& operator=(const Node& other);
        ~Node() = default;
        
        // Getter operations (implemented in node_getters.cpp)
        int getId() const;
        const std::string& getName() const;
        double getX() const;
        double getY() const;

        // Comparison operators (implemented in node_operators.cpp)
        bool operator==(const Node& other) const;
        bool operator!=(const Node& other) const;
        bool operator<(const Node& other) const; //For sorting operations

        // Distance calculations (implemented in distance_calculation.cpp)
        double distanceTo(const Node& other) const;
        double euclideanDistance(const Node& other) const;
        double manhattanDistance(const Node& other) const;

        friend std::ostream& operator<<(std::ostream& os, const Node& node);
};