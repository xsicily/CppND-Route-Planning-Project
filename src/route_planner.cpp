#include "route_planner.h"
#include <algorithm>

// Compare the f values of two nodes
bool Compare (const RouteModel::Node *node_A, const RouteModel::Node *node_B){
    auto f1 = node_A->g_value + node_A->h_value; // f1 = g1 + h1
    auto f2 = node_B->g_value + node_B->h_value; // f2 = g2 + h2
    return f1 > f2;
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x,start_y);
    end_node = &m_Model.FindClosestNode(end_x,end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node -> distance(*end_node); // distance function in "route_model.h" 

}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    
    current_node -> FindNeighbors(); //populate current_node.neighbors vector

    auto neighbors = current_node->neighbors; // Get neighbors

    for (auto &neighbor : neighbors){
        // Set parent
        neighbor ->parent = current_node;
        // Set g_value
        neighbor ->g_value = current_node->g_value + neighbor ->distance(*current_node);
        // Calculate h_value
        neighbor ->h_value = CalculateHValue(neighbor);
        // Store neighbor in open_list 
        open_list.push_back(neighbor);
        // Set neighbor's visited attribute 
        neighbor->visited = true;
    }
    
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {
    
    RouteModel::Node *node_ptr;

    std::sort(open_list.begin(), open_list.end(), Compare); // Sort the vector in descending order
    node_ptr = open_list.back(); // the last node has the lowest sum
    open_list.pop_back(); // remove the last node
    
    return node_ptr;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
    path_found.push_back(*current_node); // add current_node (final node) to path_found vector
    RouteModel::Node *parent_node; // define parent_node pointer

    while (current_node!= this->start_node){

        parent_node = current_node->parent;
        // Calculate distance
        distance += current_node->distance(*parent_node);
        // Set parent_node as current_node
        current_node = parent_node;
        // Add "new" current_node to path_found nodes vector
        path_found.push_back(*current_node);

    }

    reverse(path_found.begin(),path_found.end()); // Change the order

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.

    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {

    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
    start_node->visited = true; // Set the start_node as visited

    open_list.emplace_back(start_node); // Add the start_node to open_list

    while (open_list.size() > 0){

        auto current_node = NextNode(); // return the node with the lowest f

        // Check if reach at "end_node": also can be "current_node->distance(*end_node) == 0"
        if (current_node == end_node){
            std::cout << "Path was found!" << "\n";
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
    return;
}