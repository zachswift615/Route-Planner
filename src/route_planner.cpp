#include "route_planner.h"
#include "route_model.h"
#include <algorithm>


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find the closest nodes to the starting and ending coordinates.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);

}


// H Value is the distance from the current node to the end node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*end_node);
}


// Add all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor: current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.push_back(neighbor);
        
    }
}


bool Compare(const RouteModel::Node *a, const RouteModel::Node *b) {
  return (a->g_value + a->h_value) > (b->g_value + b->h_value);
}


// Find the best next node based distance from current to next node and distance to end node
RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *lowest_sum_node = open_list.back();
    open_list.pop_back();
    return lowest_sum_node;
}


// Return the final path found from the A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    while (current_node->parent != nullptr) {
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }
    path_found.push_back(*current_node);
    std::reverse(path_found.begin(), path_found.end());
    
    // Multiply the distance by the scale of the map to get meters.
    distance *= m_Model.MetricScale(); 
    return path_found;

}



void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node; 
    start_node->visited = true;
    open_list.push_back(start_node);

    while (open_list.size() > 0) {
        current_node = NextNode();
        if (current_node->h_value == 0) {
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
        AddNeighbors(current_node);
    }
}