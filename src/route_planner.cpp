#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y,
                           float end_x, float end_y)
    : m_Model(model) {
  // Convert inputs to percentage:
  start_x *= 0.01;
  start_y *= 0.01;
  end_x *= 0.01;
  end_y *= 0.01;

  start_node = &m_Model.FindClosestNode(start_x, start_y);
  end_node = &m_Model.FindClosestNode(end_x, end_y);
}

/*
  Calculate the Euclidean distance from the given node to the end_node.
*/
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
  return node->distance(*end_node);
}

/*
  Expand the current node by adding all unvisited neighbors to the open list.
*/
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  // Populate current_node.neighbors vector with all the neighbors
  current_node->FindNeighbors();
  // Set parameters for each neighbor
  for (auto node : current_node->neighbors) {
    node->parent = current_node;
    node->h_value = CalculateHValue(node);
    node->g_value =
        current_node->g_value + node->distance(*current_node); // increment
    open_list.emplace_back(node); // Add to open_list
    node->visited = true;
  }
}

/*
   Sort the open list and return the next node.
*/
RouteModel::Node *RoutePlanner::NextNode() {
  // Sort in descending order according to the sum of the h value and g value.
  std::sort(std::begin(open_list), std::end(open_list), [](auto lhs, auto rhs) {
    return (lhs->h_value + lhs->g_value) > (rhs->h_value + rhs->g_value);
  });
  auto node = open_list.back();
  open_list.pop_back(); // discard used
  return node;
}

/*
  Return the final path found from your A* search.
*/
std::vector<RouteModel::Node>
RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  // Create path_found vector
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;

  while (current_node->parent) {
    distance += current_node->distance(*current_node->parent);
    path_found.emplace_back(*current_node);
    current_node = current_node->parent; // rewind
  }
  path_found.emplace_back(*current_node); // start node

  std::reverse(std::begin(path_found), std::end(path_found));

  // Multiply the distance by the scale of the map to get meters.
  distance *= m_Model.MetricScale();

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

}
