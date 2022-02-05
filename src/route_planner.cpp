#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model)
{
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node)
{
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node)
{
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors)
    {
        neighbor->visited = true;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->parent = current_node;
        open_list.push_back(neighbor);
    }
}
bool Compare(RouteModel::Node *a, RouteModel::Node *b)
{
    return a->weight() > b->weight();
}

RouteModel::Node *RoutePlanner::NextNode()
{
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node *current = open_list.back();
    open_list.pop_back();

    return current;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
{
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    RouteModel::Node *node = current_node;

    while (node)
    {
        path_found.push_back(*node);

        if (node == start_node)
            break;

        distance += node->distance(*(node->parent));
        node = node->parent;
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale();
    return path_found;
}

void RoutePlanner::AStarSearch()
{
    RouteModel::Node *current_node = nullptr;
    start_node->visited = true;
    open_list.push_back(start_node);

    while (open_list.size() > 0)
    {
        RouteModel::Node *current_node = this->NextNode();
        if (current_node == end_node)
        {
            m_Model.path = this->ConstructFinalPath(current_node);
            return;
        }

        AddNeighbors(current_node);
    }

    std::cout << "No path found" << std::endl;
}