#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include "astern/unit.h"
#include "astern/coordinate_graph.h"


// Taken from https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
static double haversine(double lat1, double lon1,
                        double lat2, double lon2)
    {
        // distance between latitudes
        // and longitudes
        double dLat = (lat2 - lat1) *
                      M_PI / 180.0;
        double dLon = (lon2 - lon1) * 
                      M_PI / 180.0;

        // convert to radians
        lat1 = (lat1) * M_PI / 180.0;
        lat2 = (lat2) * M_PI / 180.0;

        // apply formulae
        double a = pow(std::sin(dLat / 2), 2) + 
                   pow(std::sin(dLon / 2), 2) * 
                   std::cos(lat1) * std::cos(lat2);
        double rad = 6371.0;
        double c = 2 * asin(std::sqrt(a));
        return rad * c;
    }


const DistanceGraph::NeighborT CoordinateGraph::getNeighbors(VertexT v) const {
  if (v >= vertexCount) {
    throw std::out_of_range("Vertex index out of range");
  }
  return adjacency_list[v];
}

CostT CoordinateGraph::cost(VertexT from, VertexT to) const {
  if (from >= vertexCount || to >= vertexCount) {
    throw std::out_of_range("Vertex index out of range");
  }
  for (const auto& neighbor : adjacency_list[from]) {
    if (neighbor.first == to) {
      return neighbor.second;
    }
  }
  return infty;  // No edge exists
}

CostT CoordinateGraph::estimatedCost(VertexT from, VertexT to) const {
  if (from >= vertexCount || to >= vertexCount) {
    throw std::out_of_range("Vertex index out of range");
  }
  auto [x1, y1] = coordinates[from];
  auto [x2, y2] = coordinates[to];

  switch (exampleID) {
    case 1: {
      // Euclidean distance
      return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    }

    case 2: {
      // Euclidean distance with scale factor
      double euclidean = std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
      return scale_factor * euclidean;
    }

    case 3: {
      // Haversine formula for spherical distance
      return haversine_scale_factor * haversine(y1, x1, y2, x2);
    }

    case 4: {
      // Haversine formula / avg speed
      double distance = haversine(y1, x1, y2, x2);
      double avg_speed = 75.0; // Assuming an average speed of 75 km/h
      return distance / avg_speed;
    }
      
    default: {
      throw std::runtime_error("Heuristic not implemented for this example");
    }
  }
}

std::istream& operator>>(std::istream& is, CoordinateGraph& g) {
  is >> g.vertexCount >> g.num_edges;
  g.adjacency_list.assign(g.vertexCount, {});
  g.coordinates.resize(g.vertexCount);

  for (size_t v = 0; v < g.num_edges; ++v) {
    VertexT from, to;
    CostT cost;
    is >> from >> to >> cost;
    g.adjacency_list[from].emplace_back(to, cost);
  }

  for (VertexT v = 0; v < g.vertexCount; ++v) {
    double x, y;
    is >> x >> y;
    g.coordinates[v] = {x, y};
  }

  return is;
}

void CoordinateGraph::setExampleID(int id) {
  exampleID = id;
}

void CoordinateGraph::computeScaleFactor() {
  double max_ratio = 1.0;
  for (VertexT from = 0; from < vertexCount; ++from) {
    auto [x1, y1] = coordinates[from];
    for (const auto& [to, cost] : adjacency_list[from]) {
      auto [x2, y2] = coordinates[to];
      double euclidean = std::sqrt((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1));
      if (euclidean > 0.0) {
        max_ratio = std::min(max_ratio, cost / euclidean);
      }
    }
  }
  scale_factor = max_ratio;
}

void CoordinateGraph::computeHaversineScaleFactor() {
  double max_ratio = 1.0;
  for (VertexT from = 0; from < vertexCount; ++from) {
    auto [x1, y1] = coordinates[from];
    for (const auto& [to, cost] : adjacency_list[from]) {
      auto [x2, y2] = coordinates[to];
      double haversine_distance = haversine(y1, x1, y2, x2);
      if (haversine_distance > 0.0) {
        max_ratio = std::min(max_ratio, cost / haversine_distance);
      }
    }
  }
  haversine_scale_factor = max_ratio;
}
