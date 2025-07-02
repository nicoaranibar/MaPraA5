
#include "astern/text_visualizer.h"
#include "astern/unit.h"
#include <algorithm>
#include <iostream>
#include <cmath>

// Maze Graph
class MazeGraph : public DistanceGraph {
  public:
    const NeighborT getNeighbors(VertexT v) const override;

    CostT estimatedCost(VertexT from, VertexT to) const override;

    CostT cost(VertexT from, VertexT to) const override;
};

// Ein Graph, der Koordinaten von Knoten speichert.
class CoordinateGraph : public DistanceGraph {
 public:
  const NeighborT getNeighbors(VertexT v) const override {
    if (v >= vertexCount) {
      throw std::out_of_range("Vertex index out of range");
    }
    return adjacency_list[v];
  }

  CostT estimatedCost(VertexT from, VertexT to) const override {
    if (from >= vertexCount || to >= vertexCount) {
      throw std::out_of_range("Vertex index out of range");
    }
    auto [x1, y1] = coordinates[from];
    auto [x2, y2] = coordinates[to];
    
    switch (exampleID) {
      case 1:
        // Euclidean distance
        return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

      case 2:
        // Manhattan distance until we get a better idea
        return std::abs(x2 - x1) + std::abs(y2 - y1);

      case 3:
        // Haversine formula for spherical distance
        return haversine(y1, x1, y2, x2);

      case 4:
        // Haversine formula / avg speed
        double distance = haversine(y1, x1, y2, x2);
        double time_sum = 0.0;
        double distance_sum = 0.0;
        // Find average speed
        for (VertexT v = 0; v < vertexCount; ++v) {
          for (const auto& neighbor : getNeighbors(v)) {
            time_sum += neighbor.second;
            distance_sum += haversine(
                coordinates[neighbor.first].second, coordinates[neighbor.first].first,
                coordinates[v].second, coordinates[v].first);
          }
        }
        double avg_speed = distance_sum / time_sum;
        return distance / avg_speed;
        
      default:
        throw std::runtime_error("Heuristic not implemented for this example");
    }
  }

  CostT cost(VertexT from, VertexT to) const override {
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

  friend std::istream& operator>>(std::istream& is, CoordinateGraph& g) {
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

  void setExampleID(int id) {
    exampleID = id;
  }

private:
  std::vector<NeighborT> adjacency_list;
  std::vector<std::pair<double, double>> coordinates; 
  int exampleID;
  size_t num_edges;

};


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

void Dijkstra(const DistanceGraph& g, /* GraphVisualizer& v, */ VertexT start,
              std::vector<CostT>& kostenZumStart) {
  // Initialize V\S 
  std::vector<VertexT> not_visited(g.numVertices());
  for (VertexT i = 0; i < g.numVertices(); ++i) {
    not_visited[i] = i;
  }
  not_visited.erase(std::find(not_visited.begin(), not_visited.end(), start));

  // Initialize costs
  kostenZumStart.assign(g.numVertices(), infty);
  kostenZumStart[start] = 0;
  for (const auto& neighbor : g.getNeighbors(start)) {
    kostenZumStart[neighbor.first] = neighbor.second;
  }

  // Actual algorithm
  while (!not_visited.empty()) {
    VertexT min_index = infty;
    CostT min_cost = infty;
    for (const auto& v : not_visited) {
      if (kostenZumStart[v] < min_cost) {
        min_cost = kostenZumStart[v];
        min_index = v;
      }
    }
    if (min_index == infty) {
      break;  // No more reachable vertices
    }
    not_visited.erase(std::find(not_visited.begin(), not_visited.end(), min_index));

    for (const auto& neighbor : g.getNeighbors(min_index)) {
      kostenZumStart[neighbor.first] =
          std::min(kostenZumStart[neighbor.first], kostenZumStart[min_index] + neighbor.second);
    }
  }
}

bool A_star(const DistanceGraph& g, GraphVisualizer& v, VertexT start,
            VertexT ziel, std::list<VertexT>& weg) {
  // ...
  return false;  // Kein Weg gefunden.
}

int main() {
  // Frage Beispielnummer vom User ab

  // Lade die zugehoerige Textdatei in einen Graphen
  // PruefeHeuristik

  // Loese die in der Aufgabenstellung beschriebenen Probleme fuer die jeweilige
  // Datei PruefeDijkstra / PruefeWeg

  return 0;
}
