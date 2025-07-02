
#include "astern/text_visualizer.h"
#include "astern/unit.h"
#include <algorithm>
#include <iostream>

// Ein Graph, der Koordinaten von Knoten speichert.
class CoordinateGraph : public DistanceGraph {
 public:
  const NeighborT getNeighbors(VertexT v) const override;

  CostT estimatedCost(VertexT from, VertexT to) const override;

  CostT cost(VertexT from, VertexT to) const override;

  friend std::istream& operator>>(std::istream& is, CoordinateGraph& g) {
    size_t num_edges;
    is >> g.vertexCount >> num_edges;
    g.adjacency_list.assign(g.vertexCount, {});
    g.coordinates.resize(g.vertexCount);

    for (size_t v = 0; v < num_edges; ++v) {
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

private:
  std::vector<NeighborT> adjacency_list;
  std::vector<std::pair<double, double>> coordinates; 

};

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
