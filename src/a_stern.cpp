
#include "astern/text_visualizer.h"
#include "astern/unit.h"
#include <algorithm>

// Ein Graph, der Koordinaten von Knoten speichert.
class CoordinateGraph : public DistanceGraph {
 public:
  const NeighborT getNeighbors(VertexT v) const override;

  CostT estimatedCost(VertexT from, VertexT to) const override;

  CostT cost(VertexT from, VertexT to) const override;
};

void Dijkstra(const DistanceGraph& g, /* GraphVisualizer& v, */ VertexT start,
              std::vector<CostT>& kostenZumStart) {
  // Initialize V\S 
  std::vector<VertexT> not_visited(g.numVertices());
  for (VertexT i = 0; i < g.numVertices(); ++i) {
    not_visited[i] = i;
  }
  not_visited.erase(std::remove(not_visited.begin(), not_visited.end(), start), not_visited.end());

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
    not_visited.erase(std::remove(not_visited.begin(), not_visited.end(), min_index), not_visited.end());

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
