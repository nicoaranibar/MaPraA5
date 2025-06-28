
#include "astern/text_visualizer.h"
#include "astern/unit.h"

// Ein Graph, der Koordinaten von Knoten speichert.
class CoordinateGraph : public DistanceGraph {
 public:
  const NeighborT getNeighbors(VertexT v) const override;

  CostT estimatedCost(VertexT from, VertexT to) const override;

  CostT cost(VertexT from, VertexT to) const override;
};

void Dijkstra(const DistanceGraph& g, GraphVisualizer& v, VertexT start,
              std::vector<CostT>& kostenZumStart) {
  // ...
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
