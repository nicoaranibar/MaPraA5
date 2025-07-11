#ifndef MAZE_GRAPH_H_
#define MAZE_GRAPH_H_

#include "astern/unit.h"
#include <vector>
#include <string>

class MazeGraph : public DistanceGraph {
 public:
  MazeGraph() = default;

  const NeighborT getNeighbors(VertexT v) const override;
  CostT estimatedCost(VertexT from, VertexT to) const override;
  CostT cost(VertexT from, VertexT to) const override;

  friend std::istream& operator>>(std::istream& is, MazeGraph& g);
  void setDimensions(int width, int height);
  void setCells(std::vector<CellType> new_cells);
  std::vector<CellType> getCells() const;


 private:
  unsigned int width, height;
  std::vector<CellType> cells;
};



#endif  // MAZE_GRAPH_H_