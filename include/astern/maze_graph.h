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
  VertexT coordToVertex(int x, int y) const;
  // Getter methods
  int getWidth() const;
  int getHeight() const;
  const std::vector<CellType>& getCells() const;
  
  // Setter method for cells
  void setCells(const std::vector<CellType>& newCells);

 private:
  int width, height;
  std::vector<CellType> cells;
  bool isValidCell(int x, int y) const;
  std::pair<int, int> vertexToCoord(VertexT v) const;

};



#endif  // MAZE_GRAPH_H_