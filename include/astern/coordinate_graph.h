#ifndef COORDINATE_GRAPH_H_
#define COORDINATE_GRAPH_H_

#include "astern/unit.h"
#include <vector>
#include <utility>
#include <iostream>

static double haversine(double lat1, double lon1,
                        double lat2, double lon2);

class CoordinateGraph : public DistanceGraph {
 public:
  CoordinateGraph() = default;

  const NeighborT getNeighbors(VertexT v) const override;
  CostT estimatedCost(VertexT from, VertexT to) const override;
  CostT cost(VertexT from, VertexT to) const override;

  friend std::istream& operator>>(std::istream& is, CoordinateGraph& g);
  void setExampleID(int id);
  void computeScaleFactor();
  void computeHaversineScaleFactor();

 private:
  std::vector<NeighborT> adjacency_list;
  std::vector<std::pair<double, double>> coordinates; 
  int exampleID;
  size_t num_edges;
  double scale_factor = 1.0;
  double haversine_scale_factor = 1.0; 
};

std::istream& operator>>(std::istream& in, CoordinateGraph& g);

#endif  // COORDINATE_GRAPH_H_