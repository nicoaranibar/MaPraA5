#ifndef MAZE_VISUALIZER_H_
#define MAZE_VISUALIZER_H_

#include "astern/unit.h"
#include "astern/maze_graph.h"
#include <SFML/Graphics.hpp>
#include <vector>

class MazeVisualizer : public GraphVisualizer {
 private:
  sf::RenderWindow window;
  const MazeGraph& maze;
  sf::Font font;

  std::vector<VertexStatus> vertexStatuses;
  std::vector<CostT> gCosts;
  std::vector<CostT> fCosts;
  float cellSize;
  bool show_text = true;

 public:
  MazeVisualizer(const MazeGraph& m, int WindowSize = 800);
  ~MazeVisualizer() = default;

  void markVertex(VertexT vertex, VertexStatus status) override;
  void markEdge(EdgeT e, EdgeStatus status) override {}
  void updateVertex(VertexT vertex, double cost, double estimate,
                    VertexT parent, VertexStatus status) override;
  void draw() override;

  void markOptimalPath(const std::list<VertexT>& path);
  void resetStatus();
  void turnOffText() {
    show_text = false;
  }

  void waitUntilClosed();
};

#endif // MAZE_VISUALIZER_H_