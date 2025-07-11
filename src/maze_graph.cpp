#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include "astern/unit.h"


class MazeGraph : public DistanceGraph {
  public:
    const NeighborT getNeighbors(VertexT v) const override {
      if (v >= height * width) {
        throw std::out_of_range("Vertex index out of range");
      }
      NeighborT neighbors;
      unsigned int row = v / width;
      unsigned int col = v % width;

      if (cells[v] == CellType::Wall) {
        return neighbors;  // No neighbors for walls
      }

      // Check up
      if (row > 0 && cells[(row - 1) * width + col] != CellType::Wall) {
        neighbors.emplace_back((row - 1) * width + col, 1);
      }
      // Check down
      if (row < height - 1 && cells[(row + 1) * width + col] != CellType::Wall) {
        neighbors.emplace_back((row + 1) * width + col, 1);
      }
      // Check left
      if (col > 0 && cells[row * width + (col - 1)] != CellType::Wall) {
        neighbors.emplace_back(row * width + (col - 1), 1);
      }
      // Check right
      if (col < width - 1 && cells[row * width + (col + 1)] != CellType::Wall) {
        neighbors.emplace_back(row * width + (col + 1), 1);
      }

      return neighbors;
    }

    CostT estimatedCost(VertexT from, VertexT to) const override {
      if (from >= height * width || to >= height * width) {
        throw std::out_of_range("Vertex index out of range");
      }
      int from_row = from / width;
      int from_col = from % width;
      int to_row = to / width;
      int to_col = to % width;

      // Using Manhattan distance as heuristic
      return std::abs(from_row - to_row) + std::abs(from_col - to_col);
    }

    CostT cost(VertexT from, VertexT to) const override {
      if (from >= height * width || to >= height * width) {
        throw std::out_of_range("Vertex index out of range");
      }
      for (const auto& neighbor : getNeighbors(from)) {
        if (neighbor.first == to) {
          return neighbor.second;  // Return the cost to the neighbor
        }
      }
      return infty;  // No edge exists
    }

    friend std::istream& operator>>(std::istream& is, MazeGraph& g) {
      unsigned int height, width;
      is >> height >> width;
      g.setDimensions(width, height);
      for (unsigned int i = 0; i < g.height; ++i) {
        for (unsigned int j = 0; j < g.width; ++j) {
          char cell;
          is >> cell;
          if (cell == '#') {
            g.cells[i*g.width + j] = CellType::Wall;
          } else {
            g.cells[i*g.width + j] = CellType::Ground;
          }
        }
      }
      return is;
    }

    std::vector<CellType> getCells() const {
      return cells;
    }

    void setDimensions(unsigned int w, unsigned int h) {
      width = w;
      height = h;
      cells.resize(width * height);
      vertexCount = width * height;
    }

    void setCells(std::vector<CellType> new_cells) {
      if (new_cells.size() != width * height) {
        throw std::invalid_argument("Size of new_cells does not match dimensions");
      }
      cells = new_cells;
    }

  private:
    unsigned int width, height;
    std::vector<CellType> cells;
};