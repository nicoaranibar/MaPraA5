#include <vector>
#include <cmath>
#include <stdexcept>
#include <iostream>
#include "astern/unit.h"
#include "astern/maze_graph.h"

const DistanceGraph::NeighborT MazeGraph::getNeighbors(VertexT v) const {
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

CostT MazeGraph::estimatedCost(VertexT from, VertexT to) const {
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

CostT MazeGraph::cost(VertexT from, VertexT to) const {
    if (from >= height * width || to >= height * width) {
        throw std::out_of_range("Vertex index out of range");
    }
    if (cells[from] == CellType::Wall || cells[to] == CellType::Wall) {
        return infty;  // No cost if either cell is a wall
    }
    return 1;  // Assuming uniform cost for moving between adjacent cells
}

std::istream& operator>>(std::istream& is, MazeGraph& g) {
    unsigned int height, width;
    is >> height >> width;
    g.setDimensions(width, height);
    g.cells.resize(width * height);
    for (unsigned int i = 0; i < height; ++i) {
        for (unsigned int j = 0; j < width; ++j) {
            char cell;
            is >> cell;
            if (cell == '#') {
                g.cells[i * width + j] = CellType::Wall;
            } else if (cell == 'S') {
                g.cells[i * width + j] = CellType::Start;
            } else if (cell == 'D') {
                g.cells[i * width + j] = CellType::Destination;
            } else {
                g.cells[i * width + j] = CellType::Ground;
            }
        }
    }
    return is;
}

std::vector<CellType> MazeGraph::getCells() const {
    return cells;
}

void MazeGraph::setDimensions(int w, int h) {
    width = w;
    height = h;
    cells.resize(width * height);
    vertexCount = width * height;
}

void MazeGraph::setCells(std::vector<CellType> new_cells) {
    if (new_cells.size() != width * height) {
        throw std::invalid_argument("Size of new_cells does not match dimensions");
    }
    cells = new_cells;
}

