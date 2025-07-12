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
    auto [row, col] = vertexToCoord(v);

    static const int dx[] = {0,  1,  0, -1};
    static const int dy[] = {-1, 0,  1,  0};

    for (int d = 0; d < 4; ++d) {
        int nx = row + dx[d];
        int ny = col + dy[d];

        if (isValidCell(nx, ny)) {
            VertexT nv = coordToVertex(nx, ny);
            if (cells[nv] != CellType::Wall) {
                neighbors.emplace_back(nv, 1.0);
            }
        }
    }

    return neighbors;
}



CostT MazeGraph::estimatedCost(VertexT from, VertexT to) const {
    if (from >= height * width || to >= height * width) {
        throw std::out_of_range("Vertex index out of range");
    }
    auto [from_row, from_col] = vertexToCoord(from);
    auto [to_row, to_col] = vertexToCoord(to);


    // Using Manhattan distance as heuristic
    return std::abs(from_row - to_row) + std::abs(from_col - to_col);
}



CostT MazeGraph::cost(VertexT from, VertexT to) const {
    if (from >= height * width || to >= height * width) {
        throw std::out_of_range("Vertex index out of range");
    }
    
    for (const auto& neighbor : getNeighbors(from)) {
        if (neighbor.first == to) {
            return neighbor.second; 
        }
    }
}



std::istream& operator>>(std::istream& is, MazeGraph& g) {
    int height, width;
    is >> width >> height;
    g.setDimensions(width, height);

    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            char cell;
            is >> cell;
            if (cell == '#') {
                g.cells[i * width + j] = CellType::Wall;
            } else {
                g.cells[i * width + j] = CellType::Ground;
            }
        }
    }
    return is;
}



void MazeGraph::setDimensions(int w, int h) {
    width = w;
    height = h;
    cells.resize(width * height);
    vertexCount = width * height;
}




bool MazeGraph::isValidCell(int x, int y) const {
    return x < width && y < height && x >= 0 && y >= 0;
}



VertexT MazeGraph::coordToVertex(int x, int y) const {
    if (x >= width || y >= height || x < 0 || y < 0) {
        throw std::out_of_range("Coordinates out of range");
    }
    return y * width + x;
}



std::pair<int, int> MazeGraph::vertexToCoord(VertexT v) const {
    if (v >= vertexCount) {
        throw std::out_of_range("Vertex index out of range");
    }
    return {v % width, v / width};
}