#include "astern/unit.h"
#include "astern/random_maze.h"
#include <stdexcept>


RandomMaze::RandomMaze(int w, int h, const std::vector<CellType>& c) {
    if (c.size() != w * h) {
        throw std::invalid_argument("Size of cells does not match dimensions");
    }

    setDimensions(width, height);
    cells = c;
    
    // Find start and destination cells
    for (VertexT i = 0; i < cells.size(); ++i) {
        if (cells[i] == CellType::Start) {
            start = i;
        } else if (cells[i] == CellType::Destination) {
            destination = i;
        }
    }

}

VertexT RandomMaze::getStart() const {
    return start;
}

VertexT RandomMaze::getDestination() const {
    return destination;
}
