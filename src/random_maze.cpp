#include "astern/unit.h"
#include "astern/random_maze.h"
#include <stdexcept>


RandomMaze::RandomMaze(int w, int h, const std::vector<CellType>& c) {
    if (c.size() != w * h) {
        throw std::invalid_argument("Size of cells does not match dimensions");
    }

    setDimensions(w, h);
    setCells(c);
    
    // Find start and destination cells
    const auto& cells = getCells();
    for (VertexT i = 0; i < cells.size(); ++i) {
        if (cells[i] == CellType::Start) {
            start = i;
        } else if (cells[i] == CellType::Destination) {
            destination = i;
        }
    }
}

std::ostream& operator<<(std::ostream& os, const RandomMaze& maze) {
    os << "RandomMaze: " << maze.getWidth() << "x" << maze.getHeight() << "\n";
    const auto& cells = maze.getCells();
    for (int y = 0; y < maze.getHeight(); ++y) {
        for (int x = 0; x < maze.getWidth(); ++x) {
            switch (cells[y * maze.getWidth() + x]) {
                case CellType::Wall:
                    os << "# ";
                    break;
                case CellType::Ground:
                    os << ". ";
                    break;
                case CellType::Start:
                    os << "S ";
                    break;
                case CellType::Destination:
                    os << "D ";
                    break;
            }
        }
        os << "\n";
    }
    return os;
}



VertexT RandomMaze::getStart() const {
    return start;
}

VertexT RandomMaze::getDestination() const {
    return destination;
}
