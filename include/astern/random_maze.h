#ifndef RANDOM_MAZE_H_
#define RANDOM_MAZE_H_

#include "astern/unit.h"
#include "astern/maze_graph.h"
#include <vector>


class RandomMaze : public MazeGraph {
    public:
        RandomMaze(int width, int height, const std::vector<CellType>& cells);

        VertexT getStart() const;
        VertexT getDestination() const;
    private:
        VertexT start;
        VertexT destination;
        int width, height;
        std::vector<CellType> cells;
};



#endif  // RANDOM_MAZE_H_