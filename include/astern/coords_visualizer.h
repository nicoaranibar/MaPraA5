#ifndef COORDS_VISUALIZER_H_
#define COORDS_VISUALIZER_H_
#include "astern/unit.h"
#include "astern/coordinate_graph.h"
#include <SFML/Graphics.hpp>
#include <map>
#include <cmath>

class CoordsVisualizer : public GraphVisualizer {
    private:
        sf::RenderWindow window;
        const CoordinateGraph& graph;
        sf::Font font;

        std::vector<VertexStatus> vertexStatuses;
        std::vector<CostT> gCosts;
        std::vector<CostT> fCosts;
        std::vector<VertexT> from;

        std::map<EdgeT, EdgeStatus> edgeStatuses;

        float width, height;
        float scaleX, scaleY;
        float margin = 50.0f;
    public:
        CoordsVisualizer(const CoordinateGraph& g, float w = 1300, float h = 800);
        ~CoordsVisualizer() = default;

        void markVertex(VertexT vertex, VertexStatus status) override;
        void markEdge(EdgeT e, EdgeStatus status) override;
        void updateVertex(VertexT vertex, double cost, double estimate,
                          VertexT parent, VertexStatus status) override;
        void draw() override;

        void markOptimalPath(const std::list<VertexT>& path);

    private:
        std::pair<float, float> toScreenCoords(std::pair<double, double> logicalCoords) const;
};



#endif // COORDS_VISUALIZER_H_

