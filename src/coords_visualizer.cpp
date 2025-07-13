#include "astern/coords_visualizer.h"
#include <stdexcept>
#include <iostream>

CoordsVisualizer::CoordsVisualizer(const CoordinateGraph& g, float w, float h)
    : graph(g), width(w), height(h) {

    window.create(sf::VideoMode(static_cast<unsigned int>(w), static_cast<unsigned int>(h)), "Coordinates Visualizer");
   
    gCosts.resize(g.numVertices(), infty);
    fCosts.resize(g.numVertices(), infty);
    from.resize(g.numVertices(), undefinedVertex);

    vertexStatuses.resize(g.numVertices(), VertexStatus::UnknownVertex);
    edgeStatuses.clear();
    for (VertexT u = 0; u < g.numVertices(); ++u) {
        for (const auto& [v, cost] : g.getNeighbors(u)) {
            edgeStatuses[{u, v}] = EdgeStatus::UnknownEdge;
        }
    }

    if (!font.loadFromFile("../data/font/BebasNeue-Regular.ttf")) {
        std::cout << "Error loading font file." << std::endl;
    }

    auto [minX, maxX] = std::minmax_element(g.getCoordinates().begin(), g.getCoordinates().end(),
                                            [](auto a, auto b) { return a.first < b.first; });
    auto [minY, maxY] = std::minmax_element(g.getCoordinates().begin(), g.getCoordinates().end(),
                                            [](auto a, auto b) { return a.second < b.second; });

    double dx = maxX->first - minX->first;
    double dy = maxY->second - minY->second;
    scaleX = (w - 2 * margin) / (dx == 0 ? 1.0 : dx);
    scaleY = (h - 2 * margin) / (dy == 0 ? 1.0 : dy);
}

void CoordsVisualizer::markVertex(VertexT vertex, VertexStatus status) {
    if (vertex >= vertexStatuses.size()) {
        throw std::out_of_range("Vertex index out of range");
    }
    vertexStatuses[vertex] = status;
}

void CoordsVisualizer::markEdge(EdgeT e, EdgeStatus status) {
    edgeStatuses[e] = status;
}

void CoordsVisualizer::updateVertex(VertexT vertex, double cost, double estimate,
                                    VertexT parent, VertexStatus status) {
    if (vertex >= gCosts.size()) {
        throw std::out_of_range("Vertex index out of range");
    }
    gCosts[vertex] = cost;
    fCosts[vertex] = cost + estimate;
    from[vertex] = parent;
    vertexStatuses[vertex] = status;
}

void CoordsVisualizer::draw() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
    }

    window.clear(sf::Color::Black);
    const float radius = 5.0f;

    for (VertexT v = 0; v < graph.numVertices(); ++v) {
        for (const auto& [u, cost] : graph.getNeighbors(v)) {
            if (u >= graph.numVertices()) {
                throw std::out_of_range("Neighbor vertex index out of range");
            }
            auto [x1, y1] = toScreenCoords(graph.getCoordinates()[v]);
            auto [x2, y2] = toScreenCoords(graph.getCoordinates()[u]);
            auto edge = std::make_pair(v, u);

            sf::Color edgeColor =  sf::Color(80, 80, 80); // Default edge color

            auto it = edgeStatuses.find(edge);
            if (it != edgeStatuses.end()) {
                switch (it->second) {
                    case EdgeStatus::Visited:
                        edgeColor = sf::Color::Green;
                        break;
                    case EdgeStatus::Active:
                        edgeColor = sf::Color::Yellow;
                        break;
                    case EdgeStatus::Optimal:
                        edgeColor = sf::Color::Red;
                        break;
                    default:
                        break;
                }
            }

            sf::Vertex line[] = {
                sf::Vertex(sf::Vector2f(x1, y1), edgeColor),
                sf::Vertex(sf::Vector2f(x2, y2), edgeColor)
            };
            window.draw(line, 2, sf::Lines);

        }
    }


    for (VertexT v = 0; v < graph.numVertices(); ++v) {
        auto [x, y] = toScreenCoords(graph.getCoordinates()[v]);
        sf::CircleShape circle(radius);
        circle.setPosition(x - radius, y - radius);

        sf::Color vertexColor = sf::Color::White; // Default vertex color

        if (v < vertexStatuses.size()) {
            switch (vertexStatuses[v]) {
                case VertexStatus::InQueue:
                    vertexColor = sf::Color::Blue;
                    break;
                case VertexStatus::Active:
                    vertexColor = sf::Color::Yellow;
                    break;
                case VertexStatus::Done:
                    vertexColor = sf::Color::Green;
                    break;
                case VertexStatus::Destination:
                    vertexColor = sf::Color::Red;
                    break;
                default:
                    break;
            }
        }

        circle.setFillColor(vertexColor);
        window.draw(circle);

        // Draw cost text
        if (gCosts[v] < infty) {
            sf::Text costText(std::to_string(static_cast<int>(gCosts[v])), font, 12);
            costText.setFillColor(sf::Color::White);
            costText.setPosition(x + radius, y - radius);
            window.draw(costText);
        }
    }

    window.display();
    sf::sleep(sf::milliseconds(50)); // slow down the drawing to make it visible
}


void CoordsVisualizer::markOptimalPath(const std::list<VertexT>& path) {
    for (auto it = path.begin(); it != path.end(); ++it) {
        if (*it < vertexStatuses.size()) {
            markVertex(*it, VertexStatus::Destination);
        }
    }
}


std::pair<float, float> CoordsVisualizer::toScreenCoords(std::pair<double, double> logicalCoords) const {
    float x = margin + (logicalCoords.first - graph.getCoordinates()[0].first) * scaleX;
    float y = margin + (logicalCoords.second - graph.getCoordinates()[0].second) * scaleY;
    return {x, y};
}