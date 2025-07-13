#include "astern/coords_visualizer.h"
#include <stdexcept>
#include <iostream>

CoordsVisualizer::CoordsVisualizer(const CoordinateGraph& g, float w, float h)
    : graph(g), width(w), height(h) {

    window.create(sf::VideoMode(static_cast<unsigned int>(w), static_cast<unsigned int>(h)), "Coordinates Visualizer");
    if (!font.loadFromFile("../data/font/BebasNeue-Regular.ttf")) {
        std::cout << "Error loading font file." << std::endl;
        std::exit(1);
    }

    size_t n = g.numVertices();
    gCosts.assign(n, infty);
    fCosts.assign(n, infty);
    vertexStatuses.assign(n, VertexStatus::UnknownVertex);

    edgeStatuses.clear();
    for (VertexT u = 0; u < n; ++u) {
        for (const auto& [v, cost] : g.getNeighbors(u)) {
            if (v < n) {
                edgeStatuses[{u, v}] = EdgeStatus::UnknownEdge;
            } else {
                std::cerr << "Arista apunta a vértice inválido: " << u << " -> " << v << std::endl;
            }
        }
    }


    const auto& coords = g.getCoordinates();
    if (coords.empty()) {
        throw std::runtime_error("No coordinates available in the graph.");
    }
    auto [minX, maxX] = std::minmax_element(coords.begin(), coords.end(),
                                            [](auto a, auto b) { return a.first < b.first; });
    auto [minY, maxY] = std::minmax_element(coords.begin(), coords.end(),
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

            sf::Color edgeColor =  sf::Color::White; // Default edge color

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
        if (show_text && gCosts[v] < infty) {
            sf::Text text;
            text.setFont(font);
            text.setCharacterSize(20);
            text.setFillColor(sf::Color::White);
            text.setPosition(x + radius + 5, y - radius - 5);

            std::string displayText = "g: " + std::to_string(gCosts[v]) + "\n";
            displayText += "f: " + std::to_string(fCosts[v]);

            text.setString(displayText);
            window.draw(text);
        }

        if (show_text && gCosts[v] == infty) {
            sf::Text text;
            text.setFont(font);
            text.setCharacterSize(20);
            text.setFillColor(sf::Color::Red);
            text.setPosition(x + radius + 5, y - radius - 5);
            text.setString("?");
            window.draw(text);
        }
    }

    window.display();
    sf::sleep(sf::milliseconds(50)); // slow down the drawing to make it visible
}


void CoordsVisualizer::markOptimalPath(const std::list<VertexT>& path) {
    markVertex(path.back(), VertexStatus::Destination);
    for (auto it = path.begin(); it != path.end(); ++it) {
        if (*it < vertexStatuses.size()) {
            EdgeT edge;
            if (std::next(it) != path.end()) {
                edge = {*it, *std::next(it)};
            } else if (it != path.begin()) {
                edge = {*std::prev(it), *it};
            } else {
                continue; // Skip if it's the only vertex in the path
            }
            auto edgeIt = edgeStatuses.find(edge);
            if (edgeIt != edgeStatuses.end()) {
                edgeIt->second = EdgeStatus::Optimal;
            } else {
                std::cerr << "Edge not found in edgeStatuses: " << edge.first << " -> " << edge.second << std::endl;
            }
        }
    }
}


std::pair<float, float> CoordsVisualizer::toScreenCoords(std::pair<double, double> logicalCoords) const {
    float x = margin + (logicalCoords.first) * scaleX;
    float y = margin + (logicalCoords.second) * scaleY;
    return {x, y};
}


void CoordsVisualizer::waitUntilClosed() {
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
    }
}

void CoordsVisualizer::resetStatus() {
    std::fill(vertexStatuses.begin(), vertexStatuses.end(), VertexStatus::UnknownVertex);
    std::fill(gCosts.begin(), gCosts.end(), infty);
    std::fill(fCosts.begin(), fCosts.end(), infty);
    for (auto& [edge, status] : edgeStatuses) {
        status = EdgeStatus::UnknownEdge;
    }
}