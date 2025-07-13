#include "astern/maze_visualizer.h"
#include "astern/unit.h"
#include <SFML/Graphics.hpp>
#include <iostream>
#include <vector>

MazeVisualizer::MazeVisualizer(const MazeGraph& m, int WindowSize)
    : maze(m), cellSize(static_cast<float>(WindowSize) / std::max(m.getWidth(), m.getHeight())) {

    window.create(sf::VideoMode(WindowSize, WindowSize), "Maze Visualizer");
    if (!font.loadFromFile("../data/font/BebasNeue-Regular.ttf")) {
        std::cout << "Error loading font file." << std::endl;
        std::exit(1);
    }

    size_t n = maze.numVertices();
    vertexStatuses.assign(n, VertexStatus::UnknownVertex);
    gCosts.assign(n, infty);
    fCosts.assign(n, infty);
}

void MazeVisualizer::markVertex(VertexT vertex, VertexStatus status) {
    if (vertex >= vertexStatuses.size()) {
        throw std::out_of_range("Vertex index out of range");
    }
    vertexStatuses[vertex] = status;
}

void MazeVisualizer::updateVertex(VertexT vertex, double cost, double estimate,
                                    VertexT parent, VertexStatus status) {
    if (vertex >= gCosts.size()) {
        throw std::out_of_range("Vertex index out of range");
    }
    gCosts[vertex] = cost;
    fCosts[vertex] = cost + estimate;
    vertexStatuses[vertex] = status;
}


void MazeVisualizer::draw() {
    sf::Event event;
    while (window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.close();
        }
    }

    window.clear(sf::Color::Black);

    for (int y = 0; y < maze.getHeight(); ++y) {
        for (int x = 0; x < maze.getWidth(); ++x) {
            VertexT vertex = maze.coordToVertex(x, y);
            if (vertex >= vertexStatuses.size()) {
                throw std::out_of_range("Vertex index out of range");
            }

            sf::RectangleShape cell(sf::Vector2f(cellSize, cellSize));
            cell.setPosition(x * cellSize, y * cellSize);

            if (maze.getCells()[vertex] == CellType::Wall) {
                cell.setFillColor(sf::Color::Black);
            } else {
                switch (vertexStatuses[vertex]) {
                    case VertexStatus::UnknownVertex:
                        cell.setFillColor(sf::Color::White);
                        break;
                    case VertexStatus::InQueue:
                        cell.setFillColor(sf::Color::Blue); 
                        break;
                    case VertexStatus::Active:
                        cell.setFillColor(sf::Color::Yellow);
                        break;
                    case VertexStatus::Done:
                        cell.setFillColor(sf::Color::Green); 
                        break;
                    case VertexStatus::Destination:
                        cell.setFillColor(sf::Color::Red);
                        break;
                }
            }

            window.draw(cell);

            // Draw cost text
            if (show_text && gCosts[vertex] < infty) {
                sf::Text text;
                text.setFont(font);
                text.setCharacterSize(20);
                text.setFillColor(sf::Color::White);
                text.setPosition(x * cellSize + 5, y * cellSize + 5);
                text.setString("g: " + std::to_string(gCosts[vertex]) + "\nf: " + std::to_string(fCosts[vertex]));
                window.draw(text);
            }
        }
    }

    window.display();
}

void MazeVisualizer::markOptimalPath(const std::list<VertexT>& path) {
    if (path.empty()) return;

    for (auto it = path.begin(); it != path.end(); ++it) {
        if (*it < vertexStatuses.size()) {
            markVertex(*it, VertexStatus::Destination);
        }
    }
}


void MazeVisualizer::waitUntilClosed() {
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
    }
}


void MazeVisualizer::resetStatus() {
    std::fill(vertexStatuses.begin(), vertexStatuses.end(), VertexStatus::UnknownVertex);
    std::fill(gCosts.begin(), gCosts.end(), infty);
    std::fill(fCosts.begin(), fCosts.end(), infty);
}