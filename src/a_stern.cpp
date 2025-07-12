
#define _USE_MATH_DEFINES
#include "astern/text_visualizer.h"
#include "astern/unit.h"
#include "astern/coordinate_graph.h"
#include "astern/maze_graph.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <unordered_set>


void Dijkstra(const DistanceGraph& g, /* GraphVisualizer& v, */ VertexT start,
              std::vector<CostT>& kostenZumStart) {
  // Initialize V\S 
  std::vector<VertexT> not_visited(g.numVertices());
  for (VertexT i = 0; i < g.numVertices(); ++i) {
    not_visited[i] = i;
  }
  not_visited.erase(std::find(not_visited.begin(), not_visited.end(), start));

  // Initialize costs
  kostenZumStart.assign(g.numVertices(), infty);
  kostenZumStart[start] = 0;
  for (const auto& neighbor : g.getNeighbors(start)) {
    kostenZumStart[neighbor.first] = neighbor.second;
  }

  // Actual algorithm
  while (!not_visited.empty()) {
    VertexT min_vertex = infty;
    CostT min_cost = infty;
    for (const auto& v : not_visited) {
      if (kostenZumStart[v] < min_cost) {
        min_cost = kostenZumStart[v];
        min_vertex = v;
      }
    }
    if (min_vertex == infty) {
      break;  // No more reachable vertices
    }
    not_visited.erase(std::find(not_visited.begin(), not_visited.end(), min_vertex));

    for (const auto& neighbor : g.getNeighbors(min_vertex)) {
      kostenZumStart[neighbor.first] =
          std::min(kostenZumStart[neighbor.first], kostenZumStart[min_vertex] + neighbor.second);
    }
  }
}



class PriorityQueue {
 public:
  using PQItem = std::pair<CostT, VertexT>;

 private:
  std::vector<PQItem> heap;

 public:
  void push(const PQItem& item) {
    heap.push_back(item);
    std::push_heap(heap.begin(), heap.end(), std::greater<PQItem>());
  }

  VertexT pop() {
    std::pop_heap(heap.begin(), heap.end(), std::greater<PQItem>());
    PQItem min = heap.back();
    heap.pop_back();
    return min.second;
  }

  void update(const PQItem& item) {
    for (auto& existing_item : heap) {
      if (existing_item.second == item.second) {
        if (item.first < existing_item.first) {
          existing_item = item;
          std::make_heap(heap.begin(), heap.end(), std::greater<PQItem>());
        }
        return;
      }
      push(item);
      return;
    }
  }

  bool empty() const { return heap.empty(); }

};




bool A_star(const DistanceGraph& g, /* GraphVisualizer& v, */ VertexT start,
            VertexT ziel, std::list<VertexT>& weg) {
  const size_t N = g.numVertices();
  weg.clear();
  std::vector<CostT> gCost(N, infty);
  std::vector<CostT> fCost(N, infty);
  std::vector<VertexT> came_from(N, infty);

  gCost[start] = 0;
  fCost[start] = g.estimatedCost(start, ziel);

  using PQItem = std::pair<CostT, VertexT>;
  std::vector<PQItem> open;
  open.emplace_back(fCost[start], start);
  std::push_heap(open.begin(), open.end(), std::greater<PQItem>());

  std::vector<bool> in_open(N, false);
  in_open[start] = true;

  while(!open.empty()) {
    std::pop_heap(open.begin(), open.end(), std::greater<PQItem>());
    auto [current_fCost, current] = open.back();
    open.pop_back();
    in_open[current] = false;

    if (current == ziel) {
      // Ziel reached, reconstruct the path
      std::unordered_set<VertexT> visited;
      while (current != infty && current != undefinedVertex && visited.count(current) == 0) {
        weg.push_front(current);
        visited.insert(current);
        current = came_from[current];
      }
      return true;  // Path found
    }

    for (const auto& neighbor : g.getNeighbors(current)) {
      VertexT neighbor_vertex = neighbor.first;
      CostT edge_cost = neighbor.second;
      CostT possible_cost = gCost[current] + edge_cost;

      if (possible_cost < gCost[neighbor_vertex] && g.cost(current, neighbor_vertex) != infty) {
        came_from[neighbor_vertex] = current;
        gCost[neighbor_vertex] = possible_cost;
        fCost[neighbor_vertex] = possible_cost + g.estimatedCost(neighbor_vertex, ziel);

        if (!in_open[neighbor_vertex]) {
          open.emplace_back(fCost[neighbor_vertex], neighbor_vertex);
          std::push_heap(open.begin(), open.end(), std::greater<PQItem>());
          in_open[neighbor_vertex] = true;
        }
      }
    }
  }

  return false;  // Kein Weg gefunden.
}

int main() {
  // Frage Beispielnummer vom User ab
  int exampleID;
  int graph_or_maze;
  std::cout << "Type 1 for Graph, 2 for Maze: ";
  std::cin >> graph_or_maze;
  if (graph_or_maze != 1 && graph_or_maze != 2) {
    std::cerr << "Invalid input. Please enter 1 for Graph or 2 for Maze." << std::endl;
    return 1;
  }
  CoordinateGraph g;
  MazeGraph mg;
  if (graph_or_maze == 1) {
    std::cout << "Enter graph example number (1-4): ";
    std::cin >> exampleID;
    if (exampleID < 1 || exampleID > 4) {
      std::cerr << "Invalid example number. Please enter a number between 1 and 4." << std::endl;
      return 1;
    }
    std::ifstream inputFile("../data/daten/Graph" + std::to_string(exampleID) + ".dat");
    if (!inputFile) {
      std::cerr << "Error opening file for graph example " << exampleID << std::endl;
      return 1;
    }
    g.setExampleID(exampleID);
    inputFile >> g;
    g.computeScaleFactor();
    g.computeHaversineScaleFactor();
    PruefeHeuristik(g);

    for (VertexT v = 0; v < g.numVertices(); ++v) {
      std::vector<CostT> kostenZumStart;
      Dijkstra(g, v, kostenZumStart);
      PruefeDijkstra(exampleID, v, kostenZumStart);
      for (VertexT ziel = 0; ziel < g.numVertices(); ++ziel) {
        if (v != ziel) {
          //std::cout << "Trying A* from " << v << " to " << ziel << std::endl;
          std::list<VertexT> weg;
          if (A_star(g, v, ziel, weg)) {
            PruefeWeg(exampleID, weg);
          } /*else {
            std::cout << "No path found from " << v << " to " << ziel << std::endl;
          } */
        }
      }
    }


    
  } else if (graph_or_maze == 2) {
    std::cout << "Enter maze example number (1-5) or 0 for random maze: ";
    std::cin >> exampleID;
    if (exampleID < 0 || exampleID > 5) {
      std::cerr << "Invalid example number. Please enter a number between 1 and 4." << std::endl;
      return 1;
    }

    // FOR RANDOM MAZE GENERATION
    if (exampleID == 0) {
      unsigned int seed, width, height;
      std::cout << "Enter width and height for the random maze: " << std::endl;
      std::cin >> width >> height;
      std::cout << "Enter seed for random maze generation: " << std::endl;
      std::cin >> seed;
      if (width <= 0 || height <= 0 || seed <= 0) {
        std::cerr << "Width and height and seed must be positive integers." << std::endl;
        return 1;
      }
      mg = MazeGraph();
      mg.setDimensions(width, height);
      mg.setCells(ErzeugeLabyrinth(width, height, seed));
      


      VertexT start_vertex = undefinedVertex;
      VertexT destination_vertex = undefinedVertex;
      for (VertexT i = 0; i < mg.numVertices(); ++i) {
        if (mg.getCells()[i] == CellType::Start) start_vertex = i;
        if (mg.getCells()[i] == CellType::Destination) destination_vertex = i;
      }

      if (start_vertex == undefinedVertex || destination_vertex == undefinedVertex) {
      std::cerr << "Random maze has no start or destination cell." << std::endl;
      return 1;
      }

      std::list<VertexT> weg;
      if (A_star(mg, start_vertex, destination_vertex, weg)) {
        PruefeWeg(10, weg);
      } else {
        std::cout << "No path found from " << start_vertex << " to " << destination_vertex << std::endl;
      }

    } else {
      std::ifstream inputFile("../data/daten/Maze" + std::to_string(exampleID) + ".dat");
      if (!inputFile) {
        std::cerr << "Error opening file for maze example " << exampleID << std::endl;
        return 1;
      }
      inputFile >> mg;
      PruefeHeuristik(mg);

      for (const auto& pair : StartZielPaare(exampleID+4)) {
        VertexT start = pair.first;
        VertexT ziel = pair.second;
        std::list<VertexT> weg;
        if (A_star(mg, start, ziel, weg)) {
          PruefeWeg(exampleID + 4, weg);
        } else {
          std::cout << "No path found from " << start << " to " << ziel << std::endl;
        }
      }
    }

  }


  // Lade die zugehoerige Textdatei in einen Graphen
  // PruefeHeuristik
  // Loese die in der Aufgabenstellung beschriebenen Probleme fuer die jeweilige
  // Datei PruefeDijkstra / PruefeWeg

  std::cout << "\n \nAll tests completed successfully!" << std::endl;
  std::cout << "Press Enter to close" << std::endl;
  std::cin.ignore();
  std::cin.get();

  return 0;
}
