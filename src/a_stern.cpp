
#define _USE_MATH_DEFINES
#include "astern/text_visualizer.h"
#include "astern/unit.h"
#include <algorithm>
#include <iostream>
#include <fstream>
#include <cmath>
#include <unordered_set>

// Maze Graph
class MazeGraph : public DistanceGraph {
  public:
    const NeighborT getNeighbors(VertexT v) const override {
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

    CostT estimatedCost(VertexT from, VertexT to) const override {
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

    CostT cost(VertexT from, VertexT to) const override {
      if (from >= height * width || to >= height * width) {
        throw std::out_of_range("Vertex index out of range");
      }
      for (const auto& neighbor : getNeighbors(from)) {
        if (neighbor.first == to) {
          return neighbor.second;  // Return the cost to the neighbor
        }
      }
      return infty;  // No edge exists
    }

    friend std::istream& operator>>(std::istream& is, MazeGraph& g) {
      is >> g.height >> g.width;
      g.cells.resize(g.height * g.width);
      for (unsigned int i = 0; i < g.height; ++i) {
        for (unsigned int j = 0; j < g.width; ++j) {
          char cell;
          is >> cell;
          if (cell == '#') {
            g.cells[i*g.width + j] = CellType::Wall;
          } else {
            g.cells[i*g.width + j] = CellType::Ground;
          }
        }
      }
      return is;
    }

    std::vector<CellType> getCells() const {
      return cells;
    }

    void setDimensions(unsigned int w, unsigned int h) {
      width = w;
      height = h;
      cells.resize(width * height);
      vertexCount = width * height;
    }

    void setCells(std::vector<CellType> new_cells) {
      if (new_cells.size() != width * height) {
        throw std::invalid_argument("Size of new_cells does not match dimensions");
      }
      cells = new_cells;
    }

  private:
    unsigned int width, height;
    std::vector<CellType> cells;
};

static double haversine(double lat1, double lon1,
                        double lat2, double lon2);

// Ein Graph, der Koordinaten von Knoten speichert.
class CoordinateGraph : public DistanceGraph {
 public:
  const NeighborT getNeighbors(VertexT v) const override {
    if (v >= vertexCount) {
      throw std::out_of_range("Vertex index out of range");
    }
    return adjacency_list[v];
  }

  CostT estimatedCost(VertexT from, VertexT to) const override {
    if (from >= vertexCount || to >= vertexCount) {
      throw std::out_of_range("Vertex index out of range");
    }
    auto [x1, y1] = coordinates[from];
    auto [x2, y2] = coordinates[to];
    
    switch (exampleID) {
      case 1: {
        // Euclidean distance
        return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
      }

      case 2: {
        // Manhattan distance until we get a better idea
        return std::abs(x2 - x1) + std::abs(y2 - y1);
      }

      case 3: {
        // Haversine formula for spherical distance
        return haversine(y1, x1, y2, x2);
      }

      case 4: {
        // Haversine formula / avg speed
        double distance = haversine(y1, x1, y2, x2);
        double time_sum = 0.0;
        double distance_sum = 0.0;
        // Find average speed
        for (VertexT v = 0; v < vertexCount; ++v) {
          for (const auto& neighbor : getNeighbors(v)) {
            time_sum += neighbor.second;
            distance_sum += haversine(
                coordinates[neighbor.first].second, coordinates[neighbor.first].first,
                coordinates[v].second, coordinates[v].first);
          }
        }
        double avg_speed = distance_sum / time_sum;
        return distance / avg_speed;
      }
        
      default: {
        throw std::runtime_error("Heuristic not implemented for this example");
      }
    }
  }

  CostT cost(VertexT from, VertexT to) const override {
    if (from >= vertexCount || to >= vertexCount) {
      throw std::out_of_range("Vertex index out of range");
    }
    for (const auto& neighbor : adjacency_list[from]) {
      if (neighbor.first == to) {
        return neighbor.second;
      }
    }
    return infty;  // No edge exists
  }

  friend std::istream& operator>>(std::istream& is, CoordinateGraph& g) {
    is >> g.vertexCount >> g.num_edges;
    g.adjacency_list.assign(g.vertexCount, {});
    g.coordinates.resize(g.vertexCount);

    for (size_t v = 0; v < g.num_edges; ++v) {
      VertexT from, to;
      CostT cost;
      is >> from >> to >> cost;
      g.adjacency_list[from].emplace_back(to, cost);
    }

    for (VertexT v = 0; v < g.vertexCount; ++v) {
      double x, y;
      is >> x >> y;
      g.coordinates[v] = {x, y};
    }

    return is;
  }

  void setExampleID(int id) {
    exampleID = id;
  }

private:
  std::vector<NeighborT> adjacency_list;
  std::vector<std::pair<double, double>> coordinates; 
  int exampleID;
  size_t num_edges;

};


// Taken from https://www.geeksforgeeks.org/haversine-formula-to-find-distance-between-two-points-on-a-sphere/
static double haversine(double lat1, double lon1,
                        double lat2, double lon2)
    {
        // distance between latitudes
        // and longitudes
        double dLat = (lat2 - lat1) *
                      M_PI / 180.0;
        double dLon = (lon2 - lon1) * 
                      M_PI / 180.0;

        // convert to radians
        lat1 = (lat1) * M_PI / 180.0;
        lat2 = (lat2) * M_PI / 180.0;

        // apply formulae
        double a = pow(std::sin(dLat / 2), 2) + 
                   pow(std::sin(dLon / 2), 2) * 
                   std::cos(lat1) * std::cos(lat2);
        double rad = 6371.0;
        double c = 2 * asin(std::sqrt(a));
        return rad * c;
    }

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
      while (current != infty && current != undefinedVertex && visited.find(current) == visited.end()) {
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

      if (possible_cost < gCost[neighbor_vertex]) {
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
    PruefeHeuristik(g);

    for (VertexT v = 0; v < g.numVertices(); ++v) {
      std::vector<CostT> kostenZumStart;
      Dijkstra(g, v, kostenZumStart);
      PruefeDijkstra(exampleID, v, kostenZumStart);
      for (VertexT ziel = 0; ziel < g.numVertices(); ++ziel) {
        if (v != ziel) {
          std::cout << "Trying A* from " << v << " to " << ziel << std::endl;
          std::list<VertexT> weg;
          if (A_star(g, v, ziel, weg)) {
            PruefeWeg(exampleID, weg);
          } else {
            std::cout << "No path found from " << v << " to " << ziel << std::endl;
          }
        }
      }
    }


    
  } else {
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
      mg.setCells(ErzeugeLabyrinth(width, height, seed));
      mg.setDimensions(width, height);


      VertexT start_vertex = undefinedVertex;
      VertexT destination_vertex = undefinedVertex;
      for (VertexT i = 0; i < mg.numVertices(); ++i) {
        if (mg.getCells()[i] == CellType::Start) start_vertex = i;
        if (mg.getCells()[i] == CellType::Destination) destination_vertex = i;
      }

      std::list<VertexT> weg;
      if (A_star(mg, start_vertex, destination_vertex, weg)) {
        PruefeWeg(10, weg);
      } else {
        std::cout << "No path found from " << start_vertex << " to " << destination_vertex << std::endl;
      }

    }

    std::ifstream inputFile("../data/daten/Maze" + std::to_string(exampleID) + ".dat");
    if (!inputFile) {
      std::cerr << "Error opening file for maze example " << exampleID << std::endl;
      return 1;
    }
    g.setExampleID(exampleID);
    inputFile >> mg;
    PruefeHeuristik(mg);

    for (const auto& pair : StartZielPaare(exampleID)) {
      VertexT start = pair.first;
      VertexT ziel = pair.second;
      std::list<VertexT> weg;
      if (A_star(mg, start, ziel, weg)) {
        PruefeWeg(exampleID + 4, weg);
      } else {
        std::cout << "No path found from " << start << " to " << ziel << std::endl;
      }
    }

    for (VertexT v = 0; v < mg.numVertices(); ++v) {
      std::vector<CostT> kostenZumStart;
      Dijkstra(mg, v, kostenZumStart);
      PruefeDijkstra(exampleID + 4, v, kostenZumStart);
    }

  }


  // Lade die zugehoerige Textdatei in einen Graphen
  // PruefeHeuristik
  // Loese die in der Aufgabenstellung beschriebenen Probleme fuer die jeweilige
  // Datei PruefeDijkstra / PruefeWeg

  std::cout << "\n \n All tests completed successfully!" << std::endl;
  std::cout << "Press Enter to close" << std::endl;
  std::cin.get();

  return 0;
}
