
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


void Dijkstra(const DistanceGraph& g, /* GraphVisualizer& v,*/ VertexT start,
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
    // Check if item already exists in the queue
    for (auto& existing_item : heap) {
      if (existing_item.second == item.second) {
        if (item.first < existing_item.first) {
          existing_item = item;
          std::make_heap(heap.begin(), heap.end(), std::greater<PQItem>());
        }
        return;
      }
    }
    // Item not found, add it to the queue
    push(item);
  }

  bool empty() const { return heap.empty(); }

};




bool A_star(const DistanceGraph& g, /*GraphVisualizer& v,*/ VertexT start,
            VertexT ziel, std::list<VertexT>& weg) {
  const size_t N = g.numVertices();
  weg.clear();
  std::vector<CostT> gCost(N, infty);
  std::vector<CostT> fCost(N, infty);
  std::vector<VertexT> came_from(N, undefinedVertex);
  std::vector<bool> closed(N, false);  // Track processed vertices

  gCost[start] = 0;
  fCost[start] = g.estimatedCost(start, ziel);
  PriorityQueue open;
  open.push({fCost[start], start});
  //v.markVertex(start, VertexStatus::InQueue);
  
  while (!open.empty()) {
    VertexT cur = open.pop();

    // Skip if already processed
    if (closed[cur]) {
      continue;
    }
    closed[cur] = true;

    if (cur == ziel) {
      weg.clear();
      for (VertexT v = ziel; v != undefinedVertex; v = came_from[v]) {
        weg.push_front(v);
      }
      //v.markVertex(ziel, VertexStatus::Done);
      return true;
    }

    //v.markVertex(cur, VertexStatus::Active);

    for (const auto& [neighbor, cost] : g.getNeighbors(cur)) {
      if (closed[neighbor]) {
        continue;  // Skip already processed vertices
      }
      
      CostT tentative_gCost = gCost[cur] + cost;
      if (tentative_gCost < gCost[neighbor]) {
        came_from[neighbor] = cur;
        gCost[neighbor] = tentative_gCost;
        fCost[neighbor] = tentative_gCost + g.estimatedCost(neighbor, ziel);
        open.update({fCost[neighbor], neighbor});
        //v.markEdge({cur, neighbor}, EdgeStatus::Active);
        //v.updateVertex(neighbor, tentative_gCost, fCost[neighbor], cur, VertexStatus::InQueue);
      }
    }
    //v.markVertex(cur, VertexStatus::Done);
  }
  
  return false;  // Kein Weg gefunden.
}



int main() {
  std::cout << "Starting tests... \n" << std::endl;

  int exampleID;
  std::cout << "Enter example ID. 1-4 for CoordinateGraph, 5-9 for MazeGraph, 10 for RandomMaze: \n";
  std::cin >> exampleID;
  if (exampleID < 1 || exampleID > 10) {
    std::cerr << "Invalid example ID. Please enter a number between 1 and 10." << std::endl;
    return 1;
  }

  if (exampleID >= 1 && exampleID <= 4) {
    // ----- Beispiele 1–4: CoordinateGraph + Dijkstra
    std::string filename = "daten/Graph" + std::to_string(exampleID) + ".dat";
    std::ifstream file(filename);
    if (!file) {
      std::cerr << "Could not open file: " << filename << std::endl;
      return 1;
    }

    CoordinateGraph g;
    file >> g;
    g.setExampleID(exampleID);
    g.computeScaleFactor();
    g.computeHaversineScaleFactor();
    PruefeHeuristik(g);

    for (VertexT start = 0; start < g.numVertices(); ++start) {
      std::cout << "\nNow checking Dijkstra for start vertex " << start << "...";
      std::vector<CostT> D;
      Dijkstra(g, start, D);
      PruefeDijkstra(exampleID, start, D);

      std::cout << "Now checking A* for start vertex " << start << "...\n";
      for (VertexT ziel = 0; ziel < g.numVertices(); ++ziel) {
        if (ziel == start) continue;  // Skip if start and goal are the same
        std::list<VertexT> weg;
        if (A_star(g, start, ziel, weg)) {
          PruefeWeg(exampleID, weg);
        } else {
          std::cout << "A* did not find a path from " << start << " to " << ziel << ".\n";
        }
      }
      
    }

    

  } else if (exampleID >= 5 && exampleID <= 9) {
    // ----- Beispiele 5–9: MazeGraph + Dijkstra + A*
    std::string filename = "daten/Maze" + std::to_string(exampleID-4) + ".dat";
    std::ifstream file(filename);
    if (!file) {
      std::cerr << "Could not open file: " << filename << std::endl;
      return 1;
    }

    MazeGraph g;
    file >> g;
    PruefeHeuristik(g);

    for (const auto& [start, ziel] : StartZielPaare(exampleID)) {
      std::list<VertexT> weg;
      if (A_star(g, start, ziel, weg)) {
        PruefeWeg(exampleID, weg);
      } else {
        std::cout << "A* did not find a path from " << start << " to " << ziel << ".\n";
      }
    }

  } 


  std::cout << "\n \nAll tests completed successfully!" << std::endl;
  std::cout << "Press Enter to close" << std::endl;
  std::cin.ignore();
  std::cin.get();

  return 0;
}
