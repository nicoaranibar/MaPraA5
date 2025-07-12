
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




bool A_star(const DistanceGraph& g, GraphVisualizer& v, VertexT start,
            VertexT ziel, std::list<VertexT>& weg) {
  const size_t N = g.numVertices();
  weg.clear();
  std::vector<CostT> gCost(N, infty);
  std::vector<CostT> fCost(N, infty);
  std::vector<VertexT> came_from(N, undefinedVertex);

  gCost[start] = 0;
  fCost[start] = g.estimatedCost(start, ziel);
  PriorityQueue open;
  open.push({fCost[start], start});
  v.markVertex(start, VertexStatus::InQueue);
  
  while (!open.empty()) {
    VertexT cur = open.pop();

    if (cur == ziel) {
      weg.clear();
      for (VertexT v = ziel; v != infty; v = came_from[v]) {
        weg.push_front(v);
      }
      v.markVertex(ziel, VertexStatus::Done);
      return true;
    }

    v.markVertex(cur, VertexStatus::Active);

    for (const auto& [neighbor, cost] : g.getNeighbors(cur)) {
      CostT tentative_gCost = gCost[cur] + cost;
      if (tentative_gCost < gCost[neighbor]) {
        came_from[neighbor] = cur;
        gCost[neighbor] = tentative_gCost;
        fCost[neighbor] = tentative_gCost + g.estimatedCost(neighbor, ziel);
        open.update({fCost[neighbor], neighbor});
        v.markEdge({cur, neighbor}, EdgeStatus::Active);
        v.updateVertex(neighbor, tentative_gCost, fCost[neighbor], cur, VertexStatus::InQueue);
      }
    }
    v.markVertex(cur, VertexStatus::Done);
  }
  
  return false;  // Kein Weg gefunden.
}

int main() {
  std::cout << "Starting tests... \n" << std::endl;




  std::cout << "\n \nAll tests completed successfully!" << std::endl;
  std::cout << "Press Enter to close" << std::endl;
  std::cin.ignore();
  std::cin.get();

  return 0;
}
