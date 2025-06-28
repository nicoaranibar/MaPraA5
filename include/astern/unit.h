// Copyright (c) 2023, The MaPra Authors.

#ifndef UNIT_H_
#define UNIT_H_

#include <functional>
#include <list>
#include <utility>
#include <vector>

using VertexT = std::size_t;
using EdgeT = std::pair<VertexT, VertexT>;
using CostT = double;

extern const CostT infty;
extern const VertexT undefinedVertex;

//-------------------------------------------------------------------------------------------------
// Abstrakte Basisklasse zur Repraesentation eines Distanzgraphen
//-------------------------------------------------------------------------------------------------

class DistanceGraph {
 public:
  using LocalEdgeT = std::pair<VertexT, CostT>;
  using NeighborT = std::vector<LocalEdgeT>;

 protected:
  // Die Anzahl der Knoten im Graphen.
  std::size_t vertexCount;

 public:
  DistanceGraph(int num_verts = 0) : vertexCount(num_verts) {}

  virtual ~DistanceGraph() {}

  // Gibt die Anzahl der Knoten im Graphen zurueck.
  std::size_t numVertices() const { return vertexCount; }

  // Gibt die Nachbarknoten eines gegebenen Knotens v zurueck.
  virtual const NeighborT getNeighbors(VertexT v) const = 0;

  // Heuristik: Schaetzt die Kosten, um vom Knoten "from" zum Knoten "to" zu
  // gelangen.
  virtual CostT estimatedCost(VertexT from, VertexT to) const = 0;

  // Gibt die wahren Kosten c an, um vom Knoten "from" zum Knoten "to" zu
  // gelangen. Existiert zwischen den beiden Knoten keine Kante, werden die
  // Kosten mit infty bewertet.
  virtual CostT cost(VertexT from, VertexT to) const = 0;
};

//-------------------------------------------------------------------------------------------------
// Prueft den gegebenen Distanzvektor D auf Korrektheit. Der i-te Eintrag von D
// enthaelt die Kosten, um vom Startvertex v0 zum Vertex i zu gelangen. Bsp ist
// die jeweilig untersuchte Beispielnummer.
//-------------------------------------------------------------------------------------------------
void PruefeDijkstra(int Bsp, VertexT v0, const std::vector<CostT>& D);

//-------------------------------------------------------------------------------------------------
// Prueft einen gegebenen Weg zwischen zwei Knoten fuer ein Beispiel auf
// Optimalitaet. Bsp ist die jeweilig untersuchte Beispielnummer.
//-------------------------------------------------------------------------------------------------
void PruefeWeg(int Bsp, const std::list<VertexT>& weg);

//-------------------------------------------------------------------------------------------------
// Diese Funktion prueft einen gegebenen Distanzgraphen auf Zulaessigkeit, d.h.
// ob die Kostenschaetzung dieses Graphen die tatsaechlichen Kosten nie
// ueberschaetzt.
//-------------------------------------------------------------------------------------------------
bool PruefeHeuristik(const DistanceGraph& g);

//-------------------------------------------------------------------------------------------------
// Diese Enumeration beschreibt die moeglichen Zellentypen im Labyrinth.
//-------------------------------------------------------------------------------------------------
enum class CellType { Wall, Ground, Start, Destination };

//-------------------------------------------------------------------------------------------------
// Diese Methode gibt ein linearisiertes Labyrinth der Groesse breite x hoehe
// zurueck. Im zurueckgegebenen std::vector stehen die ersten <breite> Eintraege
// fuer die erste Zeile des Labyrinths, die naechsten <breite> Eintraege fuer
// die zweite Zeile usw. Es gibt jeweils genau eine Zelle mit dem Zelltyp Start
// und mit Destination, die den Start- bzw Zielknoten des zu findenden Weges
// markieren. Der Parameter seed wird benutzt, um den Zufallsgenerator zur
// Labyrinth-Erzeugung zu initialisieren - bei fester groesse erzeugt der
// gleiche seed das gleiche Labyrinth.
//-------------------------------------------------------------------------------------------------
std::vector<CellType> ErzeugeLabyrinth(int breite, int hoehe,
                                       unsigned int seed);

//-------------------------------------------------------------------------------------------------
// Diese Methode gibt fuer die Beispiele 5-9 einen std::vector von
// Start-End-Paaren zurueck, die der A*-Algorithmus loesen soll. Beispiel: for (
// auto pair : StartZielPaare(9)) {
//   auto start = pair.first;
//   auto goal  = pair.second;
//   (Berechne den kuerzesten Weg von start zu goal)
// }
// Sollte Ihnen dieser Code kryptisch vorkommen, informieren Sie sich (online
// oder bei den HiWis) ueber den "range-based for loop" in C++11.
//-------------------------------------------------------------------------------------------------
std::vector<std::pair<VertexT, VertexT>> StartZielPaare(int Bsp);

//-------------------------------------------------------------------------------------------------
// Eine Liste der moeglichen Zustaende, die ein Knoten haben kann.
//-------------------------------------------------------------------------------------------------
enum class VertexStatus { UnknownVertex, InQueue, Active, Done, Destination };

//-------------------------------------------------------------------------------------------------
// Eine Liste der moeglichen Zustaende, die eine Kante haben kann.
//-------------------------------------------------------------------------------------------------
enum class EdgeStatus { UnknownEdge, Visited, Active, Optimal };

//-------------------------------------------------------------------------------------------------
// Eine abstrakes Interface, das Methoden fuer eine Visualisierung von
// Graphalgorithmen beinhaltet.
//-------------------------------------------------------------------------------------------------
class GraphVisualizer {
 public:
  virtual ~GraphVisualizer() {}

  // Zeige an, dass sich ein Knoten jetzt in dem angegebenen Zustand befindet.
  virtual void markVertex(VertexT vertex, VertexStatus status) = 0;

  // Zeige an, dass sich eine Kante im angegebenen Zustand befindet.
  virtual void markEdge(EdgeT e, EdgeStatus status) = 0;

  // Aktualisiere jegliche Daten eines Knotens.
  virtual void updateVertex(VertexT vertex, double cost, double estimate,
                            VertexT parent, VertexStatus status) = 0;

  // Zeichne den aktuellen Zustand des Graphen.
  virtual void draw() = 0;
};

#endif  // UNIT_H_
