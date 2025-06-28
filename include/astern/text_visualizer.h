// Copyright (c) 2023, The MaPra Authors.

#ifndef TEXT_VISUALIZER_H_
#define TEXT_VISUALIZER_H_

#include <iostream>

#include "astern/unit.h"

class TextVisualizer : public GraphVisualizer {
 public:
  void markVertex(VertexT vertex, VertexStatus status) override {
    static const char* TEXT[] = {"Unbekannt", "In Queue", "Aktiv", "Fertig",
                                 "Ziel"};

    std::cerr << "VERTEX " << vertex
              << " - Neuer Status: " << TEXT[static_cast<int>(status)] << '\n';
  }

  void markEdge(EdgeT e, EdgeStatus status) override {
    static const char* TEXT[] = {" - Unbekannt", " - Besucht", " - Aktiv",
                                 " - Optimal"};

    std::cerr << "KANTE  (" << e.first << ", " << e.second << ")"
              << TEXT[static_cast<int>(status)] << '\n';
  }

  void updateVertex(VertexT vertex, double cost, double estimate,
                    VertexT parent, VertexStatus status) override {
    static const char* TEXT[] = {" - Unbekannt", " - In Queue", " - Aktiv",
                                 " - Fertig", " - Ziel"};

    std::cerr << "VERTEX " << vertex << TEXT[static_cast<int>(status)]
              << ", Vaterknoten ";

    if (parent != undefinedVertex) {
      std::cerr << parent;
    } else {
      std::cerr << '?';
    }

    std::cerr << ", Kosten " << (cost + estimate) << " = " << cost << "+"
              << estimate << '\n';
  }

  void draw() override {
    // Nothing to draw.
  }
};

#endif  // TEXT_VISUALIZER_H_
