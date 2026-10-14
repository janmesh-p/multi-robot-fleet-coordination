#pragma once

#include <utility>
#include <vector>

#include "fleet_core/common/config.hpp"
#include "fleet_core/common/types.hpp"
#include "fleet_core/world/grid_map.hpp"

namespace fleet_core {

// Parametric warehouse:
//   - parking rows along the top and bottom edges ('R')
//   - a station lane down each side: pickups on the left ('P'), dropoffs on the right ('D')
//   - a main aisle inside each lane, aisle rows along the top and bottom
//   - blocks of shelves ('#') separated by one-cell aisles
//
// Every edge between free cells lies on a cycle: no bridges. Paths still never pass through a
// station or parking cell, so for planning each parking cell is a dead end, and so is each corner
// cell whose other neighbors are a station and a parking cell. PIBT's reachability guarantee
// needs a graph without dead ends, so the pibt planner adds a rule for robots leaving one.
// Throws ConfigError for an invalid layout.
GridMap generateWarehouse(const MapGeneratorConfig& layout);

// Edges between static free cells whose removal disconnects the free-cell graph.
std::vector<std::pair<Cell, Cell>> findBridges(const GridMap& map);

// True when every static free cell can reach every other.
bool isConnected(const GridMap& map);

}  // namespace fleet_core
