# av-path-planner

A C++17 portfolio project implementing A* path planning on a 2D grid with multiple heuristics and terminal visualization.

## Why this is relevant to AV motion planning

Autonomous vehicles repeatedly solve shortest-path problems under constraints (free space vs. obstacles) and must balance **solution quality** with **search efficiency**; A* is a canonical building block that demonstrates this tradeoff via different admissible heuristics.

## Architecture

```
Scenario (grid + start + goal)
     |
     v
AStar solver (priority_queue min-heap, 8-directional, 3 heuristics)
     |
     v
AStarResult (path, nodes_explored, path_length)
     |
     v
Visualizer (ANSI terminal, comparison table)
```

## Build

```bash
cmake -B build && cmake --build build
```

## Run

```bash
./build/av-path-planner
```

## Test

```bash
./build/test_astar
```

## Sample output (excerpt)

```
Scenario: Open Field
Heuristic    | Nodes Explored | Path Length
-------------|----------------|------------
Manhattan    |             22 |       27.45
Euclidean    |             36 |       27.45
Diagonal     |            158 |       27.45
```

## Design decisions

1. **priority_queue min-heap vs. a sorted list**: a binary heap maintains fast \(O(\log n)\) push/pop for the open set, which is critical as A* scales with explored nodes.
2. **8-directional moves with diagonal cost 1.414**: approximates continuous motion better than 4-connected grids while still keeping costs simple and interpretable.
3. **Three heuristics and the tradeoff**: more accurate heuristics (e.g., Euclidean/Diagonal) reduce nodes explored but must remain cheap to compute; comparing them highlights the efficiency/optimality balance.

