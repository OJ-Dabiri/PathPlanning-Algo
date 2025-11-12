# Multi-Robot Rendezvous Path Planning (A*)

## Overview

This project computes shortest **collision-free paths** for one or more robots on a 2D grid with obstacles, guiding each robot to a **common rendezvous point**.  
It uses an **A\*** (A-star) search algorithm with a **Manhattan heuristic** over a **4-connected grid** (up, down, left, right).  

Input files specify:
- Grid size
- Robot start positions
- The rendezvous coordinate
- A binary occupancy map (`0 = free`, `1 = blocked`)

The script parses the input, plans a path for each robot, and prints the resulting coordinate sequences.

---

## Key Features

- **Multiple robots:** Plans independently for each start position to a shared goal  
- **A\* with Manhattan heuristic:** Admissible for 4-direction movement on grids without diagonal moves  
- **Simple input format with inline comments:** Lines after `//` are ignored during parsing  
- **Obstacle-aware validity checks:** Nodes are traversable only if inside bounds and marked as `0`  

---

## Algorithm (A\*) — How It Works

**State space:**  
Grid cells `(row, col)` where a cell is valid if within bounds and `room[row][col] == 0`.

**Neighbors:**  
4-connected moves: `(-1,0)`, `(1,0)`, `(0,-1)`, `(0,1)`  

**Costs:**  
- Move cost = 1 per step  
- `g(n)` = steps from start  
- `h(n)` = Manhattan distance to rendezvous  
- `f(n) = g(n) + h(n)`

**Data structures:**
- **Open set:** Min-heap (`heapq`) keyed by `f(n)`
- **Visited:** Set of expanded cells
- **came_from:** Parent map for path reconstruction

**Search loop:**
1. Pop node with smallest `f`  
2. Skip if already visited  
3. Stop at goal  
4. Relax valid neighbors  

**Path reconstruction:**  
Trace `came_from` back from the rendezvous to the start, then reverse.

**Complexity:**  
- Time: `O(V log V)` over reachable cells  
- Space: `O(V)` for scores and bookkeeping  

---

## Input Format

Text files follow this structure:

<rows> <cols>
<num_robots>
<r1> <c1>
<r2> <c2>
...
<rN> <cN>
<goal_r> <goal_c>
<rows> lines of grid (each line is a string of 0/1 of length <cols>)



**Notes:**
- Comments after `//` are ignored  
- Coordinates are **(row, col)** using **0-based indexing**

**Example:**
8 10
2
1 2
2 8
7 4

Followed by 8 lines of 10 characters (each `0` or `1`).

**Provided examples:**  
`input1.txt`, `input2.txt`, `input3.txt`, `input4.txt`, plus large-scale maps like `input5.txt` (1000×1000) and `input6.txt` (100×100).

---

## Usage

### Install & Run
```bash
python PathPlanning.py input.txt
```
The script accepts an input_file argument via argparse.
However, the current main block overrides that with input4.txt.
Comment out or remove that hardcoded line to honor CLI input.

### Output
For each robot, you’ll see either:
- A list of grid coordinates from start to rendezvous, or
- No path found

## Design Choices & Trade-offs
- 4-connected grid: Keeps heuristic simple and admissible, but produces Manhattan-style paths (no diagonals)
- Independent planning per robot: Simpler but ignores inter-robot collisions or congestion
- Uniform step cost: Fast, but could be extended to weighted terrains

## Limitations
- No diagonal moves or variable step costs
- Static maps only (no dynamic obstacles)
- No multi-robot coordination or collision avoidance

## Repurposing Ideas
- Warehouse/Factory routing: Guide multiple AGVs to a common dock
- Search & Rescue: Direct responders to rally points on gridded building maps
- Game AI: NPC pathfinding toward shared objectives
- Traffic simulation: Model vehicle flow on discrete intersections
- Education: Teach heuristic search concepts and extend to advanced algorithms

## Extending This Code
- Honor CLI input by removing the hardcoded file override in __main__
- Add diagonals (8-connected) with octile/chebyshev heuristic
- Include richer outputs: path length, explored nodes, runtime
- Export to JSON/CSV for visualization
- Add unit tests for parser and path validity
