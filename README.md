# Mini-Courier Planner
**Course:** Design & Analysis of Algorithms (ETCCDA302) – Capstone Assignment
**Student:** _<AANYA MANGAL / 2501730123>_  **Faculty:** Dr. Shweta Agarwal

Given a list of parcels and a road network, the program (1) prioritises the parcels, (2) chooses the best set for one van trip, (3) finds shortest road distances, and (4) builds an ordered delivery route.

---

## 1. Phase 1 – Problem design and planning

**Problem.** A courier van leaves a depot (node 0) with a weight capacity. There are more parcels than the van can carry. Choose which parcels to load, then visit their customers and return to the depot, driving as little as possible.

**Data design**

| Item | Format | Fields |
|---|---|---|
| Parcel (`parcels_*.csv`) | one row per parcel | `id, weight (kg), priority (1-10), deadline (hours), dest (graph node)` |
| Road network (`graph_*.csv`) | undirected weighted edge list, stored as an adjacency list `{node: [(neighbour, km)]}` | `u, v, distance` |
| Knapsack value | `score = priority + 3 × max(0, 12 − deadline)` | priority plus an urgency bonus |

**Why a score?** With raw priority alone, a low-priority parcel due in 2 hours would be left behind. The urgency bonus makes close deadlines matter in the packing decision.

**Algorithm selection**

| Sub-problem | Algorithm | Why this one | Complexity |
|---|---|---|---|
| Prioritise parcels | **Merge Sort** | Guaranteed O(n log n), stable (equal keys keep their order) | O(n log n) time, O(n) space |
| Choose parcels for one trip | **0/1 Knapsack, Dynamic Programming** | Each parcel is taken whole or not at all; greedy can be wrong; DP is exact | O(n·W) time and space |
| Shortest road distance | **Dijkstra with a min-heap** | All road lengths are positive | O((V+E) log V) |
| Order of visiting customers | **Nearest-Neighbour TSP approximation** | Exact TSP is NP-hard (O(k!) or O(k²2ᵏ)); NN is fast and gives a decent tour | O(k²) after k+1 Dijkstra runs |

**How the parts work together**

```
parcels.csv ──► Merge Sort ──► Knapsack DP ──► chosen parcels ──► their destination nodes
                (deadline,     (max score,                              │
                 priority)      weight ≤ W)                             ▼
graph.csv ─────────────────────────────────────► Dijkstra from depot + each stop (distances, paths)
                                                                        │
                                                                        ▼
                                                  Nearest-Neighbour tour: depot → … → depot
```

---

## 2. Phase 2 – Prioritisation and box filling (small dataset, capacity 20 kg)

**Sorted order** (earliest deadline first, ties → higher priority):
`P8, P2, P4, P1, P10, P6, P3, P9, P5, P7`

**Knapsack result:** selected `P8, P1, P10, P3` – total weight **20 kg**, total score **128**.
Left for the next trip: `P2, P4, P6, P9, P5, P7`.

The complete DP table is printed by the program and saved in `outputs/sample_run_output.txt`.

**How the DP works**
`dp[i][w]` = best score using only the first `i` parcels with capacity `w`.
`dp[i][w] = max( dp[i-1][w],  dp[i-1][w - weight_i] + score_i )` (skip it / take it).
The chosen parcels are recovered by walking back from `dp[n][W]`: if `dp[i][w] ≠ dp[i-1][w]`, parcel `i` was taken.

---

## 3. Phase 3 – Route maker

* Graph: 10 nodes (0 = depot), 15 undirected roads (`data/graph_small.csv`).
* Stops needed: destinations of the chosen parcels → nodes `{3, 6, 7}`.
* **Route:** `0 → 3 → 6 → 7 → 0`
* **Real road path** (through intermediate junctions): `0 → 3 → 6 → 7 → 4 → 3 → 0`
* **Total distance: 30 km**

Nearest-Neighbour: from the current stop, always go to the closest unvisited stop (distance from Dijkstra), then return to the depot at the end.

---

## 4. Phase 4 – Performance testing

### Correctness evidence (`benchmark.py`)
| Check | Result |
|---|---|
| Merge sort equals Python `sorted()` | PASS |
| Knapsack DP equals brute-force optimum over all 2ⁿ subsets (30 random cases) | PASS |
| Dijkstra equals Floyd–Warshall on 10 random graphs | PASS |
| NN tour vs true optimal tour (20 cases, 6 stops) | average **1.07×** optimal, worst 1.20× |

### Runtime results (ms, best of 3)
| Sorting n | Merge sort | Insertion sort | Built-in |
|---|---|---|---|
| 1000 | 2.15 | 40.67 | 0.25 |
| 4000 | 9.95 | 584.02 | 1.19 |

Quadrupling n made merge sort ≈4.6× slower (n log n) but insertion sort ≈14× slower (n²).

| Knapsack n × W | Time | | Dijkstra V | Time | | NN-TSP stops k | Time |
|---|---|---|---|---|---|---|---|
| 100 × 200 | 1.6 | | 1000 | 1.1 | | 10 | 6.3 |
| 400 × 800 | 35.6 | | 4000 | 7.7 | | 80 | 55.8 |

Knapsack time grows with the product n×W (4× n and 4× W → 16× cells → ≈22× time). Dijkstra is almost linear in V. NN-TSP grows linearly in k here because the k+1 Dijkstra runs dominate the O(k²) selection.

### Small vs large pipeline
| | Small (10 parcels, 10 nodes) | Large (1000 parcels, 500 nodes) |
|---|---|---|
| Sort | 0.02 ms | 3.6 ms |
| Knapsack (capacity 20 / 150 kg) | 0.04 ms | 19.8 ms |
| Route (Dijkstra + NN) | 0.07 ms | 67.3 ms |
| Result | 4 parcels, 30 km | 82 parcels, 75 stops, 1084 km |

Charts: `outputs/chart1_sorting.png`, `chart2_scaling.png`, `chart3_stage_comparison.png`.

### Observations and future optimisation
1. **Routing is the slowest step** on large inputs because it runs one Dijkstra per stop. Improvement: stop each Dijkstra early once all stops are reached, or cache distances between trips.
2. **Knapsack memory is O(n·W).** The table is only needed for back-tracking; a 1-D array gives O(W) memory if only the best value is required.
3. **Sorting:** Python's built-in Timsort is ~8× faster than the hand-written merge sort (C implementation). Use it in production; keep merge sort for demonstrating the algorithm.
4. **Better tours:** apply 2-opt improvement after Nearest-Neighbour, or use Christofides (1.5× guarantee).
5. **Multiple trips:** repeat Steps 2–3 on the left-over parcels until all are delivered.
6. **Time windows / one-way roads:** extend the graph to be directed and add arrival-time checks against deadlines.

### Limitations
Nearest-Neighbour has no constant-factor guarantee (can be far from optimal on adversarial graphs). Parcel weights must be integers for the DP table. Deadlines are not checked against route travel time.

---

## 5. How to run

```bash
pip install matplotlib          # only needed for charts
python courier_planner.py       # full demo on the small dataset (prints sorted list, DP table, route)
python benchmark.py             # correctness tests, runtime tables, creates the charts + large dataset
```

Files
```
courier_planner.py   all algorithms + integrated planner
benchmark.py         tests, timing, charts
data/                parcels_small.csv, graph_small.csv, parcels_large.csv, graph_large.csv
outputs/             sample_run_output.txt, benchmark_results.txt, chart1-3 PNG
```


## 6. Viva preparation – likely questions
* **Why merge sort?** Stable and always O(n log n); quicksort's worst case is O(n²).
* **Why can't greedy solve 0/1 knapsack?** Taking the best score-per-kg first can leave unused capacity that a different combination would fill better; DP checks all combinations implicitly.
* **Time and space of the DP?** O(n·W) both; this is *pseudo-polynomial* because it depends on the value of W, not its bit-length.
* **Why can't Dijkstra handle negative edges?** It assumes a finalised node can never be improved; a negative edge can break that. Use Bellman–Ford.
* **Is Nearest-Neighbour optimal?** No. It is a heuristic; it was 1.07× optimal on average in our tests.
* **Why `if d > dist[u]: continue`?** The heap may hold outdated entries for a node already settled with a shorter distance.
* **Trace the DP:** pick any cell, e.g. `dp[P1][20]`, and show the skip/take comparison.
