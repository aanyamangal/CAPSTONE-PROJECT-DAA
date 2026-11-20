"""
Mini-Courier Planner  --  Design & Analysis of Algorithms (ETCCDA302) Capstone
=============================================================================
Pipeline:
    Phase 2a  Merge Sort           -> prioritise parcels (earliest deadline, then highest priority)
    Phase 2b  0/1 Knapsack (DP)    -> choose the best set of parcels for ONE trip (weight capacity)
    Phase 3a  Dijkstra             -> shortest road distance between any two places
    Phase 3b  Nearest-Neighbour    -> approximate TSP tour over the delivery points
    Phase 4   Benchmarking         -> see benchmark.py

Run:  python courier_planner.py
"""

import csv
import heapq
import math
import os

INF = math.inf
DEPOT = 0                       # node 0 is always the courier depot / warehouse
URGENCY_CAP = 12                # deadlines >= 12h get no urgency bonus
URGENCY_WEIGHT = 3              # how strongly a close deadline raises the score


# ----------------------------------------------------------------------------
# 1. DATA DESIGN
# ----------------------------------------------------------------------------
class Parcel:
    """A parcel to deliver.

    id        : unique parcel id, e.g. 'P1'
    weight    : weight in kg  (knapsack 'cost')
    priority  : 1-10, higher = more important
    score     : priority + urgency bonus  (knapsack 'value')
    deadline  : delivery deadline in hours from now (smaller = more urgent)
    dest      : graph node (customer location) the parcel must reach
    """
    def __init__(self, pid, weight, priority, deadline, dest):
        self.id = pid
        self.weight = int(weight)
        self.priority = int(priority)
        self.deadline = int(deadline)
        self.dest = int(dest)
        # knapsack "value": priority plus a bonus that grows as the deadline gets closer,
        # so urgent parcels are not left behind just because their raw priority is low
        self.score = self.priority + URGENCY_WEIGHT * max(0, URGENCY_CAP - self.deadline)

    def __repr__(self):
        return (f"{self.id}(w={self.weight}kg, pr={self.priority}, "
                f"dl={self.deadline}h, score={self.score}, node={self.dest})")


def load_parcels(path):
    """Read parcels from a CSV file: id,weight,priority,deadline,dest"""
    parcels = []
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            parcels.append(Parcel(r["id"], r["weight"], r["priority"],
                                  r["deadline"], r["dest"]))
    return parcels


def load_graph(path):
    """Read the road network from a CSV edge list: u,v,distance (undirected).
    Returns an adjacency list  {node: [(neighbour, distance), ...]}."""
    graph = {}
    with open(path, newline="") as f:
        for r in csv.DictReader(f):
            u, v, w = int(r["u"]), int(r["v"]), float(r["distance"])
            graph.setdefault(u, []).append((v, w))
            graph.setdefault(v, []).append((u, w))      # undirected roads
    return graph


# ----------------------------------------------------------------------------
# 2. PHASE 2a : SORTING  (Merge Sort - stable, O(n log n) in best/avg/worst case)
# ----------------------------------------------------------------------------
def sort_key(p):
    """Earliest deadline first; ties broken by higher priority first."""
    return (p.deadline, -p.priority)


def merge_sort(items, key=sort_key):
    if len(items) <= 1:
        return items[:]
    mid = len(items) // 2
    left = merge_sort(items[:mid], key)
    right = merge_sort(items[mid:], key)
    # merge step
    out, i, j = [], 0, 0
    while i < len(left) and j < len(right):
        if key(left[i]) <= key(right[j]):       # '<=' keeps the sort stable
            out.append(left[i]); i += 1
        else:
            out.append(right[j]); j += 1
    out.extend(left[i:])
    out.extend(right[j:])
    return out


# ----------------------------------------------------------------------------
# 3. PHASE 2b : 0/1 KNAPSACK  (Dynamic Programming, O(n*W) time and space)
# ----------------------------------------------------------------------------
def knapsack(parcels, capacity):
    """Pick the subset of parcels with maximum total score whose total
    weight <= capacity.  Returns (best_value, chosen_parcels, dp_table)."""
    n = len(parcels)
    # dp[i][w] = best score using the first i parcels with capacity w
    dp = [[0] * (capacity + 1) for _ in range(n + 1)]
    for i in range(1, n + 1):
        wt, val = parcels[i - 1].weight, parcels[i - 1].score
        for w in range(capacity + 1):
            dp[i][w] = dp[i - 1][w]                         # skip parcel i
            if wt <= w and dp[i - 1][w - wt] + val > dp[i][w]:
                dp[i][w] = dp[i - 1][w - wt] + val          # take parcel i
    # back-track through the table to recover WHICH parcels were taken
    chosen, w = [], capacity
    for i in range(n, 0, -1):
        if dp[i][w] != dp[i - 1][w]:
            chosen.append(parcels[i - 1])
            w -= parcels[i - 1].weight
    chosen.reverse()
    return dp[n][capacity], chosen, dp


# ----------------------------------------------------------------------------
# 4. PHASE 3a : DIJKSTRA  (binary heap, O((V+E) log V))
# ----------------------------------------------------------------------------
def dijkstra(graph, source):
    """Shortest distance from `source` to every node.
    Returns (dist, prev) so that paths can be rebuilt."""
    dist = {v: INF for v in graph}
    prev = {v: None for v in graph}
    dist[source] = 0
    heap = [(0, source)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue                       # stale entry, skip
        for v, w in graph[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    return dist, prev


def build_path(prev, target):
    path = []
    while target is not None:
        path.append(target)
        target = prev[target]
    return path[::-1]


# ----------------------------------------------------------------------------
# 5. PHASE 3b : NEAREST-NEIGHBOUR TSP APPROXIMATION  (O(k^2) after Dijkstra)
# ----------------------------------------------------------------------------
def nearest_neighbour_route(graph, stops, depot=DEPOT):
    """Visit every node in `stops` starting & ending at the depot.
    Distances between stops are the Dijkstra shortest-road distances.
    Returns (stop_order, total_distance, full_road_path)."""
    targets = set(stops) - {depot}
    # one Dijkstra run per stop (k+1 runs) gives all the pairwise distances needed
    sp = {s: dijkstra(graph, s) for s in targets | {depot}}

    order, total, current = [depot], 0.0, depot
    full_path = [depot]
    while targets:
        nxt = min(targets, key=lambda s: sp[current][0][s])   # closest unvisited
        total += sp[current][0][nxt]
        full_path += build_path(sp[current][1], nxt)[1:]      # road nodes in between
        order.append(nxt)
        targets.remove(nxt)
        current = nxt
    total += sp[current][0][depot]                            # return to depot
    full_path += build_path(sp[current][1], depot)[1:]
    order.append(depot)
    return order, total, full_path


# ----------------------------------------------------------------------------
# 6. INTEGRATION : one complete planning run
# ----------------------------------------------------------------------------
def plan_trip(parcels, graph, capacity, verbose=True):
    log = print if verbose else (lambda *a, **k: None)

    # Step 1 - prioritise
    sorted_parcels = merge_sort(parcels)
    log("\n=== STEP 1: PARCELS SORTED (earliest deadline, then highest priority) ===")
    for p in sorted_parcels:
        log("  ", p)

    # Step 2 - choose the parcels for this trip
    best, chosen, dp = knapsack(sorted_parcels, capacity)
    log(f"\n=== STEP 2: KNAPSACK (capacity = {capacity} kg) ===")
    log(f"  Best total score    : {best}")
    log(f"  Total weight loaded : {sum(p.weight for p in chosen)} kg")
    log("  Selected parcels    :", [p.id for p in chosen])
    left_behind = [p.id for p in sorted_parcels if p not in chosen]
    log("  Left for next trip  :", left_behind)

    # Step 3 - route over the destinations of the chosen parcels
    stops = sorted({p.dest for p in chosen})
    order, dist, path = nearest_neighbour_route(graph, stops)
    log("\n=== STEP 3: ROUTE (Dijkstra + Nearest-Neighbour TSP) ===")
    log("  Stop order      :", " -> ".join(map(str, order)))
    log("  Full road path  :", " -> ".join(map(str, path)))
    log(f"  Total distance  : {dist:.1f} km")
    return dict(sorted=sorted_parcels, best=best, chosen=chosen, dp=dp,
                order=order, distance=dist, path=path, left=left_behind)


def print_dp_table(parcels, dp, capacity, max_cols=21):
    """Pretty-print the DP table (first max_cols capacity columns)."""
    cols = min(capacity, max_cols - 1) + 1
    print("\n--- DP table  dp[i][w]  (rows = parcels considered, cols = capacity) ---")
    print("        " + "".join(f"{w:4d}" for w in range(cols)))
    for i, row in enumerate(dp):
        label = "none" if i == 0 else parcels[i - 1].id
        print(f"{label:>6}  " + "".join(f"{x:4d}" for x in row[:cols]))


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    parcels = load_parcels(os.path.join(here, "data", "parcels_small.csv"))
    graph = load_graph(os.path.join(here, "data", "graph_small.csv"))
    CAPACITY = 20                                    # van capacity in kg
    result = plan_trip(parcels, graph, CAPACITY)
    print_dp_table(result["sorted"], result["dp"], CAPACITY)
