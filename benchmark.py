

import csv
import itertools
import os
import random
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from courier_planner import (Parcel, merge_sort, sort_key, knapsack, dijkstra,
                             nearest_neighbour_route, load_parcels, load_graph,
                             plan_trip, DEPOT, INF)

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")
OUT = os.path.join(HERE, "outputs")
random.seed(42)                                   # reproducible results
report = []


def say(*a):
    line = " ".join(str(x) for x in a)
    print(line)
    report.append(line)


def timeit(fn, *args, repeat=3):
    """Best-of-`repeat` wall-clock time in milliseconds."""
    best = INF
    for _ in range(repeat):
        t = time.perf_counter()
        fn(*args)
        best = min(best, time.perf_counter() - t)
    return best * 1000


# ------------------------------------------------------------------ data generators
def random_parcels(n, num_nodes):
    return [Parcel(f"P{i+1}", random.randint(1, 15), random.randint(1, 10),
                   random.randint(1, 24), random.randint(1, num_nodes - 1))
            for i in range(n)]


def random_graph(v, extra_edges_factor=2):
    """Connected random road network: a random spanning tree + extra edges."""
    g = {i: [] for i in range(v)}
    edges = set()

    def add(a, b):
        if a != b and (min(a, b), max(a, b)) not in edges:
            w = random.randint(1, 20)
            edges.add((min(a, b), max(a, b), w))
            g[a].append((b, w)); g[b].append((a, w))
            return True
        return False
    for i in range(1, v):
        add(i, random.randint(0, i - 1))          # spanning tree -> connected
    for _ in range(v * extra_edges_factor):
        add(random.randrange(v), random.randrange(v))
    return g, sorted(edges)


def insertion_sort(items, key=sort_key):
    a = items[:]
    for i in range(1, len(a)):
        cur, j = a[i], i - 1
        while j >= 0 and key(a[j]) > key(cur):
            a[j + 1] = a[j]; j -= 1
        a[j + 1] = cur
    return a


# ------------------------------------------------------------------ 1. correctness
def correctness_tests():
    say("=" * 70); say("CORRECTNESS TESTS"); say("=" * 70)

    # Sorting vs Python's built-in sorted()
    ok = True
    for n in (0, 1, 2, 10, 200):
        ps = random_parcels(n, 10)
        ok &= [p.id for p in merge_sort(ps)] == [p.id for p in sorted(ps, key=sort_key)]
    say(f"[Merge sort]  matches built-in sorted() on 5 test sizes ........ {'PASS' if ok else 'FAIL'}")

    # Knapsack vs brute force over all 2^n subsets
    ok = True
    for _ in range(30):
        ps = random_parcels(random.randint(3, 12), 10)
        cap = random.randint(5, 60)
        best = max(sum(p.score for p in c)
                   for r in range(len(ps) + 1) for c in itertools.combinations(ps, r)
                   if sum(p.weight for p in c) <= cap)
        val, chosen, _ = knapsack(ps, cap)
        ok &= (val == best and sum(p.weight for p in chosen) <= cap
               and sum(p.score for p in chosen) == val)
    say(f"[Knapsack DP] equals brute-force optimum on 30 random cases ..... {'PASS' if ok else 'FAIL'}")

    # Dijkstra vs Floyd-Warshall
    ok = True
    for _ in range(10):
        v = 25
        g, edges = random_graph(v)
        fw = [[INF] * v for _ in range(v)]
        for i in range(v):
            fw[i][i] = 0
        for a, b, w in edges:
            fw[a][b] = fw[b][a] = min(fw[a][b], w)
        for k in range(v):
            for i in range(v):
                for j in range(v):
                    if fw[i][k] + fw[k][j] < fw[i][j]:
                        fw[i][j] = fw[i][k] + fw[k][j]
        d, _ = dijkstra(g, 0)
        ok &= all(d[j] == fw[0][j] for j in range(v))
    say(f"[Dijkstra]    equals Floyd-Warshall on 10 random graphs ......... {'PASS' if ok else 'FAIL'}")

    # Nearest neighbour vs brute-force optimal tour -> approximation ratio
    ratios = []
    for _ in range(20):
        g, _ = random_graph(30)
        stops = random.sample(range(1, 30), 6)
        _, nn_len, _ = nearest_neighbour_route(g, stops)
        sp = {s: dijkstra(g, s)[0] for s in stops + [DEPOT]}
        opt = min(sp[DEPOT][p[0]] + sum(sp[p[i]][p[i + 1]] for i in range(5)) + sp[p[-1]][DEPOT]
                  for p in itertools.permutations(stops))
        ratios.append(nn_len / opt)
    say(f"[NN-TSP]      tour length / optimal tour (20 cases, 6 stops): "
        f"avg {sum(ratios)/len(ratios):.3f}, worst {max(ratios):.3f}, never below 1.0: "
        f"{'PASS' if min(ratios) >= 1 - 1e-9 else 'FAIL'}")


# ------------------------------------------------------------------ 2. runtime scaling
def scaling_experiments():
    say("\n" + "=" * 70); say("RUNTIME SCALING (milliseconds, best of 3)"); say("=" * 70)
    res = {}

    # sorting: merge sort vs insertion sort vs built-in
    sizes = [100, 500, 1000, 2000, 4000]
    res["sort"] = {"n": sizes, "merge": [], "insertion": [], "builtin": []}
    say("\nSorting parcels      n    MergeSort   InsertionSort   built-in sorted")
    for n in sizes:
        ps = random_parcels(n, 50)
        m, i, b = timeit(merge_sort, ps), timeit(insertion_sort, ps), timeit(lambda x: sorted(x, key=sort_key), ps)
        res["sort"]["merge"].append(m); res["sort"]["insertion"].append(i); res["sort"]["builtin"].append(b)
        say(f"                  {n:5d}   {m:9.2f}   {i:13.2f}   {b:15.3f}")

    # knapsack: grows with n * W
    ks = [(50, 100), (100, 200), (200, 400), (400, 800)]
    res["knap"] = {"label": [], "time": []}
    say("\nKnapsack DP     (n parcels, W capacity)   cells=n*W    time(ms)")
    for n, W in ks:
        ps = random_parcels(n, 50)
        t = timeit(knapsack, ps, W, repeat=2)
        res["knap"]["label"].append(f"{n}x{W}"); res["knap"]["time"].append(t)
        say(f"                       {n:4d} x {W:4d}          {n*W:8d}   {t:9.2f}")

    # dijkstra: V vertices, ~3V edges
    vs = [100, 500, 1000, 2000, 4000]
    res["dij"] = {"v": vs, "time": []}
    say("\nDijkstra (single source)   V vertices   time(ms)")
    for v in vs:
        g, _ = random_graph(v)
        t = timeit(dijkstra, g, 0)
        res["dij"]["time"].append(t)
        say(f"                          {v:8d}   {t:9.2f}")

    # NN-TSP: k stops on a 500-node graph
    ks2 = [5, 10, 20, 40, 80]
    g, _ = random_graph(500)
    res["tsp"] = {"k": ks2, "time": []}
    say("\nNN-TSP on 500-node graph   k stops   time(ms)   (includes k+1 Dijkstra runs)")
    for k in ks2:
        stops = random.sample(range(1, 500), k)
        t = timeit(nearest_neighbour_route, g, stops, repeat=2)
        res["tsp"]["time"].append(t)
        say(f"                          {k:8d}   {t:9.2f}")
    return res


# ------------------------------------------------------------------ 3. small vs large pipeline
def write_large_dataset(n_parcels=1000, v=500):
    g, edges = random_graph(v)
    ps = random_parcels(n_parcels, v)
    with open(os.path.join(DATA, "parcels_large.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["id", "weight", "priority", "deadline", "dest"])
        for p in ps:
            w.writerow([p.id, p.weight, p.priority, p.deadline, p.dest])
    with open(os.path.join(DATA, "graph_large.csv"), "w", newline="") as f:
        w = csv.writer(f); w.writerow(["u", "v", "distance"]); w.writerows(edges)


def pipeline_timing():
    """Time each stage of the full pipeline on the small and the large dataset."""
    say("\n" + "=" * 70); say("FULL PIPELINE: SMALL vs LARGE DATASET"); say("=" * 70)
    stages = {}
    for name, pf, gf, cap in (("small", "parcels_small.csv", "graph_small.csv", 20),
                              ("large", "parcels_large.csv", "graph_large.csv", 150)):
        parcels = load_parcels(os.path.join(DATA, pf))
        graph = load_graph(os.path.join(DATA, gf))
        t0 = time.perf_counter(); sp = merge_sort(parcels)
        t1 = time.perf_counter(); best, chosen, _ = knapsack(sp, cap)
        t2 = time.perf_counter()
        stops = sorted({p.dest for p in chosen})
        order, dist, _ = nearest_neighbour_route(graph, stops)
        t3 = time.perf_counter()
        stages[name] = [(t1 - t0) * 1000, (t2 - t1) * 1000, (t3 - t2) * 1000]
        say(f"{name:5s}: {len(parcels):4d} parcels, {len(graph):3d} nodes, capacity {cap:3d} kg | "
            f"loaded {len(chosen):3d} parcels ({sum(p.weight for p in chosen)} kg, score {best}) | "
            f"{len(stops):3d} stops, route {dist:.0f} km")
        say(f"       sort {stages[name][0]:8.2f} ms | knapsack {stages[name][1]:8.2f} ms | "
            f"route (Dijkstra+NN) {stages[name][2]:8.2f} ms")
    return stages


# ------------------------------------------------------------------ charts
def make_charts(res, stages):
    # Chart 1 - sorting comparison
    s = res["sort"]
    plt.figure(figsize=(7, 4.5))
    plt.plot(s["n"], s["insertion"], "o-", label="Insertion sort  O(n²)")
    plt.plot(s["n"], s["merge"], "s-", label="Merge sort  O(n log n)")
    plt.plot(s["n"], s["builtin"], "^-", label="Python sorted() (Timsort)")
    plt.xlabel("Number of parcels (n)"); plt.ylabel("Time (ms)")
    plt.title("Sorting runtime vs input size"); plt.legend(); plt.grid(alpha=.3)
    plt.tight_layout(); plt.savefig(os.path.join(OUT, "chart1_sorting.png"), dpi=150); plt.close()

    # Chart 2 - scaling of knapsack, dijkstra, tsp
    fig, ax = plt.subplots(1, 3, figsize=(13, 4))
    ax[0].bar(res["knap"]["label"], res["knap"]["time"], color="#d9822b")
    ax[0].set_title("Knapsack DP  O(n·W)"); ax[0].set_xlabel("parcels x capacity"); ax[0].set_ylabel("Time (ms)")
    ax[1].plot(res["dij"]["v"], res["dij"]["time"], "o-", color="#2b7ad9")
    ax[1].set_title("Dijkstra  O((V+E) log V)"); ax[1].set_xlabel("Vertices (V)")
    ax[2].plot(res["tsp"]["k"], res["tsp"]["time"], "o-", color="#2bb673")
    ax[2].set_title("NN-TSP (500-node graph)"); ax[2].set_xlabel("Delivery stops (k)")
    for a in ax:
        a.grid(alpha=.3)
    plt.tight_layout(); plt.savefig(os.path.join(OUT, "chart2_scaling.png"), dpi=150); plt.close()

    # Chart 3 - stage breakdown small vs large (log scale)
    labels = ["Sort", "Knapsack", "Route"]
    x = range(3)
    plt.figure(figsize=(7, 4.5))
    plt.bar([i - .2 for i in x], stages["small"], .4, label="Small dataset (10 parcels)")
    plt.bar([i + .2 for i in x], stages["large"], .4, label="Large dataset (1000 parcels)")
    plt.xticks(list(x), labels); plt.yscale("log"); plt.ylabel("Time (ms, log scale)")
    plt.title("Runtime of each pipeline stage"); plt.legend(); plt.grid(alpha=.3, axis="y")
    plt.tight_layout(); plt.savefig(os.path.join(OUT, "chart3_stage_comparison.png"), dpi=150); plt.close()


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    correctness_tests()
    results = scaling_experiments()
    write_large_dataset()
    stage_times = pipeline_timing()
    make_charts(results, stage_times)
    with open(os.path.join(OUT, "benchmark_results.txt"), "w") as f:
        f.write("\n".join(report))
    print("\nCharts saved in outputs/ : chart1_sorting.png, chart2_scaling.png, chart3_stage_comparison.png")
