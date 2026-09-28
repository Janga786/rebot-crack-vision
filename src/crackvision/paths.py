"""crackvision.paths — ordered crack polylines from a skeleton graph. Built by PERC-03.

Consumes the pixel graph PERC-02 builds over a `{0,255}` skeleton raster and turns it into a
deterministic, ordered set of polylines per crack component, in the original `(row, col)` frame
(docs/INTERFACES.md §0.5). Schema for the on-disk artifact is docs/INTERFACES.md §3.13.

Policy (docs/INTERFACES.md §3.13), applied per connected component of the pruned graph:

1. **Path decomposition = repeated graph-diameter extraction.** The component's *main path* is
   its graph diameter (docs/adr's usual sense: the longest shortest-path between any two nodes,
   found by the standard two-sweep Dijkstra used to compute tree diameters — exact for the tree-
   shaped components that make up the overwhelming majority of real cracks, and the conventional
   heuristic when a component contains a cycle). Its edges are then removed and the diameter of
   whatever remains becomes the next branch; this repeats until every edge has been claimed by
   some polyline. Branches are then sorted by pixel length, longest first.
2. **Multi-component ordering** is a deterministic greedy nearest-endpoint tour: starting from the
   first component (component discovery order, itself deterministic — see `skeleton_graph.
   build_graph`), each subsequent component is the one whose nearer endpoint (start or end of its
   main path — a direction choice) is closest to the current position; ties break on the lower
   component id. This does not attempt a globally optimal tour, only a cheap, reproducible one.
3. **Simplification.** Both the dense (one point per skeleton pixel) and an RDP-simplified
   polyline (tolerance in px, from config) are kept for every path/branch, dense first.
4. **Junction crossing.** A PERC-02 edge can end on a raw junction pixel next to the graph node
   instead of on it (`skeleton_graph.build_graph` merges mutually-adjacent junction pixels into one
   node keyed by their cluster's representative pixel). Wherever two consecutive edges in a
   decomposed path don't meet on the same pixel, the node's own pixel is spliced in so the dense
   path still passes through the junction/cluster and stays 8-connected (no step exceeds a
   Chebyshev distance of 1) — no traversed edge pixel is ever dropped.
"""

from __future__ import annotations

import argparse
import json
import logging
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np
from PIL import Image

from crackvision.config import Config, ConfigError, add_common_args, load_config
from crackvision.logging_setup import (
    EXIT_OK,
    EXIT_PRECONDITION,
    EXIT_RUNTIME,
    EXIT_USAGE,
    RunSummary,
    setup_logging,
)
from crackvision.skeleton_graph import (
    Edge,
    Node,
    SkeletonGraph,
    build_graph,
    prune_spurs,
    spur_threshold_from_config,
)

SCHEMA_VERSION = 1
DEFAULT_RDP_TOLERANCE_PX = 1.5

Pixel = tuple[int, int]


# ---------------------------------------------------------------------------
# RDP simplification
# ---------------------------------------------------------------------------


def rdp_simplify(points: list[Pixel], tolerance_px: float) -> list[Pixel]:
    """Ramer-Douglas-Peucker on a dense pixel chain; iterative (no recursion-depth limit)."""
    if len(points) < 3 or tolerance_px < 0:
        return list(points)

    keep = [False] * len(points)
    keep[0] = keep[-1] = True
    stack = [(0, len(points) - 1)]
    pts = np.asarray(points, dtype=np.float64)

    while stack:
        lo, hi = stack.pop()
        if hi - lo < 2:
            continue
        p0, p1 = pts[lo], pts[hi]
        seg = p1 - p0
        norm = np.linalg.norm(seg)
        segment_pts = pts[lo + 1 : hi]
        if norm == 0:
            dist = np.linalg.norm(segment_pts - p0, axis=1)
        else:
            rel = segment_pts - p0
            dist = np.abs(seg[0] * rel[:, 1] - seg[1] * rel[:, 0]) / norm
        if dist.size == 0:
            continue
        i_local = int(dist.argmax())
        if dist[i_local] <= tolerance_px:
            continue
        i = lo + 1 + i_local
        keep[i] = True
        stack.append((lo, i))
        stack.append((i, hi))

    return [p for p, k in zip(points, keep) if k]


def rdp_tolerance_from_config(cfg: Any) -> float:
    """`cfg.raw['path_extraction']['rdp_tolerance_px']`, else `DEFAULT_RDP_TOLERANCE_PX`.

    `cfg` is duck-typed (a `crackvision.config.Config` or any object with a `.raw` dict) so tests
    can pass a plain namespace, mirroring `skeleton_graph.spur_threshold_from_config`.
    """
    raw = getattr(cfg, "raw", {}) or {}
    section = raw.get("path_extraction", {}) or {}
    return float(section.get("rdp_tolerance_px", DEFAULT_RDP_TOLERANCE_PX))


# ---------------------------------------------------------------------------
# Component splitting
# ---------------------------------------------------------------------------


def _node_components(graph: SkeletonGraph) -> list[set[int]]:
    """Connected components of node ids, in deterministic discovery order."""
    adjacency: dict[int, set[int]] = {node_id: set() for node_id in graph.nodes}
    for edge in graph.edges.values():
        adjacency[edge.node_a].add(edge.node_b)
        adjacency[edge.node_b].add(edge.node_a)

    seen: set[int] = set()
    components: list[set[int]] = []
    for node_id in sorted(graph.nodes):
        if node_id in seen:
            continue
        component: set[int] = set()
        stack = [node_id]
        while stack:
            n = stack.pop()
            if n in component:
                continue
            component.add(n)
            stack.extend(adjacency[n] - component)
        seen |= component
        components.append(component)
    return components


# ---------------------------------------------------------------------------
# Repeated-diameter path decomposition
# ---------------------------------------------------------------------------


def _dijkstra(adjacency: dict[int, list[tuple[Edge, int]]], source: int) -> tuple[dict[int, float], dict[int, tuple[int, Edge]]]:
    """Shortest-path distances/predecessors from `source`, edge weight = `length_px`.

    Self-loop edges (`edge.node_a == edge.node_b`) never relax anything (an edge back to the same
    node can only add distance) so they are naturally excluded from every shortest-path tree.
    """
    dist: dict[int, float] = {source: 0.0}
    prev: dict[int, tuple[int, Edge]] = {}
    visited: set[int] = set()

    while True:
        unvisited = [(d, n) for n, d in dist.items() if n not in visited]
        if not unvisited:
            break
        _, u = min(unvisited)
        visited.add(u)
        for edge, v in adjacency.get(u, []):
            if v == u:
                continue
            nd = dist[u] + edge.length_px
            if v not in dist or nd < dist[v]:
                dist[v] = nd
                prev[v] = (u, edge)
    return dist, prev


def _oriented_pixels(edge: Edge, from_node: int) -> tuple[Pixel, ...]:
    """`edge.pixels` walked starting at `from_node`'s end."""
    if edge.node_a == from_node:
        return edge.pixels
    return tuple(reversed(edge.pixels))


def _farthest_node(dist: dict[int, float]) -> int:
    """Deterministic argmax: largest distance, ties broken by the smaller node id."""
    return min(dist.items(), key=lambda kv: (-kv[1], kv[0]))[0]


def _longest_remaining_path(remaining: dict[int, Edge]) -> tuple[int, list[Edge]]:
    """The longest simple path (graph diameter) among `remaining` edges: `(start_node, edges)`."""
    simple_edges = {eid: e for eid, e in remaining.items() if e.node_a != e.node_b}
    if simple_edges:
        adjacency: dict[int, list[tuple[Edge, int]]] = {}
        for edge in simple_edges.values():
            adjacency.setdefault(edge.node_a, []).append((edge, edge.node_b))
            adjacency.setdefault(edge.node_b, []).append((edge, edge.node_a))

        start = min(adjacency)
        dist1, _ = _dijkstra(adjacency, start)
        node_a = _farthest_node(dist1)
        dist2, prev2 = _dijkstra(adjacency, node_a)
        node_b = _farthest_node(dist2)

        if node_b != node_a:
            path_edges: list[Edge] = []
            cur = node_b
            while cur != node_a:
                u, edge = prev2[cur]
                path_edges.append(edge)
                cur = u
            path_edges.reverse()
            return node_a, path_edges

    # No simple-edge component reachable from any other node (an isolated self-loop, or a
    # single stray edge): fall back to the single longest remaining edge.
    edge = max(remaining.values(), key=lambda e: (e.length_px, -e.id))
    return edge.node_a, [edge]


def _decompose_edges(edges: dict[int, Edge]) -> list[tuple[int, list[Edge]]]:
    """Repeated-diameter decomposition: main path first, then each successive branch."""
    remaining = dict(edges)
    decomposition: list[tuple[int, list[Edge]]] = []
    while remaining:
        start_node, path_edges = _longest_remaining_path(remaining)
        for edge in path_edges:
            del remaining[edge.id]
        decomposition.append((start_node, path_edges))
    return decomposition


def _chebyshev(a: Pixel, b: Pixel) -> int:
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def _step_path(a: Pixel, b: Pixel) -> list[Pixel]:
    """8-connected straight-line steps from `a` to `b`, exclusive of `a`, inclusive of `b`."""
    (r0, c0), (r1, c1) = a, b
    dr, dc = r1 - r0, c1 - c0
    steps = max(abs(dr), abs(dc))
    return [(r0 + round(dr * i / steps), c0 + round(dc * i / steps)) for i in range(1, steps + 1)]


def _bridge(last: Pixel, target: Pixel, via: Pixel) -> list[Pixel]:
    """8-connected pixels from `last` to `target` (exclusive of `last`), routed through `via`.

    PERC-02 edges can end on a raw junction pixel next to the merged junction node instead of on
    it (`skeleton_graph.build_graph` merges mutually-adjacent junction pixels into one node, keyed
    by the cluster's representative pixel — not necessarily the pixel any given edge happens to
    touch). When that leaves a gap wider than one 8-connected step, this routes through the node's
    own pixel (`via`) so the dense path still passes through the junction/cluster instead of
    jumping over it.
    """
    if _chebyshev(last, target) <= 1:
        return [target]
    if via not in (last, target):
        return _step_path(last, via) + _step_path(via, target)
    return _step_path(last, target)


def _edge_chain_to_pixels(path_edges: list[Edge], first_node: int, nodes: dict[int, Node]) -> tuple[Pixel, ...]:
    """Concatenate an ordered edge chain into one dense, 8-connected pixel path.

    Every point of every traversed edge is kept; a junction/cluster pixel (`nodes[node_id]`) is
    inserted wherever consecutive edges don't meet on the same pixel, so no step exceeds a
    Chebyshev distance of 1.
    """
    pixels: list[Pixel] = []
    cur_node = first_node
    for edge in path_edges:
        oriented = _oriented_pixels(edge, cur_node)
        next_node = edge.node_b if edge.node_a == cur_node else edge.node_a
        cur_node_pixel = (nodes[cur_node].row, nodes[cur_node].col)
        for p in oriented:
            if not pixels:
                pixels.append(p)
            elif pixels[-1] == p:
                continue
            else:
                pixels.extend(_bridge(pixels[-1], p, cur_node_pixel))
        cur_node = next_node

    end_pixel = (nodes[cur_node].row, nodes[cur_node].col)
    if pixels and pixels[-1] != end_pixel:
        pixels.extend(_bridge(pixels[-1], end_pixel, end_pixel))
    return tuple(pixels)


# ---------------------------------------------------------------------------
# Public data model
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class Polyline:
    """One ordered crack polyline: dense pixel path plus its RDP-simplified counterpart."""

    kind: str  # "main" | "branch"
    dense: tuple[Pixel, ...]
    simplified: tuple[Pixel, ...]

    @property
    def length_px(self) -> int:
        """Pixel-step count along the dense path (`len(dense) - 1`, 0 for a single point)."""
        return max(0, len(self.dense) - 1)


@dataclass(frozen=True)
class ComponentPaths:
    """One connected component's main path plus its branches, longest branch first."""

    component_id: int
    main_path: Polyline
    branches: tuple[Polyline, ...]


def extract_component_paths(graph: SkeletonGraph, *, rdp_tolerance_px: float) -> list[ComponentPaths]:
    """Decompose every connected component of `graph` into an ordered main path + branches."""
    components: list[ComponentPaths] = []
    for component_id, node_ids in enumerate(_node_components(graph)):
        component_edges = {
            e.id: e for e in graph.edges.values() if e.node_a in node_ids and e.node_b in node_ids
        }
        if not component_edges:
            # An isolated node with no edges at all: a single-pixel polyline.
            (node_id,) = node_ids
            node = graph.nodes[node_id]
            point = (node.row, node.col)
            polyline = Polyline(kind="main", dense=(point,), simplified=(point,))
            components.append(ComponentPaths(component_id=component_id, main_path=polyline, branches=()))
            continue

        decomposition = _decompose_edges(component_edges)
        polylines: list[Polyline] = []
        for start_node, path_edges in decomposition:
            dense = _edge_chain_to_pixels(path_edges, start_node, graph.nodes)
            simplified = tuple(rdp_simplify(list(dense), rdp_tolerance_px))
            polylines.append(Polyline(kind="main", dense=dense, simplified=simplified))

        main_path = polylines[0]
        branches = tuple(
            Polyline(kind="branch", dense=p.dense, simplified=p.simplified)
            for p in sorted(polylines[1:], key=lambda p: (-p.length_px, p.dense))
        )
        components.append(ComponentPaths(component_id=component_id, main_path=main_path, branches=branches))
    return components


# ---------------------------------------------------------------------------
# Multi-component ordering
# ---------------------------------------------------------------------------


def _euclidean(a: Pixel, b: Pixel) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def order_components(components: list[ComponentPaths]) -> list[ComponentPaths]:
    """Deterministic greedy nearest-endpoint tour; may flip a component's main path direction.

    The first component in `components` (already in deterministic discovery order) starts the
    tour unchanged. Each subsequent step picks whichever remaining component has an endpoint (of
    its main path — start or end, a direction choice) nearest the current position, breaking ties
    on the lower `component_id`; that component's `main_path.dense`/`simplified` are reversed if
    its *end* was the nearer point, so the tour reads as a single continuous traversal.
    """
    if len(components) <= 1:
        return list(components)

    remaining = list(components)
    ordered = [remaining.pop(0)]
    current = ordered[0].main_path.dense[-1]

    while remaining:
        best_idx = None
        best_flip = False
        best_dist = None
        for idx, comp in enumerate(remaining):
            start, end = comp.main_path.dense[0], comp.main_path.dense[-1]
            d_start, d_end = _euclidean(current, start), _euclidean(current, end)
            flip = d_end < d_start
            dist = d_end if flip else d_start
            key = (dist, comp.component_id)
            if best_dist is None or key < best_dist:
                best_dist = key
                best_idx = idx
                best_flip = flip
        comp = remaining.pop(best_idx)
        if best_flip:
            flipped_main = Polyline(
                kind="main", dense=tuple(reversed(comp.main_path.dense)), simplified=tuple(reversed(comp.main_path.simplified))
            )
            comp = ComponentPaths(component_id=comp.component_id, main_path=flipped_main, branches=comp.branches)
        ordered.append(comp)
        current = comp.main_path.dense[-1]

    return ordered


# ---------------------------------------------------------------------------
# JSON assembly
# ---------------------------------------------------------------------------


def _point_dict(pixel: Pixel) -> dict[str, int]:
    row, col = pixel
    return {"row": int(row), "col": int(col), "u": int(col), "v": int(row)}


def _polyline_dict(polyline: Polyline) -> dict[str, Any]:
    return {
        "kind": polyline.kind,
        "length_px": polyline.length_px,
        "dense": [_point_dict(p) for p in polyline.dense],
        "simplified": [_point_dict(p) for p in polyline.simplified],
    }


def case_paths_to_dict(
    case_id: str,
    height: int,
    width: int,
    components: list[ComponentPaths],
    *,
    min_spur_length_px: int,
    rdp_tolerance_px: float,
) -> dict[str, Any]:
    """Build the `data/paths/{case}_paths.json` document (docs/INTERFACES.md §3.13)."""
    return {
        "case_id": case_id,
        "schema_version": SCHEMA_VERSION,
        "image_height": int(height),
        "image_width": int(width),
        "min_spur_length_px": int(min_spur_length_px),
        "rdp_tolerance_px": float(rdp_tolerance_px),
        "component_count": len(components),
        "components": [
            {
                "order_index": order_index,
                "component_id": comp.component_id,
                "main_path": _polyline_dict(comp.main_path),
                "branches": [_polyline_dict(b) for b in comp.branches],
            }
            for order_index, comp in enumerate(components)
        ],
    }


def build_case_paths(
    skeleton: np.ndarray,
    case_id: str,
    *,
    min_spur_length_px: int,
    rdp_tolerance_px: float,
) -> dict[str, Any]:
    """End-to-end: `{0,255}`/bool skeleton raster -> the `{case}_paths.json` document dict."""
    height, width = skeleton.shape[:2]
    graph = build_graph(np.asarray(skeleton) > 0)
    graph = prune_spurs(graph, min_spur_length_px)
    components = extract_component_paths(graph, rdp_tolerance_px=rdp_tolerance_px)
    components = order_components(components)
    return case_paths_to_dict(
        case_id,
        height,
        width,
        components,
        min_spur_length_px=min_spur_length_px,
        rdp_tolerance_px=rdp_tolerance_px,
    )


# ---------------------------------------------------------------------------
# CLI (docs/INTERFACES.md §3.13)
# ---------------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="paths.py",
        description="Ordered crack polylines from a skeleton raster (docs/INTERFACES.md §3.13).",
    )
    add_common_args(parser)
    parser.add_argument(
        "--cases", nargs="+", default=None, metavar="CASE_ID", help="restrict to named case_id(s) (default: all)"
    )
    parser.add_argument(
        "--rdp-tolerance-px",
        type=float,
        default=None,
        metavar="PX",
        help="RDP simplification tolerance in px (default: config/project.yaml)",
    )
    parser.add_argument(
        "--min-spur-length-px",
        type=int,
        default=None,
        metavar="N",
        help="drop endpoint spurs shorter than N px before path extraction (default: config/project.yaml)",
    )
    return parser


def _paths_case(
    cfg: Config,
    case_id: str,
    *,
    min_spur_length_px: int,
    rdp_tolerance_px: float,
    skeletons_dir: Path,
    paths_dir: Path,
    skip_existing: bool,
    logger: logging.Logger,
) -> dict[str, Any]:
    skeleton_path = skeletons_dir / f"{case_id}_skeleton.png"
    out_path = paths_dir / f"{case_id}_paths.json"

    if skip_existing and out_path.is_file():
        logger.info("skip-existing: case %s already has paths output", case_id)
        return {"case_id": case_id, "status": "skipped", "paths_path": out_path}

    if not skeleton_path.is_file():
        msg = f"skeleton missing: {skeleton_path} (run: ./env.sh python -m crackvision.skeleton first)"
        logger.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    try:
        with Image.open(skeleton_path) as img:
            img.load()
            skeleton = np.array(img) > 0
    except Exception as exc:  # noqa: BLE001 — one bad case must never abort the batch
        msg = f"failed to read skeleton {skeleton_path}: {type(exc).__name__}: {exc}"
        logger.error("case %s: %s", case_id, msg)
        return {"case_id": case_id, "status": "failed", "error": msg}

    doc = build_case_paths(
        skeleton, case_id, min_spur_length_px=min_spur_length_px, rdp_tolerance_px=rdp_tolerance_px
    )
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(doc, indent=2) + "\n", encoding="utf-8")

    logger.info(
        "case %s: %d component(s), main path length(s) %s px",
        case_id,
        doc["component_count"],
        [c["main_path"]["length_px"] for c in doc["components"]],
    )
    return {"case_id": case_id, "status": "ok", "paths_path": out_path, **doc}


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)

    try:
        cfg: Config = load_config(path=args.config, root=args.root)
    except ConfigError as exc:
        print(f"paths: {exc}", file=sys.stderr)
        return EXIT_USAGE

    min_spur_length_px = (
        spur_threshold_from_config(cfg) if args.min_spur_length_px is None else args.min_spur_length_px
    )
    rdp_tolerance_px = rdp_tolerance_from_config(cfg) if args.rdp_tolerance_px is None else args.rdp_tolerance_px

    logger = setup_logging("paths", cfg, verbose=args.verbose, quiet=args.quiet)
    summary = RunSummary()

    case_map_path = cfg.root / "data" / "case_map.json"
    if not case_map_path.is_file():
        msg = "run: ./env.sh python -m crackvision.prepare_inputs first"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "paths", "precondition", EXIT_PRECONDITION)
        return EXIT_PRECONDITION

    try:
        case_map = json.loads(case_map_path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        msg = f"malformed {case_map_path}: {exc}"
        logger.error(msg)
        summary.add_error(msg)
        summary.write(cfg, "paths", "failed", EXIT_RUNTIME)
        return EXIT_RUNTIME

    all_cases: list[dict[str, Any]] = case_map.get("cases", [])
    if args.cases:
        wanted = set(args.cases)
        cases = [c for c in all_cases if c["case_id"] in wanted]
        for name in sorted(wanted - {c["case_id"] for c in all_cases}):
            logger.error("requested case_id not found in case_map.json: %s", name)
            summary.add_error(f"unknown case_id: {name}")
            summary.increment("failed")
    else:
        cases = all_cases

    skeletons_dir = cfg.paths["skeletons"]
    paths_dir = cfg.root / "data" / "paths"

    if args.dry_run:
        for case_entry in cases:
            logger.info("dry-run: would extract paths for case %s", case_entry["case_id"])
        logger.info("dry-run: no files written")
        return EXIT_OK

    for case_entry in cases:
        result = _paths_case(
            cfg,
            case_entry["case_id"],
            min_spur_length_px=min_spur_length_px,
            rdp_tolerance_px=rdp_tolerance_px,
            skeletons_dir=skeletons_dir,
            paths_dir=paths_dir,
            skip_existing=args.skip_existing,
            logger=logger,
        )
        status = result["status"]
        if status == "failed":
            summary.increment("failed")
            summary.add_error(f"{result['case_id']}: {result.get('error', 'failed')}")
        elif status == "skipped":
            summary.increment("skipped")
        else:
            summary.increment("processed")

    failed = summary.counts.get("failed", 0)
    if failed:
        summary.write(cfg, "paths", "partial", EXIT_RUNTIME)
        return EXIT_RUNTIME

    summary.write(cfg, "paths", "ok", EXIT_OK)
    return EXIT_OK


if __name__ == "__main__":
    raise SystemExit(main())
