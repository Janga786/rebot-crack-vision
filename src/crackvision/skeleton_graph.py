"""crackvision.skeleton_graph — 8-connected pixel graph over a 1-px skeleton. Built by PERC-02.

Consumes the `{0,255}` raster written by `crackvision.skeleton` (TC-010) and represents it as a
graph of junction/endpoint/isolated nodes joined by edges that carry the full ordered pixel path
between them, in the original `(row, col)` frame (docs/INTERFACES.md §0.5). Nothing here resizes,
crops or otherwise touches pixel coordinates.
"""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from typing import Any

import numpy as np

DEFAULT_MIN_SPUR_LENGTH_PX = 5

# 8-connected offsets, row-major order for deterministic neighbor iteration.
_NEIGHBOR_OFFSETS = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]

Pixel = tuple[int, int]


@dataclass(frozen=True)
class Node:
    """A graph vertex: a skeleton pixel that is a junction, endpoint or isolated point."""

    id: int
    row: int
    col: int
    kind: str  # "endpoint" | "junction" | "loop" | "isolated"


@dataclass(frozen=True)
class Edge:
    """A graph edge: the ordered pixel path between two nodes, both endpoints included."""

    id: int
    node_a: int
    node_b: int
    pixels: tuple[Pixel, ...]

    @property
    def length_px(self) -> int:
        """Number of steps along the path (`len(pixels) - 1`)."""
        return len(self.pixels) - 1


@dataclass
class SkeletonGraph:
    """An 8-connected pixel graph: nodes keyed by id, edges keyed by id."""

    nodes: dict[int, Node] = field(default_factory=dict)
    edges: dict[int, Edge] = field(default_factory=dict)

    def node_edges(self, node_id: int) -> list[Edge]:
        return [e for e in self.edges.values() if e.node_a == node_id or e.node_b == node_id]


def _skeleton_pixels(skeleton: np.ndarray) -> set[Pixel]:
    rows, cols = np.nonzero(skeleton)
    return {(int(r), int(c)) for r, c in zip(rows, cols)}


def _neighbors(pixel: Pixel, skel_pixels: set[Pixel]) -> list[Pixel]:
    r, c = pixel
    out = []
    for dr, dc in _NEIGHBOR_OFFSETS:
        n = (r + dr, c + dc)
        if n in skel_pixels:
            out.append(n)
    return out


def degree_map(skeleton: np.ndarray) -> dict[Pixel, int]:
    """8-connected neighbor count for every `True` pixel of `skeleton`."""
    skel_pixels = _skeleton_pixels(skeleton)
    return {p: len(_neighbors(p, skel_pixels)) for p in skel_pixels}


def _connected_components(pixels: set[Pixel], skel_pixels: set[Pixel]) -> list[set[Pixel]]:
    """8-connected components of `pixels`, restricted to stepping through `pixels` only."""
    components: list[set[Pixel]] = []
    seen: set[Pixel] = set()
    for start in sorted(pixels):
        if start in seen:
            continue
        component: set[Pixel] = set()
        stack = [start]
        while stack:
            p = stack.pop()
            if p in component:
                continue
            component.add(p)
            for n in _neighbors(p, skel_pixels):
                if n in pixels and n not in component:
                    stack.append(n)
        seen |= component
        components.append(component)
    return components


def build_graph(skeleton: np.ndarray) -> SkeletonGraph:
    """Build the 8-connected pixel graph of a `{0,255}`/bool skeleton raster.

    Node pixels are skeleton pixels whose 8-connected degree is 0 (isolated), 1 (endpoint) or
    >= 3 (junction). Mutually-adjacent junction pixels (a real branch point is frequently more
    than one pixel wide under 8-connectivity, e.g. a stem meeting a straight bar) are merged into
    a single "junction" node at their lexicographically-smallest pixel. A connected component made
    entirely of degree-2 pixels is a pure loop with no natural branch point; its
    lexicographically-smallest pixel is likewise promoted to a "loop" node so the cycle is still
    representable as a (self-)edge.
    """
    skel_pixels = _skeleton_pixels(skeleton)
    degrees = {p: len(_neighbors(p, skel_pixels)) for p in skel_pixels}

    raw_nodes: set[Pixel] = {p for p, d in degrees.items() if d != 2}

    # Pure loops: connected components with no natural node pixel at all.
    remaining = set(skel_pixels) - raw_nodes
    visited_for_loops: set[Pixel] = set()
    for start in sorted(remaining):
        if start in visited_for_loops or start in raw_nodes:
            continue
        component: set[Pixel] = set()
        stack = [start]
        while stack:
            p = stack.pop()
            if p in component:
                continue
            component.add(p)
            for n in _neighbors(p, skel_pixels):
                if n not in component and n not in raw_nodes:
                    stack.append(n)
        visited_for_loops |= component
        touches_a_real_node = any(n in raw_nodes for p in component for n in _neighbors(p, skel_pixels))
        if touches_a_real_node:
            continue  # an open chain between two real nodes, not a closed cycle
        raw_nodes.add(min(component))

    # Merge mutually-adjacent junction pixels (degree >= 3) into a single node each.
    junction_pixels = {p for p in raw_nodes if degrees.get(p, 2) >= 3}
    cluster_rep: dict[Pixel, Pixel] = {}
    for component in _connected_components(junction_pixels, skel_pixels):
        rep = min(component)
        for p in component:
            cluster_rep[p] = rep

    def rep_of(p: Pixel) -> Pixel:
        return cluster_rep.get(p, p)

    sorted_node_reps = sorted({rep_of(p) for p in raw_nodes})
    pixel_to_id = {p: i for i, p in enumerate(sorted_node_reps)}

    nodes: dict[int, Node] = {}
    for rep in sorted_node_reps:
        if rep in junction_pixels:
            kind = "junction"
        else:
            d = degrees[rep]
            kind = "isolated" if d == 0 else "endpoint" if d == 1 else "loop"
        node_id = pixel_to_id[rep]
        nodes[node_id] = Node(id=node_id, row=rep[0], col=rep[1], kind=kind)

    edges: dict[int, Edge] = {}
    visited_directions: set[tuple[Pixel, Pixel]] = set()
    next_edge_id = 0
    for start in sorted(raw_nodes):
        for neighbor in sorted(_neighbors(start, skel_pixels)):
            if (start, neighbor) in visited_directions:
                continue
            if neighbor in raw_nodes and start != neighbor and rep_of(start) == rep_of(neighbor):
                # both pixels belong to the same merged junction cluster: no edge, just consumed.
                visited_directions.add((start, neighbor))
                visited_directions.add((neighbor, start))
                continue
            path = [start]
            prev, curr = start, neighbor
            visited_directions.add((prev, curr))
            while curr not in raw_nodes:
                path.append(curr)
                nxts = [n for n in _neighbors(curr, skel_pixels) if n != prev]
                nxt = nxts[0]
                visited_directions.add((curr, nxt))
                prev, curr = curr, nxt
            path.append(curr)
            visited_directions.add((curr, prev))

            edges[next_edge_id] = Edge(
                id=next_edge_id,
                node_a=pixel_to_id[rep_of(start)],
                node_b=pixel_to_id[rep_of(curr)],
                pixels=tuple(path),
            )
            next_edge_id += 1

    return SkeletonGraph(nodes=nodes, edges=edges)


def prune_spurs(graph: SkeletonGraph, min_spur_length_px: int) -> SkeletonGraph:
    """Drop endpoint spurs shorter than `min_spur_length_px`, keeping every component's main path.

    A spur is an edge whose two endpoints are an "endpoint" node and a "junction"/"loop" node
    (i.e. a leaf hanging off a branch point). Edges between two endpoint nodes are a component's
    only path and are never removed, regardless of length. If every edge incident to a junction is
    a short spur (no non-spur connection keeps the junction attached to the rest of the graph),
    the single longest of those spurs is kept so the component's main path always survives.
    """
    if min_spur_length_px <= 0:
        return SkeletonGraph(nodes=dict(graph.nodes), edges=dict(graph.edges))

    candidates: dict[int, list[int]] = {}  # junction/loop node id -> candidate spur edge ids
    for edge_id, edge in graph.edges.items():
        node_a = graph.nodes[edge.node_a]
        node_b = graph.nodes[edge.node_b]
        kinds = {node_a.kind, node_b.kind}
        is_endpoint_pair = kinds == {"endpoint"}
        touches_endpoint = "endpoint" in kinds
        if touches_endpoint and not is_endpoint_pair and edge.length_px < min_spur_length_px:
            branch_node = node_a if node_a.kind != "endpoint" else node_b
            candidates.setdefault(branch_node.id, []).append(edge_id)

    removed_edge_ids: set[int] = set()
    for branch_node_id, edge_ids in candidates.items():
        incident_count = len(graph.node_edges(branch_node_id))
        if len(edge_ids) == incident_count:
            # every edge at this branch point is a short spur: keep the longest as the main path.
            edge_ids = sorted(edge_ids, key=lambda eid: graph.edges[eid].length_px, reverse=True)[1:]
        removed_edge_ids.update(edge_ids)

    kept_edges = {edge_id: edge for edge_id, edge in graph.edges.items() if edge_id not in removed_edge_ids}
    kept_node_ids = {edge.node_a for edge in kept_edges.values()} | {edge.node_b for edge in kept_edges.values()}
    kept_nodes = {
        node_id: node
        for node_id, node in graph.nodes.items()
        if node_id in kept_node_ids or node.kind == "isolated"
    }
    return SkeletonGraph(nodes=kept_nodes, edges=kept_edges)


def to_dict(graph: SkeletonGraph) -> dict[str, Any]:
    """Deterministic plain-dict representation: nodes and edges sorted by id."""
    return {
        "schema_version": 1,
        "nodes": [
            {"id": n.id, "row": n.row, "col": n.col, "kind": n.kind}
            for n in sorted(graph.nodes.values(), key=lambda n: n.id)
        ],
        "edges": [
            {
                "id": e.id,
                "node_a": e.node_a,
                "node_b": e.node_b,
                "length_px": e.length_px,
                "pixels": [[r, c] for r, c in e.pixels],
            }
            for e in sorted(graph.edges.values(), key=lambda e: e.id)
        ],
    }


def to_json(graph: SkeletonGraph, *, indent: int | None = 2) -> str:
    """`json.dumps(to_dict(graph))` with deterministic key/element ordering."""
    return json.dumps(to_dict(graph), indent=indent)


def spur_threshold_from_config(cfg: Any) -> int:
    """`cfg.skeleton['min_spur_length_px']`, else `DEFAULT_MIN_SPUR_LENGTH_PX`.

    `cfg` is a `crackvision.config.Config`; kept duck-typed so tests can pass a plain namespace.
    """
    skeleton_cfg = getattr(cfg, "skeleton", {}) or {}
    return int(skeleton_cfg.get("min_spur_length_px", DEFAULT_MIN_SPUR_LENGTH_PX))
