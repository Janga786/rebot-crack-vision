"""tests/test_skeleton_graph.py — pixel-graph construction and spur pruning. Built by PERC-02.

Synthetic {0,255} skeleton rasters only, no model/skeletonize call needed.
"""

from __future__ import annotations

import json

import numpy as np

from crackvision.skeleton_graph import (
    DEFAULT_MIN_SPUR_LENGTH_PX,
    SkeletonGraph,
    build_graph,
    degree_map,
    prune_spurs,
    spur_threshold_from_config,
    to_dict,
    to_json,
)


def _canvas(height: int, width: int) -> np.ndarray:
    return np.zeros((height, width), dtype=bool)


def _draw(canvas: np.ndarray, pixels: list[tuple[int, int]]) -> np.ndarray:
    for r, c in pixels:
        canvas[r, c] = True
    return canvas


def _kinds_by_pixel(graph: SkeletonGraph) -> dict[tuple[int, int], str]:
    return {(n.row, n.col): n.kind for n in graph.nodes.values()}


# ---------------------------------------------------------------------------
# Frame invariant / basic shape
# ---------------------------------------------------------------------------


def test_horizontal_line_two_endpoints_no_junctions():
    skel = _canvas(5, 10)
    _draw(skel, [(2, c) for c in range(2, 8)])

    graph = build_graph(skel)

    kinds = sorted(n.kind for n in graph.nodes.values())
    assert kinds == ["endpoint", "endpoint"]
    assert len(graph.edges) == 1
    (edge,) = graph.edges.values()
    assert edge.pixels[0] == (2, 2)
    assert edge.pixels[-1] == (2, 7)
    assert edge.length_px == 5


def test_node_coordinates_are_not_resized():
    height, width = 300, 400
    skel = _canvas(height, width)
    _draw(skel, [(150, c) for c in range(50, 350)])

    graph = build_graph(skel)

    for node in graph.nodes.values():
        assert 0 <= node.row < height
        assert 0 <= node.col < width
    endpoint_cols = sorted(n.col for n in graph.nodes.values())
    assert endpoint_cols == [50, 349]


def test_diagonal_line_is_8_connected_single_component():
    skel = _canvas(6, 6)
    _draw(skel, [(i, i) for i in range(1, 5)])

    graph = build_graph(skel)

    assert len(graph.nodes) == 2
    assert len(graph.edges) == 1
    assert all(n.kind == "endpoint" for n in graph.nodes.values())


# ---------------------------------------------------------------------------
# Junction / endpoint classification on synthetic Y, X, T, loop, spur shapes
# ---------------------------------------------------------------------------


def test_y_shape_one_junction_three_endpoints():
    # A "Mercedes" Y: three arms 120-ish degrees apart (N, SW, SE) that stay clear of each
    # other's 8-neighbourhood everywhere except the shared centre pixel.
    skel = _canvas(11, 11)
    center = (5, 5)
    skel[center] = True
    _draw(skel, [(4, 5), (3, 5), (2, 5), (1, 5)])  # north arm
    _draw(skel, [(6, 4), (7, 3), (8, 2)])  # south-west arm
    _draw(skel, [(6, 6), (7, 7), (8, 8)])  # south-east arm

    graph = build_graph(skel)
    kinds = [n.kind for n in graph.nodes.values()]
    assert kinds.count("junction") == 1
    assert kinds.count("endpoint") == 3
    junction = next(n for n in graph.nodes.values() if n.kind == "junction")
    assert (junction.row, junction.col) == center
    assert len(graph.edges) == 3


def test_x_shape_one_junction_four_endpoints():
    skel = _canvas(11, 11)
    center = (5, 5)
    for i in range(1, 5):
        skel[5 - i, 5 - i] = True
        skel[5 - i, 5 + i] = True
        skel[5 + i, 5 - i] = True
        skel[5 + i, 5 + i] = True
    skel[center] = True

    graph = build_graph(skel)
    kinds = [n.kind for n in graph.nodes.values()]
    assert kinds.count("junction") == 1
    assert kinds.count("endpoint") == 4
    assert len(graph.edges) == 4


def test_t_shape_one_junction_three_endpoints():
    # An axis-aligned T: the stem's tip pixel is 8-adjacent to three bar pixels at once, so the
    # real branch point is a small merged cluster of junction pixels, not a single one — the same
    # thing skimage.morphology.skeletonize can produce at a true T-intersection. build_graph must
    # still classify it as exactly one junction node with three legitimate endpoint arms.
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])  # horizontal bar, row 6
    _draw(skel, [(r, 6) for r in range(1, 6)])  # stem col 6, touching the bar at (6, 6)

    graph = build_graph(skel)
    kinds = [n.kind for n in graph.nodes.values()]
    assert kinds.count("junction") == 1
    assert kinds.count("endpoint") == 3
    assert len(graph.edges) == 3
    for edge in graph.edges.values():
        assert edge.length_px >= 3


def test_loop_shape_single_self_loop_edge_no_endpoints():
    # A diamond (not a square): 90-degree corners under 8-connectivity make the two pixels either
    # side of a corner diagonal neighbours of each other too, contaminating them into a junction
    # cluster (same artifact as the T-shape). A diagonal-only diamond stays a genuine 1-px cycle.
    skel = _canvas(9, 9)
    ring = [
        (1, 4), (2, 5), (3, 6), (4, 7),
        (5, 6), (6, 5), (7, 4),
        (6, 3), (5, 2), (4, 1),
        (3, 2), (2, 3),
    ]
    _draw(skel, ring)

    graph = build_graph(skel)

    assert len(graph.nodes) == 1
    (node,) = graph.nodes.values()
    assert node.kind == "loop"
    assert len(graph.edges) == 1
    (edge,) = graph.edges.values()
    assert edge.node_a == edge.node_b == node.id
    assert edge.length_px == len(ring)
    assert set(edge.pixels[:-1]) == set(ring)


def test_spur_shape_junction_with_short_branch():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])  # main bar, both arms long
    _draw(skel, [(4, 6), (5, 6)])  # a short (2-px) spur off the bar

    graph = build_graph(skel)
    kinds = [n.kind for n in graph.nodes.values()]
    assert kinds.count("junction") == 1
    assert kinds.count("endpoint") == 3
    lengths = sorted(e.length_px for e in graph.edges.values())
    assert lengths[0] == 1  # the spur, shortest edge in the graph


def test_isolated_single_pixel_component():
    skel = _canvas(5, 5)
    skel[2, 2] = True

    graph = build_graph(skel)

    assert len(graph.nodes) == 1
    (node,) = graph.nodes.values()
    assert node.kind == "isolated"
    assert len(graph.edges) == 0


def test_multi_component_skeleton_classified_independently():
    skel = _canvas(12, 20)
    _draw(skel, [(2, c) for c in range(1, 6)])  # component A: a line, 2 endpoints
    _draw(skel, [(7, c) for c in range(11, 16)])  # component B: a T, bar cols 11-15
    _draw(skel, [(r, 13) for r in range(8, 10)])  # stem down from the bar's middle

    graph = build_graph(skel)

    kinds = [n.kind for n in graph.nodes.values()]
    assert kinds.count("junction") == 1
    assert kinds.count("endpoint") == 5  # 2 from the line + 3 from the T
    assert len(graph.edges) == 4  # 1 from the line + 3 from the T

    # the two components must not share any edge (no cross-component node pair).
    line_ids = {n.id for n in graph.nodes.values() if n.row == 2}
    t_ids = {n.id for n in graph.nodes.values() if n.row != 2}
    for edge in graph.edges.values():
        endpoints = {edge.node_a, edge.node_b}
        assert endpoints <= line_ids or endpoints <= t_ids


def test_empty_skeleton_has_no_nodes_or_edges():
    skel = _canvas(5, 5)
    graph = build_graph(skel)
    assert graph.nodes == {}
    assert graph.edges == {}


def test_degree_map_matches_kind_classification():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])
    _draw(skel, [(4, 6), (5, 6)])

    degrees = degree_map(skel)
    graph = build_graph(skel)
    for node in graph.nodes.values():
        d = degrees[(node.row, node.col)]
        if node.kind == "endpoint":
            assert d == 1
        elif node.kind == "junction":
            assert d >= 3


# ---------------------------------------------------------------------------
# Spur pruning
# ---------------------------------------------------------------------------


def test_prune_removes_short_spur_keeps_main_bar():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])  # both bar arms long (length 4 and 3)
    _draw(skel, [(4, 6), (5, 6)])  # length-1 spur (edge length_px, after cluster absorption)

    graph = build_graph(skel)
    pruned = prune_spurs(graph, min_spur_length_px=3)

    kinds = sorted(n.kind for n in pruned.nodes.values())
    assert kinds == ["endpoint", "endpoint", "junction"]
    assert len(pruned.edges) == 2
    for edge in pruned.edges.values():
        assert edge.length_px >= 3


def test_prune_keeps_spur_at_or_above_threshold():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])
    _draw(skel, [(2, 6), (3, 6), (4, 6), (5, 6)])  # length-3 spur (edge length_px)

    graph = build_graph(skel)
    pruned = prune_spurs(graph, min_spur_length_px=3)

    assert len(pruned.edges) == 3
    kinds = sorted(n.kind for n in pruned.nodes.values())
    assert kinds == ["endpoint", "endpoint", "endpoint", "junction"]


def test_prune_never_removes_a_two_endpoint_components_main_path():
    skel = _canvas(5, 10)
    _draw(skel, [(2, c) for c in range(2, 5)])  # a short, 2-px line: no junction at all

    graph = build_graph(skel)
    pruned = prune_spurs(graph, min_spur_length_px=1000)

    assert len(pruned.edges) == 1
    assert len(pruned.nodes) == 2


def test_prune_keeps_longest_leg_when_every_branch_is_a_short_spur():
    skel = _canvas(11, 11)
    center = (5, 5)
    skel[center] = True
    skel[4, 5] = True  # length-1 leg north
    skel[6, 4] = True  # length-1 leg south-west
    _draw(skel, [(6, 6), (7, 7), (8, 8)])  # length-3 leg south-east (the longest)

    graph = build_graph(skel)
    pruned = prune_spurs(graph, min_spur_length_px=100)

    junction = next(n for n in pruned.nodes.values() if n.kind == "junction")
    assert (junction.row, junction.col) == center
    assert len(pruned.edges) == 1
    (kept_edge,) = pruned.edges.values()
    assert kept_edge.length_px == 3


def test_prune_zero_threshold_is_a_no_op():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])
    _draw(skel, [(4, 6), (5, 6)])

    graph = build_graph(skel)
    pruned = prune_spurs(graph, min_spur_length_px=0)

    assert len(pruned.nodes) == len(graph.nodes)
    assert len(pruned.edges) == len(graph.edges)


def test_spur_threshold_from_config_default_and_override():
    class Cfg:
        skeleton = {}

    assert spur_threshold_from_config(Cfg()) == DEFAULT_MIN_SPUR_LENGTH_PX

    Cfg.skeleton = {"min_spur_length_px": 9}
    assert spur_threshold_from_config(Cfg()) == 9


# ---------------------------------------------------------------------------
# JSON serialisation
# ---------------------------------------------------------------------------


def test_to_dict_and_to_json_round_trip_and_are_deterministic():
    skel = _canvas(12, 15)
    _draw(skel, [(6, c) for c in range(1, 11)])
    _draw(skel, [(4, 6), (5, 6)])

    graph = build_graph(skel)

    d1 = to_dict(graph)
    d2 = to_dict(graph)
    assert d1 == d2

    payload = json.loads(to_json(graph))
    assert payload == d1
    assert payload["schema_version"] == 1
    assert {n["kind"] for n in payload["nodes"]} == {"endpoint", "junction"}
    for edge in payload["edges"]:
        assert isinstance(edge["pixels"], list)
        assert all(len(p) == 2 for p in edge["pixels"])
        assert edge["length_px"] == len(edge["pixels"]) - 1

    node_ids = [n["id"] for n in payload["nodes"]]
    assert node_ids == sorted(node_ids)
    edge_ids = [e["id"] for e in payload["edges"]]
    assert edge_ids == sorted(edge_ids)


def test_to_json_is_deterministic_across_rebuilds():
    skel = _canvas(11, 11)
    _draw(skel, [(5, c) for c in range(1, 6)])
    _draw(skel, [(r, 5) for r in range(1, 6)])
    _draw(skel, [(r, r) for r in range(5, 9)])
    skel[5, 5] = True

    graph_a = build_graph(skel.copy())
    graph_b = build_graph(skel.copy())

    assert to_json(graph_a) == to_json(graph_b)


def test_empty_graph_serialises_cleanly():
    graph = SkeletonGraph()
    assert to_dict(graph) == {"schema_version": 1, "nodes": [], "edges": []}
    assert json.loads(to_json(graph))["nodes"] == []
