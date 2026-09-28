# ADR-002: RGB-only into the network; D405 depth is consumed after segmentation

**Status:** accepted · **Date:** 2026-09-16
Status: Superseded in part by ADR-011 (2026-09-28)

## Context
The D405 gives synchronised colour and depth. A naive design would feed RGB-D to the segmentation
network. But `Dataset501_OpenCrack/.../plans.json` declares exactly **3 input channels** with
`dataset.json` `channel_names {"0":"R","1":"G","2":"B"}` and `NaturalImage2DIO`. A 4-channel input is
architecturally impossible without retraining the first convolution.

Separately, depth is physically weak exactly where a crack is: a crack is a narrow cavity, often
narrower than the stereo correlation window, so the D405 usually returns the *surface* depth across
it (or a hole), not the crack floor.

## Decision
The neural network sees **3-channel 8-bit RGB and nothing else**. Depth is captured and preserved
alongside, but is consumed only **after** segmentation — to lift the 2D centerline into 3D by
sampling the aligned depth frame at the skeleton's pixel coordinates.

## Consequences
+ The released checkpoint is usable as-is, zero-shot, with no architecture surgery.
+ Depth quality problems cannot corrupt the segmentation.
+ Depth capture can proceed in parallel with, and independently of, segmentation work.
− Depth carries no information into detection, so purely geometric cracks with no photometric
  contrast will be missed.
− Imposes the **frame invariant**: mask, skeleton and aligned depth must share one pixel grid, so no
  component in this phase may resize, crop or pad. (`INTERFACES.md` §0.5)
− We must align depth→colour at capture time (`rs.align(rs.stream.color)`), not later.

## Revisit when
3D lifting shows depth is systematically unusable at the crack, in which case the trajectory stage
should fit a local surface plane and project onto it rather than sample per-pixel depth.
