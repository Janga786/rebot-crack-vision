# LICENSE-NOTICES.md — third-party attribution

This project consumes third-party model weights and open-source libraries. Their licences are
independent of, and not changed by, this project's own licence (below). If you redistribute
anything derived from this repository — including generated masks/overlays that embed the model's
output — carry these notices with it.

## Model weights

### OpenCrack nnU-Net — `fadeevla/opencrack-nnunet`

- **Licence:** CC-BY-4.0 (attribution required).
- **Source:** https://huggingface.co/fadeevla/opencrack-nnunet
- **Pinned revision:** `1198179e893f5f6eb0dd3eae2d8de5f1adf85afc` (see
  `config/model_manifest.json` for the exact per-file sha256 pins).
- **Checkpoint:** `Dataset501_OpenCrack/nnUNetTrainer__nnUNetPlans__2d/fold_0/checkpoint_ep0500.pth`,
  sha256 `dcba82012874682c84387c68a2b69f64eddef07e84ae7d813844cdf6396a1a5f`.
- The model card states: *"Model weights and this card are released under CC-BY-4.0. This does not
  relicense the source images or annotations used to build OpenCrack; each source dataset remains
  under its own license."*

Citation (from the model card):

```bibtex
@misc{fadeev2026opencrack,
  author       = {Fadeev, V. A.},
  title        = {OpenCrack: A Consolidated, Leakage-Controlled Benchmark for Pavement Crack Segmentation},
  year         = {2026},
  howpublished = {\url{https://github.com/fadeevla/OpenCrack}}
}
```

The model card also asks that, if you use the model, you credit the OpenCrack source datasets
listed on the benchmark page (https://github.com/fadeevla/OpenCrack) — this project does not
redistribute those source datasets and has not needed to enumerate them.

### nnU-Net v2

- **Licence:** Apache-2.0.
- **Source:** https://github.com/MIC-DKFZ/nnUNet
- The OpenCrack model is built on nnU-Net's self-configuring pipeline; nnU-Net itself is a separate,
  Apache-2.0-licensed dependency (`nnunetv2` on PyPI), not redistributed by this repository.

```bibtex
@article{isensee2021nnunet,
  author  = {Isensee, Fabian and Jaeger, Paul F. and Kohl, Simon A. A. and Petersen, Jens and Maier-Klein, Klaus H.},
  title   = {nnU-Net: a self-configuring method for deep learning-based biomedical image segmentation},
  journal = {Nature Methods},
  volume  = {18},
  number  = {2},
  pages   = {203--211},
  year    = {2021},
  doi     = {10.1038/s41592-020-01008-z}
}
```

## Third-party libraries (brief notes)

None of the following are redistributed by this repository — they are installed as ordinary Python
dependencies (`requirements/`) and are named here only for attribution completeness.

| Library | Licence | Role in this project |
|---|---|---|
| PyTorch | BSD-3-Clause | Tensor/autograd backend nnU-Net runs on. |
| scikit-image | BSD-3-Clause | `remove_small_objects` + `skeletonize` (`crackvision.skeleton`). |
| Pillow | MIT-CMU | Image I/O and EXIF handling (`crackvision.prepare_inputs`, `.visualize`). |
| OpenCV (`opencv-python-headless`) | Apache-2.0 | Auxiliary image processing. |
| pyrealsense2 | Apache-2.0 | Intel RealSense D405 SDK bindings (`crackvision.realsense_capture`). |

## This project's own licence

**Not yet chosen — capstone coursework.** No `LICENSE` file has been added to this repository as of
this writing; do not assume a default open-source licence applies. The notices above govern only
the third-party components they name.
