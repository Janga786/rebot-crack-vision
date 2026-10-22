"""crackvision.live_predictor — the OpenCrack nnU-Net kept in memory for per-frame prediction.

`crackvision.inference` (TC-008) is the validated batch path: it launches
`nnUNetv2_predict_from_modelfolder` as a subprocess, which reloads the network for every call (about
6.5 s per image on the RTX 3090, mostly start-up). A live camera view needs the network loaded once.
This module loads it in-process through nnU-Net's own `nnUNetPredictor`, with exactly the batch
path's model: the same model folder, fold and checkpoint from `config/project.yaml`, the same
`nnUNet_extTrainer` shim (`inference.prepare_ext_trainer_shim`), the same tile step and Gaussian
weighting, and the image read the way `NaturalImage2DIO` reads a PNG (channels first, spacing
(999, 1, 1)). tests/test_camera_pipeline.py checks it against the batch path on the same image.

Two speed-ups over the stock predictor, neither changing what is computed (measured 2026-10-22 on the
D405's 848x480 frames: 204 ms -> about 45 ms per frame):
  - the sliding-window tiles go through the network as one batch instead of one at a time. The tiles,
    their order, the Gaussian weighting and the half-precision accumulation are nnU-Net's own; the
    network normalises per instance, so batching does not change a tile's output;
  - with a single fold, the weights are loaded once. The stock `predict_logits_from_preprocessed_data`
    reloads them on every call, for fold ensembles (about 68 ms per frame).

Test-time augmentation (mirroring) is OFF by default here for frame rate; `tta=True` matches the
batch path's default exactly. Never trains anything (adr/010).
"""

from __future__ import annotations

import os
from typing import Any

import numpy as np

from crackvision.config import Config
from crackvision.inference import missing_model_files, model_folder, prepare_ext_trainer_shim

TILE_BATCH = 32


def _fast_predictor_class():
    import torch
    from nnunetv2.inference.predict_from_raw_data import compute_gaussian, nnUNetPredictor

    class FastPredictor(nnUNetPredictor):
        tile_batch = TILE_BATCH
        _weights_loaded = False

        def predict_logits_from_preprocessed_data(self, data: torch.Tensor) -> torch.Tensor:
            if len(self.list_of_parameters) != 1:
                return super().predict_logits_from_preprocessed_data(data)
            if not self._weights_loaded:                 # the stock method reloads them on every call
                self.network.load_state_dict(self.list_of_parameters[0])
                self._weights_loaded = True
            return self.predict_sliding_window_return_logits(data).to("cpu")

        def _internal_predict_sliding_window_return_logits(self, data, slicers, do_on_device: bool = True):
            dev = self.device if do_on_device else torch.device("cpu")
            data = data.to(dev)
            logits = torch.zeros((self.label_manager.num_segmentation_heads, *data.shape[1:]), dtype=torch.half,
                                 device=dev)
            n_predictions = torch.zeros(data.shape[1:], dtype=torch.half, device=dev)
            gaussian = compute_gaussian(tuple(self.configuration_manager.patch_size), sigma_scale=1. / 8,
                                        value_scaling_factor=10, device=dev) if self.use_gaussian else 1
            for i in range(0, len(slicers), self.tile_batch):
                chunk = slicers[i:i + self.tile_batch]
                batch = torch.stack([data[s] for s in chunk]).to(self.device)
                prediction = self._internal_maybe_mirror_and_predict(batch).to(dev)
                for p, sl in zip(prediction, chunk):          # accumulate in nnU-Net's tile order
                    if self.use_gaussian:
                        p *= gaussian
                    logits[sl] += p
                    n_predictions[sl[1:]] += gaussian
            torch.div(logits, n_predictions, out=logits)
            if torch.any(torch.isinf(logits)):
                raise RuntimeError("inf in the predicted logits")
            return logits

    return FastPredictor


class LivePredictor:
    def __init__(self, cfg: Config, *, tta: bool = False, device: str | None = None) -> None:
        missing = missing_model_files(cfg)
        if missing:
            raise FileNotFoundError(f"model files missing: {[str(p) for p in missing]} (scripts/fetch_model.py)")
        os.environ["nnUNet_extTrainer"] = str(prepare_ext_trainer_shim(cfg))   # read at lookup time by nnU-Net
        import torch

        inf = cfg.inference
        dev = torch.device(device or inf.get("device", "cuda"))
        if dev.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available; pass device='cpu' (slow) or check nvidia-smi")
        self.tta = tta
        self.device = str(dev)
        self.predictor = _fast_predictor_class()(
            tile_step_size=float(inf.get("step_size", 0.5)),
            use_gaussian=True,
            use_mirroring=tta,
            perform_everything_on_device=dev.type == "cuda",
            device=dev,
            verbose=False,
            verbose_preprocessing=False,
            allow_tqdm=False,
        )
        self.predictor.initialize_from_trained_model_folder(
            str(model_folder(cfg)), use_folds=(int(cfg.model["fold"]),), checkpoint_name=str(cfg.model["checkpoint"]),
        )

    def predict(self, rgb: np.ndarray) -> np.ndarray:
        """uint8 RGB (H, W, 3) -> uint8 crack mask (H, W) with values {0, 1}, the batch path's label values."""
        if rgb.dtype != np.uint8 or rgb.ndim != 3 or rgb.shape[2] != 3:
            raise ValueError(f"expected uint8 (H, W, 3) RGB, got {rgb.dtype} {rgb.shape}")
        image = rgb.transpose(2, 0, 1)[:, None].astype(np.float32)      # as NaturalImage2DIO.read_images
        seg: Any = self.predictor.predict_single_npy_array(image, {"spacing": (999, 1, 1)}, None, None, False)
        return np.asarray(seg).reshape(rgb.shape[:2]).astype(np.uint8)
