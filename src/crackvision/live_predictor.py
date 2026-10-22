"""crackvision.live_predictor — the OpenCrack nnU-Net kept in memory for per-frame prediction.

`crackvision.inference` (TC-008) is the validated batch path: it launches
`nnUNetv2_predict_from_modelfolder` as a subprocess, which reloads the network for every call (about
6.5 s per image on the RTX 3090, mostly start-up). A live camera view needs the network loaded once.
This module loads it in-process through nnU-Net's own `nnUNetPredictor`, with exactly the batch
path's model: the same model folder, fold and checkpoint from `config/project.yaml`, the same
`nnUNet_extTrainer` shim (`inference.prepare_ext_trainer_shim`), the same tile step and Gaussian
weighting, and the image read the way `NaturalImage2DIO` reads a PNG (channels first, spacing
(999, 1, 1)). tests/test_live_predictor.py checks it against the batch path on the same image.

Test-time augmentation (mirroring) is OFF by default here for frame rate; `tta=True` matches the
batch path's default exactly. Never trains anything (adr/010).
"""

from __future__ import annotations

import os
from typing import Any

import numpy as np

from crackvision.config import Config
from crackvision.inference import missing_model_files, model_folder, prepare_ext_trainer_shim


class LivePredictor:
    def __init__(self, cfg: Config, *, tta: bool = False, device: str | None = None) -> None:
        missing = missing_model_files(cfg)
        if missing:
            raise FileNotFoundError(f"model files missing: {[str(p) for p in missing]} (scripts/fetch_model.py)")
        os.environ["nnUNet_extTrainer"] = str(prepare_ext_trainer_shim(cfg))   # read at lookup time by nnU-Net
        import torch
        from nnunetv2.inference.predict_from_raw_data import nnUNetPredictor

        inf = cfg.inference
        dev = torch.device(device or inf.get("device", "cuda"))
        if dev.type == "cuda" and not torch.cuda.is_available():
            raise RuntimeError("CUDA is not available; pass device='cpu' (slow) or check nvidia-smi")
        self.tta = tta
        self.device = str(dev)
        self.predictor = nnUNetPredictor(
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
