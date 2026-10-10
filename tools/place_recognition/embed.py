#!/usr/bin/env python3
"""Global image descriptors for the place-recognition benchmark.

    embed.py --set prset/ --out emb/ --model ijepa_h14_1k [--height 224]

Each frame of prset/<SEQ>.npz (grey cam0 image, replicated to RGB) is resized to the given
height keeping the EuRoC aspect ratio (width rounded to the patch size) and encoded. Descriptors
are L2-normalised and saved as emb/<model>/<SEQ>.npz with one array per pooling:
  mean : mean of the last-layer patch tokens (all models)
  cls  : the class token (DINOv2 only; I-JEPA and V-JEPA 2 have none)
  mean_L<k> : mean of the patch tokens after transformer block k (with --layers)
"""
import argparse
from pathlib import Path

import numpy as np
import torch
from PIL import Image

MODELS = {
    # name: (hub id, family, patch size)
    "ijepa_h14_1k": ("facebook/ijepa_vith14_1k", "ijepa", 14),
    "ijepa_h14_22k": ("facebook/ijepa_vith14_22k", "ijepa", 14),
    "ijepa_g16_22k": ("facebook/ijepa_vitg16_22k", "ijepa", 16),
    "vjepa2_l": ("facebook/vjepa2-vitl-fpc64-256", "vjepa2", 16),
    "dinov2_b": ("facebook/dinov2-base", "dinov2", 14),
    "dinov2_l": ("facebook/dinov2-large", "dinov2", 14),
}
MEAN = torch.tensor([0.485, 0.456, 0.406]).view(3, 1, 1)
STD = torch.tensor([0.229, 0.224, 0.225]).view(3, 1, 1)


def load(name: str):
    hub, family, patch = MODELS[name]
    if family == "ijepa":
        from transformers import IJepaModel as M
    elif family == "vjepa2":
        from transformers import VJEPA2Model as M
    else:
        from transformers import Dinov2Model as M
    model = M.from_pretrained(hub, torch_dtype=torch.float16).cuda().eval()
    return model, family, patch


def preprocess(path: str, height: int, patch: int) -> torch.Tensor:
    img = Image.open(path).convert("RGB")
    w = int(round(img.width * height / img.height / patch)) * patch
    img = img.resize((w, height), Image.BICUBIC)
    x = torch.from_numpy(np.asarray(img).copy()).permute(2, 0, 1).float() / 255.0
    return (x - MEAN) / STD


@torch.no_grad()
def encode(model, family: str, batch: torch.Tensor, layers=()) -> dict:
    x = batch.cuda().half()
    hidden = bool(layers)
    if family == "ijepa":
        res = model(pixel_values=x, interpolate_pos_encoding=True, output_hidden_states=hidden)
        first = 0
        out = {"mean": res.last_hidden_state.mean(1)}
    elif family == "vjepa2":
        # a still image as a 2-frame clip (one tubelet)
        clip = x.unsqueeze(1).repeat(1, 2, 1, 1, 1)
        res = model(pixel_values_videos=clip, skip_predictor=True, output_hidden_states=hidden)
        first = 0
        out = {"mean": res.last_hidden_state.mean(1)}
    else:
        res = model(pixel_values=x, output_hidden_states=hidden)
        first = 1  # skip the class token
        tokens = res.last_hidden_state
        out = {"cls": tokens[:, 0], "mean": tokens[:, 1:].mean(1)}
    for k in layers:
        # hidden_states[0] is the patch embedding, [k] the output of block k
        out[f"mean_L{k}"] = res.hidden_states[k][:, first:].mean(1)
    return {k: torch.nn.functional.normalize(v.float(), dim=1).cpu().numpy() for k, v in out.items()}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--model", required=True, choices=sorted(MODELS))
    ap.add_argument("--height", type=int, default=224)
    ap.add_argument("--batch", type=int, default=16)
    ap.add_argument("--layers", default="", help="comma-separated transformer blocks to pool, e.g. 8,16,24")
    a = ap.parse_args()
    model, family, patch = load(a.model)
    out = Path(a.out) / a.model
    out.mkdir(parents=True, exist_ok=True)
    for f in sorted(Path(a.set).glob("*.npz")):
        images = np.load(f)["images"]
        feats = {}
        for i in range(0, len(images), a.batch):
            batch = torch.stack([preprocess(p, a.height, patch) for p in images[i:i + a.batch]])
            layers = [int(k) for k in a.layers.split(",") if k]
            for k, v in encode(model, family, batch, layers).items():
                feats.setdefault(k, []).append(v)
        feats = {k: np.concatenate(v) for k, v in feats.items()}
        np.savez(out / f.name, **feats)
        print(f"{a.model} {f.stem}: {len(images)} x {feats['mean'].shape[1]}")


if __name__ == "__main__":
    main()
