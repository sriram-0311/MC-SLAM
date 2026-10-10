#!/usr/bin/env python3
"""Evaluate place-recognition descriptors on the EuRoC benchmark set.

    evaluate.py --set prset/ --emb emb/ [--dist 1.5 --angle 30 --gap 15] METHOD ...

METHOD is "dbow2" (emb/dbow2/<SEQ>.txt from bow_vectors) or "<model>:<pooling>" for
emb/<model>/<SEQ>.npz (cosine similarity).

Two tasks:
  loop  : each frame queries the frames of the same sequence at least --gap seconds earlier
  reloc : each frame queries every other sequence recorded in the same environment
Same place = camera centres within --dist metres and optical axes within --angle degrees.

Metrics over all queries:
  R@1, R@5 : top-1 / any of top-5 correct, over queries that have a true match in the database
  AP       : average precision of accepting the top-1 match, thresholding its score; accepting a
             match for a query without a true match counts as a false positive (as in SLAM)
  R@P100   : recall at 100 % precision, the share of revisits found with zero false loop closures
"""
import argparse
import json
from collections import defaultdict
from pathlib import Path

import numpy as np

ENVS = {"MH": ["MH_01_easy", "MH_02_easy", "MH_03_medium", "MH_04_difficult", "MH_05_difficult"],
        "V1": ["V1_01_easy", "V1_02_medium", "V1_03_difficult"],
        "V2": ["V2_01_easy", "V2_02_medium", "V2_03_difficult"]}


def load_bow(path: Path):
    vecs = []
    for line in path.read_text().splitlines():
        parts = line.split()[1:]
        vecs.append({int(w): float(v) for w, v in (p.split(":") for p in parts)})
    return vecs


def bow_similarity(A, B) -> np.ndarray:
    """DBoW2 L1 score of L1-normalised TF-IDF vectors = sum of element-wise minima."""
    index = defaultdict(list)
    for j, b in enumerate(B):
        for w, v in b.items():
            index[w].append((j, v))
    S = np.zeros((len(A), len(B)))
    for i, a in enumerate(A):
        row = S[i]
        for w, va in a.items():
            for j, vb in index.get(w, ()):
                row[j] += min(va, vb)
    return S


class Descriptors:
    def __init__(self, emb_dir: Path, method: str):
        self.emb_dir, self.method, self.cache = emb_dir, method, {}

    def get(self, seq: str):
        if seq not in self.cache:
            if self.method == "dbow2":
                self.cache[seq] = load_bow(self.emb_dir / "dbow2" / f"{seq}.txt")
            else:
                model, pooling = self.method.split(":")
                self.cache[seq] = np.load(self.emb_dir / model / f"{seq}.npz")[pooling]
        return self.cache[seq]

    def similarity(self, q: str, d: str) -> np.ndarray:
        A, B = self.get(q), self.get(d)
        return bow_similarity(A, B) if self.method == "dbow2" else A @ B.T


def positives(Pq, Fq, Pd, Fd, dist, angle) -> np.ndarray:
    D = np.linalg.norm(Pq[:, None, :] - Pd[None, :, :], axis=2)
    C = np.clip(Fq @ Fd.T, -1.0, 1.0)
    return (D < dist) & (C > np.cos(np.radians(angle)))


def query_results(S, P, mask):
    """Per query: (top-1 score, top-1 correct, any of top-5 correct, has a true match)."""
    out = []
    for i in range(S.shape[0]):
        cand = np.flatnonzero(mask[i])
        if cand.size == 0:
            continue
        order = cand[np.argsort(-S[i, cand])]
        out.append((S[i, order[0]], P[i, order[0]], P[i, order[:5]].any(), P[i, cand].any()))
    return out


def metrics(results) -> dict:
    r = np.array(results, dtype=float)
    score, top1, top5, valid = r[:, 0], r[:, 1] > 0, r[:, 2] > 0, r[:, 3] > 0
    n_valid = max(valid.sum(), 1)
    order = np.argsort(-score)
    tp = np.cumsum(top1[order])
    fp = np.cumsum(~top1[order])
    precision, recall = tp / (tp + fp), tp / n_valid
    ap = float(np.sum(np.diff(np.concatenate([[0.0], recall])) * precision))
    perfect = precision >= 1.0 - 1e-12
    r_at_p100 = float(recall[perfect].max()) if perfect.any() else 0.0
    return {"queries": int(len(r)), "valid": int(valid.sum()),
            "R@1": float(top1[valid].mean()) if valid.any() else 0.0,
            "R@5": float(top5[valid].mean()) if valid.any() else 0.0,
            "AP": ap, "R@P100": r_at_p100,
            "pr": [precision[::max(1, len(r) // 200)].tolist(), recall[::max(1, len(r) // 200)].tolist()]}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", required=True)
    ap.add_argument("--emb", required=True)
    ap.add_argument("--dist", type=float, default=1.5)
    ap.add_argument("--angle", type=float, default=30.0)
    ap.add_argument("--gap", type=float, default=15.0)
    ap.add_argument("--json", help="write all metrics (incl. PR curves) here")
    ap.add_argument("methods", nargs="+")
    a = ap.parse_args()
    sets = {s: np.load(Path(a.set) / f"{s}.npz") for env in ENVS.values() for s in env}
    report = {}
    for method in a.methods:
        desc = Descriptors(Path(a.emb), method)
        loop, reloc = [], defaultdict(list)
        for env, seqs in ENVS.items():
            for q in seqs:
                sq = sets[q]
                P = positives(sq["pos"], sq["fwd"], sq["pos"], sq["fwd"], a.dist, a.angle)
                mask = sq["t"][None, :] <= sq["t"][:, None] - a.gap
                loop += query_results(desc.similarity(q, q), P, mask)
                for d in seqs:
                    if d == q:
                        continue
                    sd = sets[d]
                    P = positives(sq["pos"], sq["fwd"], sd["pos"], sd["fwd"], a.dist, a.angle)
                    reloc[env] += query_results(desc.similarity(q, d), P, np.ones_like(P))
        report[method] = {"loop": metrics(loop),
                          "reloc": metrics(sum(reloc.values(), [])),
                          **{f"reloc_{e}": metrics(v) for e, v in reloc.items()}}
    cols = ["loop", "reloc", "reloc_MH", "reloc_V1", "reloc_V2"]
    print(f"same place: < {a.dist} m and < {a.angle} deg; loop queries look >= {a.gap} s back")
    for key in ["R@1", "AP", "R@P100"]:
        print(f"\n{key:8s} " + " ".join(f"{c:>9s}" for c in cols))
        for m, r in report.items():
            print(f"{m:24s}"[:24] + " ".join(f"{r[c][key]:9.3f}" for c in cols))
    first = next(iter(report.values()))
    print("\nqueries with a true match: " + ", ".join(f"{c} {first[c]['valid']}/{first[c]['queries']}" for c in cols))
    if a.json:
        Path(a.json).write_text(json.dumps({"args": vars(a), "report": report}, indent=1))


if __name__ == "__main__":
    main()
