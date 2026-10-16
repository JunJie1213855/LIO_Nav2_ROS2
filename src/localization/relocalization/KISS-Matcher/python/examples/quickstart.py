"""Minimal end-to-end smoke test for KISS-Matcher.

Generates a synthetic source point cloud, applies a known rigid transform
to produce the target, and recovers the transform with KISS-Matcher.
No external data files, no visualization deps — just numpy + kiss_matcher.

Run after `pip install kiss-matcher`:

    python -m kiss_matcher.examples.quickstart        # if examples are packaged
    python python/examples/quickstart.py              # from a source checkout
"""
import numpy as np

import kiss_matcher as km


def make_random_rotation(rng: np.random.Generator) -> np.ndarray:
    """Uniform random 3x3 rotation matrix via QR decomposition."""
    q, r = np.linalg.qr(rng.standard_normal((3, 3)))
    q = q @ np.diag(np.sign(np.diag(r)))
    if np.linalg.det(q) < 0:
        q[:, 0] *= -1
    return q


def main() -> None:
    rng = np.random.default_rng(seed=42)

    # Synthetic source: 5k points scattered in a 10 m cube.
    src = rng.uniform(-5.0, 5.0, size=(5000, 3)).astype(np.float32)

    # Known ground-truth rigid transform.
    R_gt = make_random_rotation(rng).astype(np.float32)
    t_gt = np.array([1.5, -0.7, 0.4], dtype=np.float32)
    tgt = (R_gt @ src.T).T + t_gt

    # Estimate the transform.
    voxel_size = 0.3  # metres; tune to your sensor's point spacing
    config = km.KISSMatcherConfig(voxel_size)
    matcher = km.KISSMatcher(config)
    result = matcher.estimate(src, tgt)

    R_est = np.asarray(result.rotation)
    t_est = np.asarray(result.translation)
    rot_err_deg = np.degrees(np.arccos(np.clip((np.trace(R_gt.T @ R_est) - 1) / 2, -1.0, 1.0)))
    trans_err = np.linalg.norm(t_est - t_gt)

    print(f"# source points          : {src.shape[0]}")
    print(f"# rotation inliers       : {matcher.get_num_rotation_inliers()}")
    print(f"# final inliers          : {matcher.get_num_final_inliers()}")
    print(f"rotation error (deg)     : {rot_err_deg:.3f}")
    print(f"translation error (m)    : {trans_err:.4f}")

    if matcher.get_num_final_inliers() < 5:
        raise SystemExit("registration likely failed — try a smaller voxel_size")
    print("\nKISS-Matcher recovered the transform.")


if __name__ == "__main__":
    main()
