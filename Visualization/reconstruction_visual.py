# Note: this may not exactly align with the implementation in C, as this script was made prior to 
# any further improvements/optimizations developed.

import random

import matplotlib.pyplot as plt
import numpy as np


MAX_ITERA = 100


class Transform:
    """Recover a 3-D anchor frame from inter-anchor distances and align it
    to the original/reference frame using three known tag positions.
    """

    def __init__(self, anchors: np.ndarray):
        if anchors.ndim != 2 or anchors.shape[1] != 3:
            raise ValueError("anchors must have shape (N, 3)")

        self.orig_anchors = anchors.astype(float).copy()
        self.est_anchors = None
        self.N = len(anchors)

        # Squared pairwise-distance matrix.
        self.S = np.zeros((self.N, self.N))
        for i in range(self.N):
            for j in range(self.N):
                dist = anchors[i] - anchors[j]
                self.S[i, j] = dist @ dist

        # Plot the original frame.
        graph_lim = np.max(np.abs(anchors))
        fig = plt.figure()
        self.ax = fig.add_subplot(111, projection="3d")
        self.ax.set_xlim(-graph_lim, graph_lim)
        self.ax.set_ylim(-graph_lim, graph_lim)
        self.ax.set_zlim(-graph_lim, graph_lim)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")

        self.ax.scatter(
            anchors[:, 0], anchors[:, 1], anchors[:, 2],
            color="blue", s=30, marker="o", label="Original"
        )

        for i, p1 in enumerate(anchors):
            for j, p2 in enumerate(anchors):
                if j <= i:
                    continue

                self.ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    [p1[2], p2[2]],
                    color="black", linestyle="--", linewidth=0.5,
                )

                midpoint = (p1 + p2) / 2
                self.ax.text(
                    *midpoint,
                    f"{np.sqrt(self.S[i, j]):.2f}",
                    color="black", fontsize=6,
                )

        # Reference axes.
        self.ax.plot([0, graph_lim], [0, 0], [0, 0], color="red")
        self.ax.plot([0, 0], [0, graph_lim], [0, 0], color="green")
        self.ax.plot([0, 0], [0, 0], [0, graph_lim], color="blue")
        self.ax.view_init(elev=30, azim=40)

    @staticmethod
    def gram_schmidt(A: np.ndarray) -> np.ndarray:
        """Return an orthonormal basis for the columns of A."""
        E = np.empty_like(A, dtype=float)
        E[:, 0] = A[:, 0] / np.linalg.norm(A[:, 0])

        for i, a in enumerate(A.T[1:], start=1):
            E[:, i] = a
            for e in E.T[:i]:
                E[:, i] -= e * (e @ a) / (e @ e)
            E[:, i] /= np.linalg.norm(E[:, i])

        return E

    def calc_anchors(self):
        """Reconstruct anchor coordinates from the pairwise distances."""

        # Reduced distance-derived matrix.
        M = np.empty((self.N - 1, self.N - 1))
        for i in range(self.N - 1):
            for j in range(self.N - 1):
                M[i, j] = (
                    self.S[0, j + 1]
                    + self.S[i + 1, 0]
                    - self.S[i + 1, j + 1]
                ) / 2

        # The original code uses a fixed 4x3 initialization for N=5.
        V = np.array([
            [0.76144030,  0.13568787,  0.03982924],
            [-0.51907907, -0.02256761,  0.61729982],
            [-0.03411009,  0.96752281,  0.17663539],
            [0.38678430, -0.21208279,  0.76560728],
        ])

        if M.shape != (4, 4):
            raise ValueError("This reconstruction code expects exactly 5 anchors.")

        for _ in range(MAX_ITERA):
            V = self.gram_schmidt(M @ V)

        E = (V.T @ M @ V) * np.eye(3)

        for i in range(3):
            if E[i, i] < 0:
                V[:, i] *= -1
            E[i, i] = abs(E[i, i])

        X = V @ np.sqrt(E)
        self.est_anchors = np.vstack((np.zeros((1, 3)), X))

        # Plot reconstructed frame on top of original frame.
        self.ax.scatter(
            X[:, 0], X[:, 1], X[:, 2],
            color="orange", s=30, marker="o", label="Reconstructed"
        )

        for i, p1 in enumerate(X):
            for j, p2 in enumerate(X):
                if j <= i:
                    continue

                self.ax.plot(
                    [p1[0], p2[0]],
                    [p1[1], p2[1]],
                    [p1[2], p2[2]],
                    color="red", linestyle="--", linewidth=0.5,
                )

        return self.est_anchors

    def get_dists_tag(self, X: np.ndarray) -> np.ndarray:
        """Simulate measured distances from a tag position to each anchor."""
        return np.array([
            np.linalg.norm(anchor - X)
            for anchor in self.orig_anchors
        ]).reshape(self.N, 1)

    def calc_tag(self, x_dists: np.ndarray) -> np.ndarray:
        """Estimate tag coordinates from anchor-to-tag distances."""

        ones = np.ones((self.N, 1))
        A = np.hstack((-2 * self.est_anchors, ones))
        b = (
            x_dists * x_dists
            - np.sum(self.est_anchors * self.est_anchors, axis=1)[:, None]
        )

        A_sq = A.T @ A

        # These values are precomputed for the fixed five-anchor geometry.
        f = np.zeros((4, 1))
        f[3, 0] = -0.5

        D = np.eye(4)
        D[3, 3] = 0

        V = np.array([
            [-0.58327955,  0.46947144, -0.58487514, -0.31193367],
            [-0.08045370,  0.77864130,  0.54669384,  0.29727222],
            [0.77870782,  0.40216641, -0.22738803, -0.42446555],
            [-0.21662482, -0.10759403,  0.55438399, -0.79633888],
        ])

        for _ in range(30):
            V = self.gram_schmidt(A_sq @ V)

        E = (V.T @ A_sq @ V) * np.eye(4)
        E[np.diag_indices(4)] = 1 / np.sqrt(np.diag(E))
        sqrt_inv_Asq = V @ E @ V.T

        ATDA = sqrt_inv_Asq @ D @ sqrt_inv_Asq

        V = np.array([
            [-0.58327955,  0.46947144, -0.58487514, -0.31193367],
            [-0.08045370,  0.77864130,  0.54669384,  0.29727222],
            [0.77870782,  0.40216641, -0.22738803, -0.42446555],
            [-0.21662482, -0.10759403,  0.55438399, -0.79633888],
        ])

        for _ in range(30):
            V = self.gram_schmidt(ATDA @ V)

        E_AD = (V.T @ ATDA @ V) * np.eye(4)
        lambda1_AD = E_AD[0, 0]

        # Find lambda by bisection.
        lower = -1 / lambda1_AD
        upper = 1000.0
        lambda_test = 0.0
        epsilon = 1e-5
        upper_max = upper

        while True:
            y_hat = self.y_hat(A, A_sq, D, b, f, lambda_test)
            phi = y_hat.T @ D @ y_hat + (2 * f.T) @ y_hat

            if np.all(np.abs(phi) <= epsilon):
                break

            if np.any(phi > 0):
                lower = lambda_test
                lambda_test += (upper - lambda_test) / 2
            else:
                upper = lambda_test
                lambda_test -= (lambda_test - lower) / 2

            if lambda_test >= upper_max * 0.90:
                upper *= 2
                upper_max = upper

        return y_hat[:3]

    @staticmethod
    def y_hat(A, A_square, D, b, f, lambda1) -> np.ndarray:
        A_sq_lc = A_square + lambda1 * D
        inv_A_bias = np.linalg.inv(A_sq_lc)
        range_sq = A.T @ b - lambda1 * f
        return inv_A_bias @ range_sq

    def transform_anchors(self, known_coords: np.ndarray, range_noise_std=60.0):
        """Align reconstructed anchors to the original reference frame.

        The existing test setup simulates three known tag positions. Gaussian
        noise is injected into their measured ranges, then the resulting
        coordinate estimates are used to recover the frame rotation.
        """

        known_coords = known_coords.astype(float).copy()

        tag_dists = np.empty((self.N, 3))
        est_tag = np.empty_like(known_coords)

        for i in range(3):
            tag_dists[:, [i]] = self.get_dists_tag(known_coords[i])

            # Noise injection
            tag_dists[:, [i]] += np.random.normal(
                0.0, range_noise_std, self.N
            ).reshape(self.N, 1)

            est_tag[i] = self.calc_tag(tag_dists[:, [i]]).ravel()

        # Normalize each reference point so the alignment uses relative
        # directions
        known_unit = known_coords / np.linalg.norm(
            known_coords, axis=1, keepdims=True
        )
        est_unit = est_tag / np.linalg.norm(
            est_tag, axis=1, keepdims=True
        )

        # Recover the orthogonal transformation.
        M = est_unit.T @ known_unit
        A = M.T @ M
        eigenvalues, eigenvectors = np.linalg.eig(A)

        S = np.diag(np.sqrt(eigenvalues))
        Sinv = np.diag(1 / np.diag(S))

        U = M @ eigenvectors @ Sinv
        H = U @ eigenvectors.T

        self.est_anchors = self.est_anchors @ H

        self.ax.scatter(
            self.est_anchors[:, 0],
            self.est_anchors[:, 1],
            self.est_anchors[:, 2],
            color="green",
            s=70,
            marker="x",
            linewidth=2.5,
            label="Aligned reconstructed",
        )

        difference = self.est_anchors - self.orig_anchors
        per_anchor_error = np.linalg.norm(difference, axis=1)
        total_error = np.sum(per_anchor_error)

        print("\nAnchor difference (reconstructed - original):")
        print(difference)
        print("\nPer-anchor Euclidean error:")
        print(per_anchor_error)
        print(f"\nTotal error: {total_error:.6f}")

        self.ax.legend()
        self.ax.set_title("Original vs. reconstructed anchor frame")

        return self.est_anchors


if __name__ == "__main__":
    np.random.seed(0)
    random.seed(0)

    # Active "real" five-anchor data
    anchors = np.array([
        [   0.0,    0.0,    0.0],
        [-700.0,  300.0, -500.0],
        [ 500.0, -600.0,  400.0],
        [-800.0, -400.0,  600.0],
        [ 400.0,  700.0, -350.0],
    ], dtype=float)
    # Three known reference/tag positions
    known = np.array([
        [-500.0, -400.0, -300.0],
        [ 600.0, -500.0,  500.0],
        [ -50.0,  700.0, -600.0],
    ], dtype=float)

    T = Transform(anchors)
    T.calc_anchors()
    T.transform_anchors(known, range_noise_std=15.0)

    # Show the original and reconstructed/aligned frames.
    plt.show()
