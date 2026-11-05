"""10x10 squares, 5 mm pitch, central artwork excluded. No QR decoding."""
from dataclasses import dataclass
import cv2
import numpy as np

# 56 real internal intersections outside the damaged central 5x5 intersections.
GRID = np.array([(x, y) for y in range(1, 10) for x in range(1, 10)
                 if x <= 2 or x >= 8 or y <= 2 or y >= 8], np.float32)
ANCHORS = np.float32([[1, 1], [9, 1], [9, 9], [1, 9]])
CELL_CENTERS = np.array([(x + .5, y + .5) for y in range(10) for x in range(10)
                         if x < 3 or x >= 7 or y < 3 or y >= 7], np.float32)
CELL_SIGNS = np.array([1 if (int(x) + int(y)) % 2 == 0 else -1
                       for x, y in CELL_CENTERS])


def project(H, xy):
    return cv2.perspectiveTransform(np.asarray(xy, np.float32).reshape(-1, 1, 2), H).reshape(-1, 2)


def sample(gray, pts):
    pts = np.asarray(pts, np.float32)
    return cv2.remap(gray, pts[:, 0].reshape(1, -1), pts[:, 1].reshape(1, -1),
                     cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT).ravel().astype(float)


def ordered_quad(pts):
    pts = np.asarray(pts, np.float32).reshape(4, 2)
    angles = np.arctan2(pts[:, 1] - pts[:, 1].mean(), pts[:, 0] - pts[:, 0].mean())
    pts = pts[np.argsort(angles)]
    return np.roll(pts, -np.argmin(pts.sum(axis=1)), axis=0)


@dataclass
class Observation:
    H: np.ndarray
    ids: np.ndarray
    pixels: np.ndarray
    grid_rms_px: float
    cell_px: float

    def object_points(self, square_m=.005):
        return np.column_stack((GRID[self.ids] * square_m, np.zeros(len(self.ids)))).astype(np.float64)


class RingBoard:
    """Auto initialize using plate outline, then track fixed physical corner IDs.

    Auto initialization requires an approximately one-square white margin. Margin
    is only a search seed; metric geometry comes entirely from measured corners.
    Failed tracking MUST be recovered with the same four physical anchors.
    """
    def __init__(self, min_corners=40, min_cell_px=8):
        self.min_corners = min_corners
        self.min_cell_px = min_cell_px
        self.previous = None
        self.obs = None
        self.started = False
        self.lost = False

    def refine(self, gray, H):
        h, w = gray.shape
        pred = project(H, GRID)
        dx = project(H, GRID + [1, 0]) - pred
        dy = project(H, GRID + [0, 1]) - pred
        pitch = np.minimum(np.linalg.norm(dx, axis=1), np.linalg.norm(dy, axis=1))
        if not np.all(np.isfinite(pred)) or np.min(pitch) < self.min_cell_px:
            return None
        radius = int(np.clip(np.median(pitch) * .2, 2, 12))
        valid = (pred[:, 0] > radius + 2) & (pred[:, 0] < w - radius - 2)
        valid &= (pred[:, 1] > radius + 2) & (pred[:, 1] < h - radius - 2)
        ids = np.flatnonzero(valid)
        if len(ids) < self.min_corners:
            return None
        points = pred[ids].reshape(-1, 1, 2).copy()
        cv2.cornerSubPix(gray, points, (radius, radius), (-1, -1),
                         (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 40, .005))
        points = points.reshape(-1, 2)
        # Four quadrants must form a saddle, not merely an edge or QR feature.
        offsets = [(-.18, -.18), (.18, -.18), (.18, .18), (-.18, .18)]
        values = np.array([sample(gray, points + a * dx[ids] + b * dy[ids]) for a, b in offsets])
        contrast = np.abs((values[0] + values[2] - values[1] - values[3]) / 2)
        mismatch = np.maximum(np.abs(values[0] - values[2]), np.abs(values[1] - values[3]))
        good = (contrast > 25) & (mismatch < contrast * .8)
        good &= np.linalg.norm(points - pred[ids], axis=1) < pitch[ids] * .32
        ids, points = ids[good], points[good]
        if len(ids) < self.min_corners:
            return None
        Hnew, mask = cv2.findHomography(GRID[ids], points, cv2.RANSAC, 1.5)
        if Hnew is None or mask is None:
            return None
        keep = mask.ravel().astype(bool)
        ids, points = ids[keep], points[keep]
        if len(ids) < self.min_corners:
            return None
        # Require observations around all four sides, not just one narrow strip.
        g = GRID[ids]
        if min(np.sum(g[:, 0] <= 2), np.sum(g[:, 0] >= 8),
               np.sum(g[:, 1] <= 2), np.sum(g[:, 1] >= 8)) < 5:
            return None
        rms = float(np.sqrt(np.mean(np.sum((project(Hnew, g) - points) ** 2, axis=1))))
        if rms > 1.0:
            return None
        # Verify the exterior cell pattern, excluding the entire center artwork.
        cell_pixels = project(Hnew, CELL_CENTERS)
        if np.any(cell_pixels < 0) or np.any(cell_pixels[:, 0] >= w) or np.any(cell_pixels[:, 1] >= h):
            return None
        values = sample(gray, cell_pixels)
        white = values[CELL_SIGNS > 0]
        black = values[CELL_SIGNS < 0]
        if abs(np.median(white) - np.median(black)) < 35:
            return None
        threshold = (np.median(white) + np.median(black)) / 2
        polarity = 1 if np.median(white) > np.median(black) else -1
        agreement = np.mean((values - threshold) * CELL_SIGNS * polarity > 0)
        if agreement < .9:
            return None
        return Observation(Hnew, ids, points, rms, float(np.median(pitch)))

    def initialize(self, gray, anchors=None):
        if self.started and anchors is None:
            raise ValueError('Tracking lost: reselect the SAME physical anchors; auto relabeling is disabled.')
        if anchors is not None:
            H = cv2.getPerspectiveTransform(ANCHORS, np.asarray(anchors, np.float32))
            candidates = [H]
        else:
            # Search plate outline at several brightness levels. No QR detector.
            candidates = []
            for level in (90, 120, 150, 170, 190, 210):
                _, bw = cv2.threshold(gray, level, 255, cv2.THRESH_BINARY)
                contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for contour in sorted(contours, key=cv2.contourArea, reverse=True)[:5]:
                    hull = cv2.convexHull(contour)
                    area = cv2.contourArea(hull)
                    if not gray.size * .008 < area < gray.size * .92:
                        continue
                    quad = cv2.approxPolyDP(hull, .025 * cv2.arcLength(hull, True), True)
                    if len(quad) != 4:
                        continue
                    quad = ordered_quad(quad)
                    sides = np.linalg.norm(quad - np.roll(quad, 1, axis=0), axis=1)
                    if min(sides) < 60 or max(sides) / min(sides) > 3:
                        continue
                    for bx in (.75, 1.0, 1.25):
                        for by in (.75, 1.0, 1.25):
                            source = np.float32([[-bx, -by], [10 + bx, -by],
                                                 [10 + bx, 10 + by], [-bx, 10 + by]])
                            candidates.append(cv2.getPerspectiveTransform(source, quad))
        best = None
        for H in candidates:
            try:
                ob = self.refine(gray, H)
                if ob is not None:
                    ob = self.refine(gray, ob.H)
            except cv2.error:
                ob = None
            if ob is not None and (best is None or (len(ob.ids), -ob.grid_rms_px) > (len(best.ids), -best.grid_rms_px)):
                best = ob
                if len(ob.ids) == len(GRID) and ob.grid_rms_px < .3:
                    break
        if best is not None:
            self.previous, self.obs = gray.copy(), best
            self.started, self.lost = True, False
        return best

    def update(self, gray):
        if self.previous is None or self.lost:
            return None
        old = self.obs.pixels.reshape(-1, 1, 2)
        new, good1, _ = cv2.calcOpticalFlowPyrLK(self.previous, gray, old, None,
                                              winSize=(21, 21), maxLevel=3)
        if new is None:
            self.lost = True
            return None
        back, good2, _ = cv2.calcOpticalFlowPyrLK(gray, self.previous, new, None,
                                               winSize=(21, 21), maxLevel=3)
        if back is None:
            self.lost = True
            return None
        valid = good1.ravel().astype(bool) & good2.ravel().astype(bool)
        valid &= np.linalg.norm(back.reshape(-1, 2) - old.reshape(-1, 2), axis=1) < .7
        valid &= np.linalg.norm(new.reshape(-1, 2) - old.reshape(-1, 2), axis=1) < self.obs.cell_px * .45
        if np.sum(valid) < self.min_corners:
            self.lost = True
            return None
        H, mask = cv2.findHomography(GRID[self.obs.ids[valid]], new.reshape(-1, 2)[valid], cv2.RANSAC, 1.5)
        ob = self.refine(gray, H) if H is not None else None
        if ob is None:
            self.lost = True
            return None
        self.previous, self.obs = gray.copy(), ob
        return ob


def estimate_pose(ob, K, square_m=.005):
    obj = ob.object_points(square_m)
    result = cv2.solvePnPGeneric(obj, ob.pixels.astype(float), K, np.zeros(5), flags=cv2.SOLVEPNP_IPPE)
    if not result[0]:
        raise ValueError('PnP failed')
    candidates = []
    for r, t in zip(result[1], result[2]):
        R = cv2.Rodrigues(r)[0]
        if np.min((obj @ R.T + t.reshape(3))[:, 2]) <= 0:
            continue
        uv = cv2.projectPoints(obj, r, t, K, np.zeros(5))[0].reshape(-1, 2)
        err = float(np.sqrt(np.mean(np.sum((uv - ob.pixels) ** 2, axis=1))))
        candidates.append((err, r, t))
    candidates.sort(key=lambda v: v[0])
    if not candidates or candidates[0][0] > 1.0:
        raise ValueError('PnP reprojection RMS > 1 px or points behind camera')
    if len(candidates) > 1:
        R0, R1 = [cv2.Rodrigues(c[1])[0] for c in candidates[:2]]
        angle = np.linalg.norm(cv2.Rodrigues(R0.T @ R1)[0]) * 180 / np.pi
        if candidates[1][0] - candidates[0][0] < .1 and angle > 5:
            raise ValueError('Planar PnP ambiguous: increase board tilt or image size')
    _, r, t = candidates[0]
    r, t = cv2.solvePnPRefineLM(obj, ob.pixels.astype(float), K, np.zeros(5), r, t)
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = cv2.Rodrigues(r)[0], t.ravel()
    uv = cv2.projectPoints(obj, r, t, K, np.zeros(5))[0].reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum((uv - ob.pixels) ** 2, axis=1))))
    if not np.all(np.isfinite(T)) or np.min((obj @ T[:3, :3].T + T[:3, 3])[:, 2]) <= 0 or rms > 1:
        raise ValueError('Invalid refined PnP')
    return T, rms


def draw(image, ob):
    out = image.copy()
    if ob is None:
        return out
    for idx, p in zip(ob.ids, ob.pixels):
        cv2.circle(out, tuple(np.rint(p).astype(int)), 3, (60, 240, 60), -1)
    anchors = project(ob.H, ANCHORS)
    for label, p in zip(('1', '2', '3', '4'), anchors):
        pt = tuple(np.rint(p).astype(int))
        cv2.circle(out, pt, 7, (0, 160, 255), 2)
        cv2.putText(out, label, (pt[0] + 9, pt[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 160, 255), 2)
    excluded = project(ob.H, np.float32([[3, 3], [7, 3], [7, 7], [3, 7]]))
    cv2.polylines(out, [excluded.astype(np.int32)], True, (50, 50, 255), 2)
    return out
