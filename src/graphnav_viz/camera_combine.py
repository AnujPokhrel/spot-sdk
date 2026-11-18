"""Merge Spot's two front cameras into one forward-looking view.

Both images are warped into a shared virtual camera that looks along the midpoint
of their view directions. This treats the cameras as if they were in the same spot.
They are about 7 cm apart, so distant scenery lines up across the seam and very close
objects can be slightly offset. Only runs while a client is watching the stream.
"""

from __future__ import annotations

import numpy as np

FRONT_COMBINED = 'front_combined'
FRONT_LEFT = 'frontleft_fisheye_image'
FRONT_RIGHT = 'frontright_fisheye_image'

WIDTH, HEIGHT = 900, 500
FOCAL_PX = 260.0
JPEG_QUALITY = 75


def _camera(response) -> tuple[np.ndarray, np.ndarray]:
    """(intrinsics matrix, camera-to-body rotation) of an ImageResponse."""
    from bosdyn.client.frame_helpers import BODY_FRAME_NAME, get_a_tform_b
    i = response.source.pinhole.intrinsics
    intrinsics = np.array([[i.focal_length.x, i.skew.x, i.principal_point.x],
                           [0.0, i.focal_length.y, i.principal_point.y],
                           [0.0, 0.0, 1.0]])
    body_tform_camera = get_a_tform_b(response.shot.transforms_snapshot, BODY_FRAME_NAME,
                                      response.shot.frame_name_image_sensor)
    return intrinsics, body_tform_camera.rot.to_matrix()


class FrontCombiner:
    """Built from a first pair of ImageResponses. The cameras don't move on the body,
    so the warps are computed once."""

    def __init__(self, left, right):
        import cv2
        left_k, left_r = _camera(left)
        right_k, right_r = _camera(right)

        # Virtual camera axes in the body frame: x right, y down, z along the view.
        view = left_r[:, 2] + right_r[:, 2]
        view /= np.linalg.norm(view)
        x_axis = np.cross(view, [0.0, 0.0, 1.0])
        x_axis /= np.linalg.norm(x_axis)
        body_r_virtual = np.column_stack([x_axis, np.cross(view, x_axis), view])
        virtual_k = np.array([[FOCAL_PX, 0.0, WIDTH / 2], [0.0, FOCAL_PX, HEIGHT / 2], [0.0, 0.0, 1.0]])

        self._left_h = virtual_k @ body_r_virtual.T @ left_r @ np.linalg.inv(left_k)
        self._right_h = virtual_k @ body_r_virtual.T @ right_r @ np.linalg.inv(right_k)

        # The cameras cross over: the front-left camera sees the right side of the
        # view. Each side comes from its own camera, or the other one where it has no pixels.
        size = (left.shot.image.cols, left.shot.image.rows)
        blank = np.full((size[1], size[0]), 255, np.uint8)
        left_covers = cv2.warpPerspective(blank, self._left_h, (WIDTH, HEIGHT)) > 0
        right_covers = cv2.warpPerspective(blank, self._right_h, (WIDTH, HEIGHT)) > 0
        right_half = np.zeros((HEIGHT, WIDTH), bool)
        right_half[:, WIDTH // 2:] = True
        self._use_left = (right_half & left_covers) | ~right_covers

    def combine(self, left, right) -> bytes:
        """JPEG of the merged view."""
        import cv2
        def warp(response, homography):
            jpeg = np.frombuffer(response.shot.image.data, np.uint8)
            return cv2.warpPerspective(cv2.imdecode(jpeg, cv2.IMREAD_COLOR), homography, (WIDTH, HEIGHT))
        out = warp(right, self._right_h)
        out[self._use_left] = warp(left, self._left_h)[self._use_left]
        ok, jpeg = cv2.imencode('.jpg', out, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
        if not ok:
            raise RuntimeError('JPEG encoding of the combined view failed')
        return jpeg.tobytes()
