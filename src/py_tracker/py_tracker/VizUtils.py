import cv2
import numpy as np
from typing import Dict

# from py_tracker.TrackedObject import TrackedObject
from .TrackedObject import TrackedObject

VIZ_PIXELS_PER_METER = 20
VIZ_FRAME_SIZE = 350


def visualize_tracks(objects: Dict[str, TrackedObject], inputPolygons, bad_points):
    offset_y = offset_x = VIZ_FRAME_SIZE / 2

    def pixelify(point):
        return (int(offset_x + VIZ_PIXELS_PER_METER * point[0]), int(offset_y - VIZ_PIXELS_PER_METER * point[1]))

    # Draw received polygons in the left image
    input_im = np.zeros((VIZ_FRAME_SIZE, VIZ_FRAME_SIZE, 3))
    for poly in inputPolygons:
        poly_coords = [pixelify(pt) for pt in poly.exterior.coords]
        cv2.fillPoly(input_im, [np.array(poly_coords)], color=(0.75, 0.25, 0))
    for point in bad_points:
        cv2.circle(input_im, pixelify(point), 3, (0, 0, 0.75), -1)

    # Draw tracked polygons, colored by their dynamic status
    track_im = np.zeros((VIZ_FRAME_SIZE, VIZ_FRAME_SIZE, 3))
    for obj in objects.values():
        poly_coords = [pixelify(pt) for pt in obj.polygon.exterior.coords]
        color = (0.075, 0.675, 0.012) if obj.isDynamic else (0.012, 0.651, 0.988)
        cv2.fillPoly(track_im, [np.array(poly_coords)], color=color)

    # Overlay some text about tracking status
    for objectID, obj in zip(objects.keys(), objects.values()):
        centroid_pt = pixelify((obj.polygon.centroid.x, obj.polygon.centroid.y))
        cv2.putText(track_im, f"{objectID}, d{obj.disappearedFrames}", centroid_pt, cv2.FONT_HERSHEY_PLAIN, 0.5, (1, 1, 1), 1, cv2.LINE_AA)

    # Combine input and tracked (left and right) images and show
    combined_im = np.hstack((input_im, track_im))
    cv2.imshow("Tracks", combined_im)
    cv2.waitKey(1)
