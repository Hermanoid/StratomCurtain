from enum import Enum
from shapely import Polygon
from rclpy.time import Time
import numpy as np


class TrackState(Enum):
    Static = 0
    WaitingForDynamic = 1
    Dynamic = 2


class TrackedObject:
    def __init__(self, polygon: Polygon, stamp: Time):
        self.polygon: Polygon = polygon
        self.disappearedFrames = 0
        self.state: TrackState = TrackState.Static
        self.stateStartTime = None
        self.lastTrackedTime = None
        self.initializedTime = stamp
        self.isDynamic = False
        self.velocity_rolling = np.array((0, 0))

    def updateState(self, newState: TrackState, nowtime):
        self.state = newState
        self.stateStartTime = nowtime
        self.isDynamic = newState == TrackState.Dynamic
