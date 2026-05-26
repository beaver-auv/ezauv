import numpy as np


class Path:
    def __init__(self, waypoints):
        self.waypoints = np.asarray(waypoints, dtype=float)

        self.finished = False # if the start is the same as the end, i.e. a point path
        if len(self.waypoints) < 2 or np.allclose(self.waypoints[0], self.waypoints[-1]):
            self.finished = True

        self.end = self.waypoints[-1]

        self.A = self.waypoints[:-1]
        self.B = self.waypoints[1:]

        self.V = self.B - self.A
        self.L = np.linalg.norm(self.V, axis=1)
        if np.any(self.L == 0) and not self.finished:
            raise ValueError("Path contains zero-length segments")

        self.V2 = self.L ** 2

        self.cumlen = np.zeros(len(self.L) + 1)
        self.cumlen[1:] = np.cumsum(self.L)
        self.total_length = self.cumlen[-1]


        self._last_index = 0

    def nearest_point(self, location):
        if self.finished:
            return 0, 0.0
        
        P = np.asarray(location, dtype=float)

        AP = P - self.A

        t = np.sum(AP * self.V, axis=1) / self.V2
        t = np.clip(t, 0.0, 1.0)

        proj = self.A + t[:, None] * self.V

        diff = P - proj
        dist2 = np.sum(diff * diff, axis=1)

        i = np.argmin(dist2)
        self._last_index = i

        return i, t[i]

    def distance_traveled(self, location):
        """
        Distance along path corresponding to the nearest point to location.
        """
        if self.finished:
            return 0.0
        
        i, t = self.nearest_point(location)
        return self.cumlen[i] + t * self.L[i]

    def point_at_distance(self, distance):
        """
        Point at a given distance from the start of the path.
        """
        if self.finished:
            return self.waypoints[0]
        
        if distance <= 0.0:
            return self.waypoints[0]

        if distance >= self.total_length:
            return self.waypoints[-1]

        i = np.searchsorted(self.cumlen, distance, side="right") - 1
        t = (distance - self.cumlen[i]) / self.L[i]

        return self.A[i] + t * self.V[i]

    def lookahead_point(self, location, distance):
        """
        Point `distance` meters ahead of the closest point to location.
        """
        s = self.distance_traveled(location)
        return self.point_at_distance(s + distance)

    def distance_from_path_squared(self, location):
        if self.finished:
            return 0.0
        
        i, t = self.nearest_point(location)
        diff = location - (self.A[i] + t * self.V[i])
        return np.dot(diff, diff)
