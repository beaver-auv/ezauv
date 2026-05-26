from abc import ABC, abstractmethod
import numpy as np


class GridObject(ABC):
    @abstractmethod
    def rasterize(self, grid, resolution, origin) -> np.ndarray:
        """Return boolean grid marking occupied cells."""
        ...

    def __eq__(self, value):
        if not isinstance(value, GridObject):
            return False
        if type(self) != type(value):
            return False
        for attr in self.__dict__:
            if not np.array_equal(getattr(self, attr), getattr(value, attr)):
                return False
        return True


class StaticGridObject(GridObject):
    def __init__(self):
        self._cached_grid = None
        self._cached_shape = None

    def rasterize(self, grid, resolution, origin):
        if self._cached_grid is None or self._cached_shape != grid.shape:
            self._cached_grid = self._rasterize(grid, resolution, origin)
            self._cached_shape = grid.shape
        return self._cached_grid


    @abstractmethod
    def _rasterize(self, grid, resolution, origin) -> np.ndarray:
        ...

class StaticSDFGridObject(StaticGridObject):
    @abstractmethod
    def sdf(self, x, y) -> np.ndarray:
        """Return signed distance field at given points."""
        ...

    @abstractmethod
    def bounding_box(self) -> tuple[float, float, float, float]:
        """Return (xmin, xmax, ymin, ymax) of bounding box."""
        ...
    
    def translate(self, offset: np.ndarray, x, y):
        return x - offset[0], y - offset[1]
    
    def rotate(self, angle: float, x, y):
        c, s = np.cos(-angle), np.sin(-angle)
        x_rot = c * x - s * y
        y_rot = s * x + c * y
        return x_rot, y_rot

    
    def _rasterize(self, grid, resolution, origin) -> np.ndarray:
        bounds = self.bounding_box()
        h, w = grid.shape

        ixmin = max(0, int((bounds[0] - origin[0]) / resolution))
        ixmax = min(w, int((bounds[1] - origin[0]) / resolution) + 1)
        iymin = max(0, int((bounds[2] - origin[1]) / resolution))
        iymax = min(h, int((bounds[3] - origin[1]) / resolution) + 1)

        if ixmin >= ixmax:
            ixmax = min(w, ixmin + 1)
        if iymin >= iymax:
            iymax = min(h, iymin + 1)

        xv = origin[0] + (np.arange(ixmin, ixmax) + 0.5) * resolution  # horizontal / x
        yv = origin[1] + (np.arange(iymin, iymax) + 0.5) * resolution  # vertical / y

        X, Y = np.meshgrid(xv, yv, indexing='xy')  # shape = (iymax-iymin, ixmax-ixmin)

        mask = self.sdf(X, Y) <= 0.0

        out = np.zeros_like(grid, dtype=bool)
        out[iymin:iymax, ixmin:ixmax] = mask

        return out




class CircleGridObject(StaticSDFGridObject):
    def __init__(self, center: np.ndarray, radius: float):
        super().__init__()
        self.center = center
        self.radius = radius
    
    def bounding_box(self):
        return (self.center[0] - self.radius, self.center[0] + self.radius,
                self.center[1] - self.radius, self.center[1] + self.radius)
    
    def sdf(self, x, y) -> np.ndarray:
        return np.sqrt((x - self.center[0])**2 + (y - self.center[1])**2) - self.radius
    
class RectangleGridObject(StaticSDFGridObject):
    def __init__(self, center: np.ndarray, width: float, height: float, angle: float = 0.0):
        super().__init__()
        self.center = center
        self.width = width
        self.height = height
        self.angle = angle  # in radians

        c, s = np.cos(-angle), np.sin(-angle)
        self.rotation_matrix = np.array([[c, -s], [s, c]])
    
    def bounding_box(self):
        corners = np.array([
            [-self.width/2, -self.height/2],
            [self.width/2, -self.height/2],
            [self.width/2, self.height/2],
            [-self.width/2, self.height/2]
        ])
        rotated_corners = (self.rotation_matrix @ corners.T).T + self.center
        xmin = np.min(rotated_corners[:,0])
        xmax = np.max(rotated_corners[:,0])
        ymin = np.min(rotated_corners[:,1])
        ymax = np.max(rotated_corners[:,1])
        return (xmin, xmax, ymin, ymax)
    
    def sdf(self, x, y) -> np.ndarray:
        x_rot, y_rot = self.translate(self.center, x, y)
        x_rot, y_rot = self.rotate(self.angle, x_rot, y_rot)

        dx = np.abs(x_rot) - self.width / 2
        dy = np.abs(y_rot) - self.height / 2

        outside = np.sqrt(np.maximum(dx, 0)**2 + np.maximum(dy, 0)**2)
        inside = np.minimum(np.maximum(dx, dy), 0)

        return outside + inside

    
class LineGridObject(RectangleGridObject):
    def __init__(self, start: np.ndarray, end: np.ndarray, thickness: float):
        center = (start + end) / 2
        length = np.linalg.norm(end - start)
        angle = np.arctan2(end[1] - start[1], end[0] - start[0])
        super().__init__(center=center, width=length, height=thickness, angle=angle)