import heapq
import numpy as np
from ezauv.map.grid_objects import GridObject
from ezauv.map.path import Path
from scipy.ndimage import distance_transform_edt
from ezauv.simulation.animator import set_goal_pixels, set_obstacle_pixels

from multiprocessing import Process, Queue
import queue
import time
import signal
import cProfile


class PathManager:
    def __init__(self, dimensions, resolution, radius, obstacles):
        self.request_q = Queue(maxsize=1)
        self.obstacle_q = Queue(maxsize=1)
        self.result_q = Queue(maxsize=1)
        
        self.process = Process(
            target=planner_worker,
            args=(
                dimensions,
                resolution,
                radius,
                obstacles,
                self.request_q,
                self.obstacle_q,
                self.result_q,
            ),
            daemon=True,
        )
        self.process.start()

        self.latest_path = None
        self.latest_path_id = None

    def _replace_queue_item(self, q, item):
        try:
            while True:
                q.get_nowait()
        except queue.Empty:
            pass
        q.put(item)


    def set_objects(self, obstacles):
        self._replace_queue_item(self.obstacle_q, (obstacles))

    def request_path(self, start, goal_region, smooth=True, temporary_obstacles=None):
        path_id = hash((goal_region.rasterize.__hash__() + time.time()) % (2**32))
        self._replace_queue_item(
            self.request_q,
            ("plan", start, goal_region, smooth, temporary_obstacles, path_id)
        )
        return path_id

    def get_path(self):
        path_id = None
        try:
            while True:
                self.latest_path, path_id, obstacle_pixels, goal_pixels = self.result_q.get_nowait()
                if obstacle_pixels is not None:
                    set_obstacle_pixels(obstacle_pixels)
                if goal_pixels is not None:
                    set_goal_pixels(goal_pixels)
        except queue.Empty:
            pass
        return self.latest_path, path_id

    def shutdown(self):
        print("Shutting down path planner...")
        self._replace_queue_item(self.request_q, ("shutdown",))
        self.process.join(5)



def planner_worker(
    dimensions,
    resolution,
    radius,
    obstacles,
    request_q,
    obstacle_q,
    result_q,
):
    profiler = cProfile.Profile()
    profiler.enable()
    signal.signal(signal.SIGINT, signal.SIG_IGN)  # ignore interrupt in child process
    planner = PathPlanner(dimensions, resolution, radius, obstacles, debug_pixels=False)
    while True:
        try:
            while True:
                obs = obstacle_q.get_nowait()
                planner.set_objects(obs)
        except queue.Empty:
            pass
        
        try:
            msg = request_q.get(timeout=0.1)  # check every 100ms
        except queue.Empty:
            continue

        kind = msg[0]
        
        if kind == "shutdown":
            break

        elif kind == "plan":
            _, start, goal_region, smooth, temporary_obstacles, path_id = msg
            print("Planning path to", goal_region, "from", start)
            a = time.time()
            path, obstacle_pixels, goal_pixels = planner.find_path(start, goal_region, smooth, temporary_obstacles)
            print("Path planned in", time.time() - a, "seconds")
            print("Planned path with", len(path.waypoints), "waypoints")
            try:
                while True:
                    result_q.get_nowait()
            except queue.Empty:
                pass
            result_q.put((path, path_id, obstacle_pixels, goal_pixels))
    print("Path planner shutting down.")
    profiler.disable()
    profiler.dump_stats("path_planner_profile.prof")




class PathPlanner:
    def __init__(self, dimensions, resolution, radius, obstacles: list[GridObject], debug_pixels=False):
        """
        dimensions: ((xmin, ymin), (xmax, ymax))
        resolution: meters per cell
        radius: obstacle inflation radius
        """
        self.resolution = resolution
        self.origin = np.array(dimensions[0])

        xs = np.arange(dimensions[0][0], dimensions[1][0], resolution)
        ys = np.arange(dimensions[0][1], dimensions[1][1], resolution)
        self.xs = xs
        self.ys = ys


        self.mesh_grid = np.meshgrid(xs, ys, indexing="ij")
        self.grid = np.zeros(self.mesh_grid[0].shape, dtype=bool)
        self.shape = self.grid.shape

        # occupancy grid (true = obstacle)
        self.obstacles = obstacles
        self.radius = radius
        self.grid = self.compute_occupancy_grid(self.shape, obstacles)
        self.cost_maps = {}
        self.solved_paths = {}
        self.debug_pixels = debug_pixels

    def set_objects(self, obstacles):
        self.obstacles = obstacles
        self.grid = self.compute_occupancy_grid(self.shape, obstacles)

    def compute_occupancy_grid(self, shape, obstacles: list[GridObject]):
        """Compute occupancy grid from list of GridObjects."""
        grid = np.zeros(shape, dtype=bool)
        for obj in obstacles:
            grid |= obj.rasterize(self.grid, self.resolution, self.origin)
        if self.radius > 0:
            dist = distance_transform_edt(~grid) * self.resolution
            grid = dist <= self.radius

        return grid

    def world_to_grid(self, pos):
        pos = np.asarray(pos)
        x_idx = int((pos[0] - self.origin[0]) / self.resolution)
        y_idx = int((pos[1] - self.origin[1]) / self.resolution)
        return (y_idx, x_idx)  # row, col


    def grid_to_world(self, idx):
        y_idx, x_idx = idx
        return np.array([self.origin[0] + (x_idx + 0.5) * self.resolution,
                        self.origin[1] + (y_idx + 0.5) * self.resolution])


    def find_path(self, start, goal_region, smooth=True, temporary_obstacles=None):
        grid = self.grid
        if temporary_obstacles is not None:
            temp_grid = self.compute_occupancy_grid(self.shape, temporary_obstacles)
            grid = grid | temp_grid

        goal_cells = goal_region.rasterize(grid, self.resolution, self.origin)
        obstacle_pixels = None
        goal_pixels = None
        if self.debug_pixels:
            goal_pixels = []
            ys, xs = np.where(goal_cells)
            for x, y in zip(xs, ys):
                goal_pixels.append(self.grid_to_world((y, x)))
            obstacle_pixels = []
            ys, xs = np.where(grid)
            for x, y in zip(xs, ys):
                obstacle_pixels.append(self.grid_to_world((y, x)))
        
        start_idx = self.world_to_grid(start)
        sy, sx = start_idx
        if sy < 0 or sy >= grid.shape[0] or sx < 0 or sx >= grid.shape[1]:
            return self.find_simple_path(start, goal_region), obstacle_pixels, goal_pixels
        
        if grid[sy, sx]:
            start = self.find_free_point(start, grid)
            start_idx = self.world_to_grid(start)
            sy, sx = start_idx

        h, w = grid.shape
        INF = 1e9

        g_score = np.full((h, w), INF, dtype=np.float32)
        closed = np.zeros((h, w), dtype=np.bool_)

        parent_y = np.full((h, w), -1, dtype=np.int16)
        parent_x = np.full((h, w), -1, dtype=np.int16)

        g_score[sy, sx] = 0.0

        ys, xs = np.where(goal_cells)
        if len(xs) == 0:
            return self.find_simple_path(start, goal_region), obstacle_pixels, goal_pixels

        goal_cy = int(np.mean(ys))
        goal_cx = int(np.mean(xs))

        NEIGHBORS = [
            (-1,  0, 1.0), (1,  0, 1.0),
            (0, -1, 1.0), (0,  1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414),
            (1, -1, 1.414),  (1,  1, 1.414),
        ]

        open_set = []
        heapq.heappush(open_set, (0.0, sy, sx))

        while open_set:
            _, y, x = heapq.heappop(open_set)

            if closed[y, x]:
                continue
            closed[y, x] = True

            if goal_cells[y, x]:
                return self._reconstruct_path(
                    start_idx=(sy, sx),
                    goal_idx=(y, x),
                    parent_y=parent_y,
                    parent_x=parent_x,
                    smooth=smooth,
                    grid=grid
                ), obstacle_pixels, goal_pixels

            base_g = g_score[y, x]

            for dy, dx, move_cost in NEIGHBORS:
                ny = y + dy
                nx = x + dx

                if ny < 0 or ny >= h or nx < 0 or nx >= w:
                    continue
                if grid[ny, nx] or closed[ny, nx]:
                    continue

                tentative_g = base_g + move_cost
                if tentative_g >= g_score[ny, nx]:
                    continue

                g_score[ny, nx] = tentative_g
                parent_y[ny, nx] = y
                parent_x[ny, nx] = x

                dxh = abs(nx - goal_cx)
                dyh = abs(ny - goal_cy)
                h_cost = max(dxh, dyh) + 0.414 * min(dxh, dyh)

                heapq.heappush(open_set, (tentative_g + h_cost, ny, nx))

        return self.find_simple_path(start, goal_region), obstacle_pixels, goal_pixels

    def _reconstruct_path(self, start_idx, goal_idx, parent_y, parent_x, smooth, grid):
        y, x = goal_idx
        sy, sx = start_idx

        path = []
        while not (y == sy and x == sx):
            path.append(self.grid_to_world((y, x)))
            py = parent_y[y, x]
            px = parent_x[y, x]
            if py < 0:
                break
            y, x = py, px

        path.append(self.grid_to_world(start_idx))
        path.reverse()

        result = Path(path)
        return self.smooth(result, grid) if smooth else result
    
    def smooth(self, path: Path, grid) -> Path:
        """Smooth the given path using simple shortcutting."""
        waypoints = path.waypoints.tolist()
        if len(waypoints) < 3:
            return path

        smoothed = [waypoints[0]]
        i = 0
        while i < len(waypoints) - 1:
            j = len(waypoints) - 1
            while j > i + 1:
                if self.is_line_free(waypoints[i], waypoints[j], grid):
                    break
                j -= 1
            smoothed.append(waypoints[j])
            i = j

        return Path(smoothed)
    
    def is_line_free(self, start, end, grid) -> bool:
        """Check if the line between start and end is free of obstacles using Bresenham's algorithm."""
        start_idx = self.world_to_grid(start)
        end_idx = self.world_to_grid(end)

        x0, y0 = start_idx
        x1, y1 = end_idx
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx - dy

        while True:
            if grid[y0, x0]:
                return False

            if (x0, y0) == (x1, y1):
                break

            err2 = err * 2
            if err2 > -dy:
                err -= dy
                x0 += sx
            if err2 < dx:
                err += dx
                y0 += sy

        return True
    
    def find_free_point(self, start, grid):
        """Find the closest free point to the start position."""
        start_idx = self.world_to_grid(start)
        sy, sx = start_idx
        h, w = grid.shape

        visited = np.zeros((h, w), dtype=bool)
        queue = [(sy, sx)]
        visited[sy, sx] = True

        NEIGHBORS = [
            (-1,  0), (1,  0),
            (0, -1), (0,  1),
            (-1, -1), (-1, 1),
            (1, -1),  (1,  1),
        ]

        while queue:
            y, x = queue.pop(0)

            if not grid[y, x]:
                return self.grid_to_world((y, x))

            for dy, dx in NEIGHBORS:
                ny = y + dy
                nx = x + dx

                if ny < 0 or ny >= h or nx < 0 or nx >= w:
                    continue
                if visited[ny, nx]:
                    continue

                visited[ny, nx] = True
                queue.append((ny, nx))

        return start
        

    def find_simple_path(self, start, goal):
        """Find the closest point in the goal to the start and return a straight-line path."""
        goal_cells = goal.rasterize(self.grid, self.resolution, self.origin)
        xs, ys = np.where(goal_cells)

        start = np.asarray(start)
        best_pos = None
        best_dist = np.inf

        for x, y in zip(xs, ys):
            pos = self.grid_to_world((x, y))
            d = np.linalg.norm(pos - start)
            if d < best_dist:
                best_dist = d
                best_pos = pos

        if best_pos is None:
            return Path([start, start])

        return Path([start, best_pos])
