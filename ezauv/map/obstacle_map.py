from ezauv.map.flat_map import FlatMap
# from ezauv.map.grid import Grid
from ezauv.map.grid import PathManager
from ezauv.map.grid_objects import GridObject, CircleGridObject, LineGridObject
from ezauv.map.path import Path
from time import time
import numpy as np
from ezauv.simulation.animator import set_visible_obstacles
from copy import copy

class Obstacle:
    def __init__(self, position, radius, lifetime=np.inf):
        self.position = position
        self.radius = radius
        self.lifetime = lifetime

class ObstacleMap(FlatMap):
    def __init__(self, 
                 max_velocity: float,
                 dimensions: tuple[tuple[float, float], tuple[float, float]],
                 bot_radius: float,
                 resolution: float,
                 R: np.ndarray = None,
                 P0: np.ndarray = None
                 ):
        """
        Dimensions is a pair of tuples ((min_x, min_y), (max_x, max_y)). Keep in mind the bot starts at (0,0).
        """
        super().__init__(max_velocity, bot_radius, R=R, P0=P0)
        self.obstacles = []
        self.dimensions = dimensions

        self.path_manager = PathManager(
            dimensions=dimensions,
            resolution=resolution,
            radius=bot_radius,
            obstacles=[]
        )
        self.obstacles_dirty = True
        self.paths = {}

    def update_obstacles(self, obstacles: list[Obstacle], dt=-1):
        # check if the obstacles are about the same as before (within some tolerance), and if any have expired

        obstacle_distance_tolerance = 1
        marked_for_removal = []
        for i, o in enumerate(self.obstacles):
            for no in obstacles:
                distance = np.linalg.norm(o.position - no.position)
                
                # the distance from the closest edge of the first obstacle to the far edge of the second obstacle
                edge_distance = distance + no.radius - o.radius
                if edge_distance < obstacle_distance_tolerance:
                    o.radius = (o.radius + no.radius) / 2  # average radius
                    o.lifetime = no.lifetime  # reset lifetime
                    obstacles.remove(no)
            if dt != -1:
                # print(o)
                if(o.lifetime == np.inf):
                    o.lifetime = 5.0  # default lifetime
                
                o.lifetime -= dt
                if o.lifetime <= 0:
                    marked_for_removal.append(i)
        
        for index in sorted(marked_for_removal, reverse=True):
            del self.obstacles[index]
        
        self.obstacles.extend([copy(o) for o in obstacles])
        if obstacles or marked_for_removal:
            grid_objects = [CircleGridObject(obstacle.position, obstacle.radius) for obstacle in self.obstacles]
            self.path_manager.set_objects(grid_objects)
            self.obstacles_dirty = True
        
        set_visible_obstacles(self.obstacles)
        

    def update(self, sensor_data):
        super().update(sensor_data)
        obstacles = sensor_data.get('obstacles', None)
        dt = sensor_data.get('dt', None)
        # print("Time taken: " + str(dt))

        if obstacles is not None:
            self.update_obstacles(obstacles, dt)

    def generate_path(self, goal, temporary_obstacles: list[GridObject] = None) -> Path:
        start = (self.position[0], self.position[1])
        ticket = self.path_manager.request_path(
            start=start,
            goal_region=goal,
            smooth=False,
            temporary_obstacles=temporary_obstacles
        )
        # path = self.grid.find_path(start, goal, temporary_obstacles=temporary_obstacles)
        # path = self.grid.find_simple_path(start, goal)
        return ticket
    
    def get_path(self, ticket) -> Path:
        if(ticket in self.paths):
            return self.paths[ticket]
        
        path, path_ticket = self.path_manager.get_path()
        if path_ticket == ticket:
            self.paths[path_ticket] = path
            return path
        return None
    
    def need_replan(self, path: Path) -> bool:
        """Whether the current path is no longer valid due to either obstacles or having deviated too far."""
        if self.obstacles_dirty:
            self.obstacles_dirty = False
            return True

        # also replan if we've deviated too far from the path
        distance = path.distance_from_path_squared(self.position)
        if distance > 2.0**2:
            return True

        return False


    def waypoint_at_distance(self, path: Path, distance: float):
        point = path.lookahead_point(self.position, distance)
        return point

    def kill(self):
        self.path_manager.shutdown()