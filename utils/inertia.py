from abc import ABC, abstractmethod
import numpy as np

# defines the geometry of the auv, so the code can figure out how it will rotate

class InertiaGeometry(ABC):
    """
    An abstract class to define a geometry which is a part of the body.
    """


    def __init__(self, mass: int, center: np.ndarray):
        self.mass = mass
        self.center = center

    @abstractmethod
    def inertia_tensor(self) -> np.ndarray:
        """
        Should return the moment of inertia tensor of this body as a numpy matrix.
        """
        pass

    def translate(self, inertia: np.ndarray, displacement_vector: np.ndarray) -> np.ndarray:
        """
        Translate the moment of inertia tensor by displacement_vector using the parallel axis theorem.
        """
        I_0 = inertia
        m = self.mass 
        R = displacement_vector
        return I_0 + m * (np.dot(R, R) * np.eye(3) - np.outer(R, R))

    def rotate(self, inertia: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
        """
        Rotate the moment of inertia tensor by the given rotation matrix.
        """
        I_0 = inertia
        R = rotation_matrix
        R_T = rotation_matrix.T
        return R @ I_0 @ R_T

    def rotate_to_vector(self, inertia: np.ndarray, current_facing: np.ndarray, to_face: np.ndarray) -> np.ndarray:
        """
        Rotate the moment of inertia tensor to face a vector, given the vector it already faces.
        """
        if(np.all(np.isclose(current_facing, to_face))): # if it pretty much already is correct
            return inertia
            
        rotation_axis = np.cross(to_face, current_facing)
        rotation_axis /= np.linalg.norm(rotation_axis)
        rotation_angle = np.arccos(np.dot(to_face, current_facing))

        K = np.array([
            [0, -rotation_axis[2], rotation_axis[1]],
            [rotation_axis[2], 0, -rotation_axis[0]],
            [-rotation_axis[1], rotation_axis[0], 0]
        ]) # skew-symmetric matrix

        R = np.eye(3) + K + np.dot(np.square(K), (1 - np.cos(rotation_angle))/np.sin(rotation_angle))
        # rodrigues's formula

        return self.rotate(inertia, R)

    def shift_center(self, inertia: np.ndarray, new_center: np.ndarray) -> np.ndarray:
        """
        Shift the moment of inertia tensor to have a new center 
        (center means origin; you're saying that position is the new (0, 0, 0))
        """
        displacement = new_center - self.center
        return self.translate(inertia, displacement) 



class Sphere(InertiaGeometry):

    def __init__(self, mass: float, center: np.ndarray, radius: float):
        super().__init__(mass, center)
        self.radius = radius

    def inertia_tensor(self) -> np.ndarray:
        I = (2 / 5) * self.mass * self.radius**2
        return I * np.eye(3)
    
class HollowCylinder(InertiaGeometry):

    def __init__(self, mass: float, center: np.ndarray, inner_radius: float, outer_radius: float, height: float, facing: np.ndarray):
        """
        mass: mass of the hollow cylinder
        center: center of the hollow cylinder
        inner_radius: radius of the hollow cutout of the cylinder
        outer_radius: radius of the whole cylinder
        height: height of the cylinder
        facing: unit vector of the flat side of the cylinder
        """
        # facing is unit vector of flat side
        super().__init__(mass, center)
        self.inner_radius = inner_radius
        self.outer_radius = outer_radius
        self.height = height
        self.facing = facing / np.linalg.norm(facing)

    def inertia_tensor(self) -> np.ndarray:

        R = self.outer_radius
        k = self.inner_radius
        h = self.height

        I_zz = (1/2) * self.mass * (R**2 + k**2)  
        I_xx_yy = (1/4) * self.mass * (R**2 + k**2) + (1/12) * self.mass * h**2

        principal_tensor = np.diag([I_xx_yy, I_xx_yy, I_zz])

        return self.rotate_to_vector(principal_tensor, np.array([0, 0, 1]), self.facing)
    
class Cuboid(InertiaGeometry):

    def __init__(self, mass: float, center: np.ndarray, width: float, height: float, depth: float):
        super().__init__(mass, center)
        self.width = width 
        self.height = height 
        self.depth = depth 

    def inertia_tensor(self) -> np.ndarray:
        a = self.width
        b = self.height
        c = self.depth

        I_xx = (1 / 12) * self.mass * (b**2 + c**2)
        I_yy = (1 / 12) * self.mass * (a**2 + c**2)
        I_zz = (1 / 12) * self.mass * (a**2 + b**2)

        I = np.diag([I_xx, I_yy, I_zz])
        return I




class InertiaBuilder:
    """
    A class to take in geometries, and return an inertia matrix of your AUV.
    """
   
    def __init__(self, *args: InertiaGeometry):
        """
        Pass in any number of InertiaGeometry objects
        """
        self.geometries = args

    def moment_of_inertia(self, center: np.ndarray = np.array([0, 0, 0])) -> np.ndarray:
        """
        Calculate the moment of inertia tensor (numpy matrix) of the AUV. Center should be the center
        of mass of the body, or just [0, 0, 0] if you're not sure.
        """
        total_inertia = np.zeros((3, 3))
        for geometry in self.geometries:
            shifted = geometry.shift_center(geometry.inertia_tensor(), center)
            total_inertia += shifted

        return total_inertia
