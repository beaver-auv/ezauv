import numpy as np
from scipy.spatial.transform import Rotation as R

class VelocityState:
    """
    Represents the velocity state of an object, including translational and rotational components.
    """
    def __init__(self, *,
                 Tx: float = None,
                 Ty: float = None,
                 Tz: float = None,
                 Rx: float = None,
                 Ry: float = None,
                 Rz: float = None,
                 local: bool = None
                 ):
        """
        Initializes the VelocityState with the given translation and rotation values.
        If `local` is set, it's a shorthand for
        `TotalVelocity((global/local)_velocity=VelocityState(...))`
        """
        self.translation = np.array([Tx, Ty, Tz])
        self.rotation = np.array([Rx, Ry, Rz])
        self.local = local

    def rotation_obj(self) -> R:
        """
        Returns a scipy Rotation object from the current rotation vector (assumed to be Euler angles in radians, order xyz).
        """
        if np.any(np.isnan(self.rotation)):
            raise ValueError("Rotation vector contains None values, cannot create Rotation object.")
        return R.from_euler('xyz', self.rotation)

    def rotate(self, rotation_obj: R) -> "VelocityState":
        """
        Rotates the translation and rotation vectors by the given SciPy Rotation object.
        """
        if np.any(np.isnan(self.translation)) or np.any(np.isnan(self.rotation)):
            raise ValueError("Translation or rotation vector contains None values, cannot rotate.")
        t = rotation_obj.apply(self.translation)
        r = rotation_obj.apply(self.rotation)
        return VelocityState(Tx=t[0], Ty=t[1], Tz=t[2], Rx=r[0], Ry=r[1], Rz=r[2])

    def to_total(self) -> "TotalVelocityState":
        """
        Converts the current VelocityState to a TotalVelocityState.
        """
        if not self.local:
            return TotalVelocityState(
                global_velocity=self
            )
        return TotalVelocityState(
            local_velocity=self,
        )
    
    def __add__(self, other: "VelocityState"):
        if not isinstance(other, VelocityState):
            raise TypeError(f"Unsupported operand type for +: VelocityState and {type(other)}")

        if self.local is None or other.local is None or self.local == other.local:
            t = self.translation + other.translation
            r = self.rotation + other.rotation
            return VelocityState(
                Tx=t[0], Ty=t[1], Tz=t[2],
                Rx=r[0], Ry=r[1], Rz=r[2],
                local=self.local if self.local is not None else other.local
            )
        else:
            if self.local:
                return TotalVelocityState(
                    local_velocity=self,
                    global_velocity=other
                )
            return TotalVelocityState(
                local_velocity=other,
                global_velocity=self
            )
            
        
    def __str__(self):
        t = self.translation
        r = self.rotation
        return f"VelocityState object: T=[{t[0]}, {t[1]}, {t[2]}], R=[{r[0]}, {r[1]}, {r[2]}]"
    
class TotalVelocityState:
    """
    Represents the total velocity state of an object, including velocity in
    local space and global space.
    """
    def __init__(self, local_velocity=None, global_velocity=None):
        self.local_velocity = local_velocity if local_velocity is not None else VelocityState()
        self.global_velocity = global_velocity if global_velocity is not None else VelocityState()

        self.local_velocity.local = True
        self.global_velocity.local = False

    def __add__(self, other: "TotalVelocityState"):
        new_state = TotalVelocityState()
        if isinstance(other, VelocityState):
            other = other.to_total()

        if not isinstance(other, TotalVelocityState):
            raise TypeError(f"Unsupported operand type for +: TotalVelocityState and {type(other)}")
        new_state.local_velocity = self.local_velocity + other.local_velocity
        new_state.global_velocity = self.global_velocity + other.global_velocity
        return new_state

    def extract_velocity(self, rotation: R) -> VelocityState:
        """
        Combines local and global velocity states into a single local space `VelocityState` object.
        The current rotation must be passed in as a SciPy rotation to de-rotate the global velocity.
        """
        local_accel = self.local_velocity
        global_accel = self.global_velocity.rotate(rotation.inv())
        global_accel.local = True
        local_accel.local = True
        return local_accel + global_accel

    def __str__(self):
        return f"TotalVelocityState: Local={self.local_velocity}, Global={self.global_velocity}"