import numpy as np
from scipy.spatial.transform import Rotation as R

class AccelerationState:
    """
    Represents the acceleration state of an object, including translational and rotational components.
    """
    def __init__(self, *,
                 Tx: float = 0,
                 Ty: float = 0,
                 Tz: float = 0,
                 Rx: float = 0,
                 Ry: float = 0,
                 Rz: float = 0,
                 local: bool = None
                 ):
        """
        Initializes the AccelerationState with the given translation and rotation values.
        If `local` is set, it's a shorthand for
        `TotalAccelerationState((global/local)_acceleration=AccelerationState(...))`
        """
        self.translation = np.array([Tx, Ty, Tz])
        self.rotation = np.array([Rx, Ry, Rz])
        self.local = local

    def rotation_obj(self) -> R:
        """
        Returns a scipy Rotation object from the current rotation vector (assumed to be Euler angles in radians, order xyz).
        """
        return R.from_euler('xyz', self.rotation)

    def rotate(self, rotation_obj: R) -> "AccelerationState":
        """
        Rotates the translation and rotation vectors by the given SciPy Rotation object.
        """
        t = rotation_obj.apply(self.translation)
        r = rotation_obj.apply(self.rotation)
        return AccelerationState(Tx=t[0], Ty=t[1], Tz=t[2], Rx=r[0], Ry=r[1], Rz=r[2])

    def to_total(self) -> "TotalAccelerationState":
        """
        Converts the current AccelerationState to a TotalAccelerationState.
        """
        if not self.local:
            return TotalAccelerationState(
                global_acceleration=self
            )
        return TotalAccelerationState(
            local_acceleration=self,
        )
    
    def __add__(self, other: "AccelerationState"):
        if not isinstance(other, AccelerationState):
            raise TypeError(f"Unsupported operand type for +: AccelerationState and {type(other)}")

        if self.local is None or other.local is None or self.local == other.local:
            t = self.translation + other.translation
            r = self.rotation + other.rotation
            return AccelerationState(
                Tx=t[0], Ty=t[1], Tz=t[2],
                Rx=r[0], Ry=r[1], Rz=r[2],
                local=self.local if self.local is not None else other.local
            )
        else:
            if(self.local):
                return TotalAccelerationState(
                    local_acceleration=self,
                    global_acceleration=other
                )
            return TotalAccelerationState(
                local_acceleration=other,
                global_acceleration=self
            )
            
        
    def __str__(self):
        t = self.translation
        r = self.rotation
        return f"AccelerationState object: T=[{t[0]}, {t[1]}, {t[2]}], R=[{r[0]}, {r[1]}, {r[2]}]"
    
class TotalAccelerationState:
    """
    Represents the total acceleration state of an object, including acceleration in
    local space and global space.
    """
    def __init__(self, local_acceleration=None, global_acceleration=None):
        self.local_acceleration = local_acceleration if local_acceleration is not None else AccelerationState()
        self.global_acceleration = global_acceleration if global_acceleration is not None else AccelerationState()

        self.local_acceleration.local = True
        self.global_acceleration.local = False

    def __add__(self, other: "TotalAccelerationState"):
        new_state = TotalAccelerationState()
        if isinstance(other, AccelerationState):
            other = other.to_total()

        if not isinstance(other, TotalAccelerationState):
            raise TypeError(f"Unsupported operand type for +: TotalAccelerationState and {type(other)}")
        new_state.local_acceleration = self.local_acceleration + other.local_acceleration
        new_state.global_acceleration = self.global_acceleration + other.global_acceleration
        return new_state

    def extract_acceleration(self, rotation: R) -> AccelerationState:
        """
        Combines local and global acceleration states into a single local space `AccelerationState` object.
        The current rotation must be passed in as a SciPy rotation to de-rotate the global acceleration.
        """
        local_accel = self.local_acceleration
        global_accel = self.global_acceleration.rotate(rotation.inv())
        global_accel.local = True
        local_accel.local = True
        return local_accel + global_accel

    def __str__(self):
        return f"TotalAccelerationState: Local={self.local_acceleration}, Global={self.global_acceleration}"