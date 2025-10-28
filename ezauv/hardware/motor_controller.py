from typing import List, Callable, Optional
import numpy as np
from gurobipy import GRB, Model, quicksum
from scipy.spatial.transform import Rotation as R
from abc import ABC, abstractmethod

from ezauv.utils.logger import LogLevel
from ezauv import TotalAccelerationState, AccelerationState


class DeadzoneOptimizer:
    def __init__(self, M, bounds, deadzones):
        self.M = M
        self.bounds = bounds
        self.deadzones = deadzones
        self.m, self.n = M.shape

        self.model = Model(
            "MIQP_deadzone"
        )  # miqp = mixed integer quadratic programming
        # quadratic because it minimized the sum of squares of elements of the matrix
        # integer because it uses boolean variables to determine what side of the deadzone the continous variables are on
        # mixed because it also has continuous
        # programming meaning optimization, because it minimizes the sum of squares

        self.eps = self.model.addVars(
            self.m, lb=-float("inf"), vtype=GRB.CONTINUOUS, name="eps"
        )

        self.u = {}
        for i in range(self.n):
            self.u[i] = self.model.addVar(
                lb=bounds[i][0], ub=bounds[i][1], vtype=GRB.CONTINUOUS, name=f"u_{i}"
            )

        self.z = self.model.addVars(self.n, vtype=GRB.BINARY, name="z")
        self.s = self.model.addVars(self.n, vtype=GRB.BINARY, name="s")

        self.M0 = max(abs(b) for bound in bounds for b in bound)

        for i in range(self.n):
            self.model.addConstr(
                self.u[i] >= -self.z[i] * bounds[i][1], name=f"u_lower_bound_{i}"
            )
            self.model.addConstr(
                self.u[i] <= self.z[i] * bounds[i][1], name=f"u_upper_bound_{i}"
            )
            # either bounded in (-b, b) or (0, 0)

        for i in range(self.n):
            self.model.addConstr(
                self.u[i] - self.deadzones[i][1] * self.s[i] + self.M0 * (1 - self.s[i])
                >= self.M0 * (1 - self.z[i]),
                name=f"u_upper_deadzone_{i}"
            )

            self.model.addConstr(
                self.u[i] - self.M0 * self.s[i] + deadzones[i][0] * (1 - self.s[i])
                <= self.M0 * (1 - self.z[i]),
                name=f"u_lower_deadzone_{i}"
            )
            # deadzones

        self.constrs = []
        for j in range(self.m):
            expr = (
                quicksum(self.M[j, i] * (self.u[i]) for i in range(self.n))
                + self.eps[j]
            )

            self.constrs.append(self.model.addConstr(expr == 0, name=f"eq_row_{j}"))
            # matrix multiplication must be true

        self.model.Params.OutputFlag = 0
        self.model.update()

    def optimize(self, V, lock_to_yaw=False):
        # if `lock_to_yaw` is true, if the wanted acceleration is not possible,
        # the optimizer will find the nearest rotation without changing roll or pitch axes
        # this means that the optimizer will only adjust the yaw rotation

        for j in range(self.m):
            self.constrs[j].setAttr(GRB.Attr.RHS, V[j])
        if lock_to_yaw:
            self.eps[3].LB = 0
            self.eps[3].UB = 0
            self.eps[4].LB = 0
            self.eps[4].UB = 0
        self.model.setObjective(
            quicksum(self.eps[j] * self.eps[j] for j in range(self.m)), GRB.MINIMIZE
        )
        self.model.optimize()

        if self.model.status != GRB.OPTIMAL:
            return False, None

        eps_opt = [self.eps[j].X for j in range(self.m)]

        for j in range(self.m):
            self.eps[j].LB = eps_opt[j]
            self.eps[j].UB = eps_opt[j]

        self.model.setObjective(
            quicksum(self.u[i] * self.u[i] for i in range(self.n)), GRB.MINIMIZE
        )
        self.model.optimize()

        for j in range(self.m):
            self.eps[j].LB = -float("inf")
            self.eps[j].UB = float("inf")
        if self.model.status == GRB.OPTIMAL:
            return True, np.array([self.u[i].X for i in range(self.n)])

        return False, None


class Motor:
    class Range:
        def __init__(self, bottom: float, top: float):
            self.max = top
            self.min = bottom

    def __init__(
        self,
        thrust_vector: np.ndarray,
        position: np.ndarray,
        set_motor: Callable,
        initialize: Callable,
        bounds: Range,
        deadzone: Range,
    ):
        self.thrust_vector: np.ndarray = thrust_vector / np.linalg.norm(thrust_vector)
        self.position: np.ndarray = position
        self.set: Callable = set_motor

        self.initialize: Callable = initialize
        self.torque_vector: np.ndarray = np.cross(self.position, self.thrust_vector)
        self.torque_vector /= np.linalg.norm(self.torque_vector)

        self.bounds: Motor.Range = bounds
        self.deadzone: Motor.Range = deadzone
    @abstractmethod
    def manual_motor(self) -> None:
        # Manually operate the motor.
        pass
    @abstractmethod
    def reset_motor(self) -> None:
        # Reset the motor
        pass

class MotorController:
    def __init__(self, *, inertia: np.ndarray, motors: List[Motor], coefficients=[0, 1]):
        self.inv_inertia: np.ndarray = np.linalg.inv(inertia)  # the inverse inertia tensor of the entire body
        self.motors: np.ndarray = np.array(motors)  # the list of motors this sub owns
        self.log: Callable = lambda str, level=None: print(
            f"Motor logger is not set --- {str}"
        )

        self.polynomial = np.polynomial.Polynomial(coefficients)

        self.optimizer: Optional[DeadzoneOptimizer] = None

        self.motor_matrix = None
        self.mT = None
        self.reset_optimizer(self.polynomial)

        self.prev_sent = {}

    def overview(self) -> None:
        self.log("---Motor controller overview---")
        self.log(f"Inverse inertia tensor:\n{self.inv_inertia}")
        self.log(f"{len(self.motors)} motors connected")

    def initialize(self) -> None:
        self.log("Initializing motors...")

        problems = 0
        for motor in self.motors:
            problems += motor.initialize()

        level = LogLevel.INFO if problems == 0 else LogLevel.WARNING

        self.log(
            f"Motors initalized with {problems} problem{'' if problems == 1 else 's'}",
            level=level,
        )

    def reset_optimizer(self, p):
        """
        Recalculate the motor matrix.
        Should be called if the inertia, motor locations, or motor thrust vectors are changed.
        """
        bounds = []
        deadzones = []

        for i, motor in enumerate(self.motors):
            new_vector = np.array(
                [
                    np.concatenate(
                        [motor.thrust_vector, self.inv_inertia @ motor.torque_vector],
                        axis=None,
                    )
                ]
            ).T
            if i == 0:
                self.motor_matrix = new_vector
            else:
                self.motor_matrix = np.hstack((self.motor_matrix, new_vector))

            bounds.append((p(motor.bounds.min), p(motor.bounds.max)))
            deadzones.append((p(motor.deadzone.min), p(motor.deadzone.max)))
        self.optimizer = DeadzoneOptimizer(self.motor_matrix, bounds, deadzones)
        self.mT = self.motor_matrix.T

    def solve(self, mixed_acceleration: TotalAccelerationState, rotation: R, lock_to_yaw: bool = False):
        """
        Find the array of motor speeds needed to travel at a specific thrust vector and rotation.
        Finds the next best solution if this vector is not possible.
        \n
        If `lock_to_yaw` is true, the given global acceleration will only be rotated by the global
        yaw rotation.
        """

        if isinstance(mixed_acceleration, AccelerationState):
            mixed_acceleration = mixed_acceleration.to_total()

        if lock_to_yaw:
            yaw, _, _ = rotation.as_euler('zyx', degrees=False)
            rotation = R.from_euler('z', yaw, degrees=False)
        # print(mixed_acceleration)
        acceleration = mixed_acceleration.extract_acceleration(rotation)
        # print(acceleration)
        Rx, Ry, Rz = acceleration.rotation
        rotated_wanted = np.append(acceleration.translation, np.array([Rx, Ry, Rz]))

        optimized = self.optimizer.optimize(rotated_wanted, lock_to_yaw)

        if not optimized[0]:
            return False, None

        motor_controls = []
        for target in optimized[1]:
            roots = (self.polynomial - target).roots()
            # print(self.polynomial)
            motor_controls.append(
                min([r.real for r in roots if abs(r.imag) < 1e-8],
                key=lambda x: abs(x))
            )
        
        return True, motor_controls

    def set_motors(self, motor_speeds):
        """
        Set each motor to a corresponding speed of motor_speeds.
        """
        for i, motor in enumerate(self.motors):
            speed = motor_speeds[i]
            if motor in self.prev_sent and self.prev_sent[motor] == speed:
                continue
            motor.set(speed)
            self.prev_sent[motor] = speed

    def killed(self):
        """
        Check if the last value sent to each motor was zero.
        """
        return np.all(np.isclose([view[1] for view in self.prev_sent.items()], 0))
