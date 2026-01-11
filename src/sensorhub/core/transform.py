
# src/sensorhub/core/transform.py
from dataclasses import dataclass, asdict
from typing import Dict, Tuple, List
import math

@dataclass
class Transform:
    # Translation in meters
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0
    # Rotation in degrees (roll-X, pitch-Y, yaw-Z)
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0
    # Unitless uniform scale
    scale: float = 1.0

    def to_dict(self) -> Dict[str, float]:
        return asdict(self)

    @staticmethod
    def from_dict(d: Dict) -> "Transform":
        return Transform(
            tx=float(d.get("tx", 0.0)),
            ty=float(d.get("ty", 0.0)),
            tz=float(d.get("tz", 0.0)),
            roll_deg=float(d.get("roll_deg", 0.0)),
            pitch_deg=float(d.get("pitch_deg", 0.0)),
            yaw_deg=float(d.get("yaw_deg", 0.0)),
            scale=float(d.get("scale", 1.0)),
        )

def _rpy_deg_to_matrix(roll_deg: float, pitch_deg: float, yaw_deg: float) -> List[List[float]]:
    rx = math.radians(roll_deg)
    ry = math.radians(pitch_deg)
    rz = math.radians(yaw_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    # R = Rz * Ry * Rx (Z-Y-X)
    Rz = [[cz, -sz, 0],[sz, cz, 0],[0,0,1]]
    Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]]
    def matmul(A,B):
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return matmul(matmul(Rz,Ry),Rx)

def apply_transform_xyz(x: float, y: float, z: float, t: Transform) -> Tuple[float,float,float]:
    # scale
    xs, ys, zs = t.scale*x, t.scale*y, t.scale*z
    # rotate
    R = _rpy_deg_to_matrix(t.roll_deg, t.pitch_deg, t.yaw_deg)
    xr = R[0][0]*xs + R[0][1]*ys + R[0][2]*zs
    yr = R[1][0]*xs + R[1][1]*ys + R[1][2]*zs
    zr = R[2][0]*xs + R[2][1]*ys + R[2][2]*zs
    # translate
    return xr + t.tx, yr + t.ty, zr + t.tz
