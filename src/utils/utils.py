import numpy as np
import math
from typing import Tuple

def normalize_quaternion(q):
    """
    Normalizuje kwaternion.
    """
    norm = math.sqrt(sum(i ** 2 for i in q))
    return [i / norm for i in q]


def multiply_quaternions(q1, q2):
    """
    Mnoży dwa kwaterniony q1 i q2.
    
    Kwaterniony powinny być w formacie [w, x, y, z], 
    gdzie w to część skalarna, a x, y, z to części wektorowe.
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    # Obliczenia zgodnie z regułami mnożenia kwaternionów
    w_result = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x_result = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y_result = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z_result = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2

    return np.array([w_result, x_result, y_result, z_result])


def yaw_to_quaternion(yaw):
    """
    Konwertuje kąt yaw (w radianach) na kwaternion.
    """
    w = math.cos(yaw / 2)
    x = 0
    y = 0
    z = math.sin(yaw / 2)
    return [w, x, y, z]



def quaternionRotation(q0123):
    q0 = q0123[0]
    q1 = q0123[1]
    q2 = q0123[2]
    q3 = q0123[3]

    q0s = q0**2
    q1s = q1**2
    q2s = q2**2
    q3s = q3**2

    R = np.array([[q0s + q1s -q2s -q3s, 2*(q1*q2 + q0*q3), 2*(q1*q3 - q0*q2)],
                  [2*(q1*q2 - q0*q3), q0s - q1s + q2s -q3s, 2*(q0*q1 + q2*q3)],
                  [2*(q0*q2 + q1*q3), 2*(q2*q3 - q0*q1), q0s - q1s -q2s +q3s]])
    return R

def isRotationMatrix(R):
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def rotationMatrixToEulerAngles(R):
    assert (isRotationMatrix(R))

    sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

    singular = sy < 1e-6

    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0

    return np.array([x, y, z])

# Function to calculate area using the Shoelace Theorem
def calculate_area(corners):
    x = corners[:, 0]
    y = corners[:, 1]
    
    # Apply the Shoelace formula
    area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))
    
    return area


def regulator_PD(Kp,Kd,val,e_prev= 0, setpoint=640,saturation=0.5) -> Tuple[float, float]:
    # Value of offset - when the error is equal zero
    offset = 1e-9
    
    # PID calculations
    e = setpoint - val
    
    P = Kp*e
    D = Kd*(e - e_prev)

    # calculate manipulated variable - MV 
    MV = offset + P + D
    
    # update stored data for next iteration
    e_prev = e

    return MV if abs(MV) <= saturation else float(np.sign(MV)*saturation), e_prev