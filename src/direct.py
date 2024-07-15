import math
import numpy as np

# Grados de libertad
DoF = 6

# Parámetros geométricos (en mm)
L1 = 202.0
L2 = 160.0
L3 = 195.0
L4 = 67.15

# Parámetros D-H
offset = np.array([0, 0, np.pi/2, 0, 0, 0])
d = np.array([L1, 0, 0, L3, 0, L4])
a = np.array([0, L2, 0, 0, 0, 0])
alpha = np.array([np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])

# Matrices de transformacion homogénea
A = []

decimal_places = 6

def main():
    q1 = math.radians(90)
    q2 = math.radians(45)
    q3 = math.radians(0)
    q = np.array([q1, q2, q3, 0, 0, 0])
    T = direct_kinematics(q)
    print(T)
    #print(A[0]@A[1]@A[2]@A[3]@A[4]@A[5])
    R06 = rotation(90, 0, 0)
    print()
    print(R06)
    R03 = A[0][:3,:3]@A[1][:3,:3]@A[2][:3,:3]
    print()
    print(R03)
    R36 = np.linalg.inv(R03) * R06
    print()
    print(R36)
    q4 = math.asin(- R36[1,2] / R36[2,2])
    q5 = math.acos(R36[2,2])
    q6 = math.atan2(- R36[2,1], R36[2,0])
    print()
    print(math.degrees(q4))
    print(math.degrees(q5))
    print(math.degrees(q6))
    print()
    q = np.array([q1, q2, q3, q4, q5, q6])
    T = direct_kinematics(q)
    T = np.around(T, 2)
    print(T)


def direct_kinematics(q):
    global A
    theta = q + offset
    T = np.eye(4)
    for i in range(DoF):
        A_i = np.array([[np.cos(theta[i]), -np.cos(alpha[i])*np.sin(theta[i]),  np.sin(alpha[i])*np.sin(theta[i]), a[i]*np.cos(theta[i])],
                        [np.sin(theta[i]),  np.cos(alpha[i])*np.cos(theta[i]), -np.sin(alpha[i])*np.cos(theta[i]), a[i]*np.sin(theta[i])],
                        [0,                 np.sin(alpha[i]),                   np.cos(alpha[i]),                  d[i]                 ],
                        [0,                 0,                                  0,                                 1                    ]])
        A_i = np.around(A_i, decimal_places)
        A.append(A_i)
        T = T@A_i
    return T

def rotation(alpha, beta, gamma):

    alpha = math.radians(alpha)
    beta = math.radians(beta)
    gamma = math.radians(gamma)

    # Rz(alpha)*Ry(beta)*Rx(gamma)
    R = np.array([[ np.cos(alpha)*np.cos(beta), -np.sin(alpha)*np.cos(gamma) + np.cos(alpha)*np.sin(beta)*np.sin(gamma),  np.sin(alpha)*np.sin(gamma) + np.cos(alpha)*np.sin(beta)*np.cos(gamma)],
                  [ np.sin(alpha)*np.cos(beta),  np.cos(alpha)*np.cos(gamma) + np.sin(alpha)*np.sin(beta)*np.sin(gamma), -np.cos(alpha)*np.sin(gamma) + np.sin(alpha)*np.sin(beta)*np.cos(gamma)],
                  [-np.sin(beta),                np.cos(beta)*np.sin(gamma),                                              np.cos(beta)*np.cos(gamma)                                            ]])
    R = np.around(R, decimal_places)
    return R

def inverse_kinematics(x, y, z, alpha, beta, gamma):
    
    try:
        # Proyección en el plano XY
        r = math.sqrt(x**2 + y**2)
        # Distancia efectiva desde la base al punto objetivo
        d = math.sqrt(r**2 + (z - L1)**2)
        
        # Verificación de alcance
        if d > (L2 + L3):
            print(f"El punto ({x}, {y}, {z}) está fuera del alcance.")
            return None

        # Cálculo del ángulo θ1
        q1 = math.atan2(y, x)
        
        # Cálculo del ángulo θ2
        a = math.atan2(z - L1, r)
        u = math.sqrt(r**2 + (z - L1)**2)
        q2 = math.pi/2 - (math.acos((L2**2 + u**2 - L3**2) / (2 * L2 * u)) + a)

        # Cálculo del ángulo θ3
        q3 = math.pi - math.acos((L2**2 + L3**2 - u**2) / (2 * L2 * L3))
        
        # Retornar ángulos en grados
        return math.degrees(q1), math.degrees(q2), math.degrees(q3)
    
    except ValueError as e:
        # Captura de errores matemáticos
        print(f"Error en el cálculo de cinemática inversa: {e}")
        return None

main()