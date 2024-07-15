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

decimal_places = 2

def main():
    global A
    q = np.array([0, 0, 0, 0, 0, 0])
    T = direct_kinematics(q)
    print(T)
    print(A[0]@A[1]@A[2]@A[3]@A[4]@A[5])

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

main()