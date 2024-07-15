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

def main():
    q = np.array([0, 0, 0, 0, 0, 0])
    T = direct_kinematics(q)
    T = np.around(T, 6)
    print(T)

def direct_kinematics(q):
    theta = q + offset
    T=np.eye(4,4)
    for i in range(DoF):
        A = np.array([[np.cos(theta[i]), -np.cos(alpha[i])*np.sin(theta[i]),  np.sin(alpha[i])*np.sin(theta[i]), a[i]*np.cos(theta[i])],
                      [np.sin(theta[i]),  np.cos(alpha[i])*np.cos(theta[i]), -np.sin(alpha[i])*np.cos(theta[i]), a[i]*np.sin(theta[i])],
                      [0,                 np.sin(alpha[i]),                   np.cos(alpha[i]),                  d[i]                 ],
                      [0,                 0,                                  0,                                 1                    ]])
        T=T@A
    return T

main()