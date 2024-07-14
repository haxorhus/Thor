import os
import math
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

# Parámetros geométricos (en mm)
L1 = 202.0
L2 = 160.0
L3 = 195.0
L4 = 67.15

# Puntos en el espacio de trabajo
matrix_c = []

# Puntos en el espacio articular
matrix_q = []

# Puntos interpolados
num_points = 30
interpolation_points = []

# Ruta
search_directory = './input/'
file_name = 'coordenadas.txt'

def STR():
    global interpolation_points, matrix_q
    
    # Buscar el archivo
    file_path = find_file(search_directory, file_name)

    if file_path:
        print(f"Archivo encontrado: {file_path}")
        read_coordinates(file_path)
        
        # Convertir coordenadas del espacio de trabajo a coordenadas del espacio articular
        for row in matrix_c:
            x, y, z = row
            q = inverse_kinematics(x, y, z)
            if q is not None:
                matrix_q.append(q)
        
        # Imprimimos ambas matrices
        print("Coordenadas del espacio de trabajo (matrix_c):")
        for row in matrix_c:
            print(row)
        
        print("Coordenadas del espacio articular (matrix_q):")
        for row in matrix_q:
            print(row)

        # Convertir la lista a un array de numpy para la interpolación
        matrix_q = np.array(matrix_q)
        
        # Interpolación spline cúbico para cada ángulo articular
        t = np.arange(len(matrix_q))
        cs_q1 = CubicSpline(t, matrix_q[:, 0])
        cs_q2 = CubicSpline(t, matrix_q[:, 1])
        cs_q3 = CubicSpline(t, matrix_q[:, 2])
        
        # Crear una secuencia de puntos para evaluar la función spline
        t_new = np.linspace(0, len(matrix_q) - 1, num=num_points)
        q1_interp = cs_q1(t_new)
        q2_interp = cs_q2(t_new)
        q3_interp = cs_q3(t_new)

        interpolation_points = list(zip(q1_interp, q2_interp, q2_interp))

        print(interpolation_points)

        # Graficar (opcional)
        plt.figure()
        plt.plot(t, matrix_q[:, 0], 'o', label='q1 Data')
        plt.plot(t_new, q1_interp, '-', label='q1 Interpolated')
        plt.plot(t, matrix_q[:, 1], 'o', label='q2 Data')
        plt.plot(t_new, q2_interp, '-', label='q2 Interpolated')
        plt.plot(t, matrix_q[:, 2], 'o', label='q3 Data')
        plt.plot(t_new, q3_interp, '-', label='q3 Interpolated')
        plt.xlabel('Index')
        plt.ylabel('Angle (degrees)')
        plt.title('Cubic Spline Interpolation of Joint Angles')
        plt.legend()
        plt.show()

    else:
        print("Archivo no encontrado.")

def find_file(directory, file_name):
    for root, dirs, files in os.walk(directory):
        if file_name in files:
            return os.path.join(root, file_name)
    return None

def read_coordinates(txt_file): 
    with open(txt_file, 'r') as file:
        for line in file:
            line = line.strip()
            if line:
                coordinates = line.split()
                matrix_c.append([float(coord) for coord in coordinates])

def inverse_kinematics(x, y, z):
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

def interpolate_trajectory(q_start, q_end):
    global interpolation_points
    # Número de puntos de interpolación
    num_points = 10
    t = np.linspace(0, 1, num_points)

    # Interpolación cúbica con condiciones de velocidad nula en los extremos
    cs_q1 = CubicSpline([0, 1], [q_start[0], q_end[0]], bc_type='clamped')
    cs_q2 = CubicSpline([0, 1], [q_start[1], q_end[1]], bc_type='clamped')
    cs_q3 = CubicSpline([0, 1], [q_start[2], q_end[2]], bc_type='clamped')

    q1_vals = cs_q1(t)
    q2_vals = cs_q2(t)
    q3_vals = cs_q3(t)

    interpolation_points = list(zip(q1_vals, q2_vals, q3_vals))

    q1_vel = cs_q1.derivative()(t)
    q2_vel = cs_q2.derivative()(t)
    q3_vel = cs_q3.derivative()(t)

    q1_acc = cs_q1.derivative(nu=2)(t)
    q2_acc = cs_q2.derivative(nu=2)(t)
    q3_acc = cs_q3.derivative(nu=2)(t)

    # Graficar posiciones, velocidades y aceleraciones
    fig, axs = plt.subplots(3, 1, figsize=(10, 8))

    axs[0].plot(t, q1_vals, label='q1')
    axs[0].plot(t, q2_vals, label='q2')
    axs[0].plot(t, q3_vals, label='q3')
    axs[0].set_ylabel('Posición (grados)')
    axs[0].legend()
    axs[0].grid()

    axs[1].plot(t, q1_vel, label='q1')
    axs[1].plot(t, q2_vel, label='q2')
    axs[1].plot(t, q3_vel, label='q3')
    axs[1].set_ylabel('Velocidad (grados/s)')
    axs[1].legend()
    axs[1].grid()

    axs[2].plot(t, q1_acc, label='q1')
    axs[2].plot(t, q2_acc, label='q2')
    axs[2].plot(t, q3_acc, label='q3')
    axs[2].set_ylabel('Aceleración (grados/s²)')
    axs[2].set_xlabel('Tiempo (s)')
    axs[2].legend()
    axs[2].grid()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    STR()
