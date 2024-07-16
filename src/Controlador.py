import os
import math
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt
import serial
import threading
import tkinter as tk

# Depuracion
DEBUG = True

# Comunicacion
PORT = 'COM4'
BAUD_RATE = 115200
com = None
is_running = True
serial_data_receiver = None
waiting_for_done = False

# Ubicacion del archivo de coordenadas
search_directory = './input/'
file_name = 'coordenadas.txt'

# Puntos interpolados
num_points = 30
decimal_places = 6
interpolation_points = []

# Grados de libertad
DoF = 6

# Parámetros geométricos (en mm)
L1 = 202.0
L2 = 160.0
L3 = 195.0
L4 = 67.15

# Parámetros D-H
offset = np.array([0, -np.pi/2, np.pi/2, 0, 0, 0])
d = np.array([L1, 0, 0, L3, 0, L4])
a = np.array([0, L2, 0, 0, 0, 0])
alpha = np.array([-np.pi/2, 0, np.pi/2, -np.pi/2, np.pi/2, 0])

def main():
    global com, serial_data_receiver

    com = serial.Serial(PORT, BAUD_RATE)
    serial_data_receiver = threading.Thread(target=receive_data)
    serial_data_receiver.start()

def STR():
    global interpolation_points

    # Buscar el archivo
    file_path = find_file(search_directory, file_name)

    if file_path:
        print(f"Archivo encontrado: {file_path}")
        matrix_c = read_coordinates(file_path)
        matrix_q = []
        
        # Convertir coordenadas del espacio de trabajo a coordenadas del espacio articular
        for row in matrix_c:
            x, y, z = row
            q = wp(x, y, z)
            if q is not None:
                matrix_q.append(q)

        # Convertir la lista a un array de numpy para la interpolación
        matrix_q = np.array(matrix_q)
        
        # Interpolación spline cúbico para cada ángulo articular
        t = np.arange(len(matrix_q))
        cs_q1 = CubicSpline(t, matrix_q[:, 0])
        cs_q2 = CubicSpline(t, matrix_q[:, 1])
        cs_q3 = CubicSpline(t, matrix_q[:, 2])
        
        # Crear una secuencia de puntos para evaluar la función spline
        u = np.linspace(0, len(matrix_q) - 1, num=num_points)
        q1_eval = cs_q1(u)
        q2_eval = cs_q2(u)
        q3_eval = cs_q3(u)

        # Crear la matriz de puntos interpolados
        interpolation_points = np.column_stack((q1_eval, q2_eval, q2_eval))

        # Redondear los valores de la matriz de puntos interpolados
        interpolation_points = np.round(interpolation_points, decimals=decimal_places)

        if DEBUG:
            print(interpolation_points)

        # Calcular velocidades y aceleraciones
        q1_speed = cs_q1.derivative()(u)
        q2_speed = cs_q2.derivative()(u)
        q3_speed = cs_q3.derivative()(u)
        
        q1_accel = cs_q1.derivative(2)(u)
        q2_accel = cs_q2.derivative(2)(u)
        q3_accel = cs_q3.derivative(2)(u)

        # Crear subgráficas para posiciones, velocidades y aceleraciones
        fig, axs = plt.subplots(3, 1, figsize=(10, 15), sharex=True)
        
        # Gráfico de posiciones articulares
        axs[0].plot(t, matrix_q[:, 0], 'o', label='q1 Datos')
        axs[0].plot(u, q1_eval, '-', label='q1 Interpolado')
        axs[0].plot(t, matrix_q[:, 1], 'o', label='q2 Datos')
        axs[0].plot(u, q2_eval, '-', label='q2 Interpolado')
        axs[0].plot(t, matrix_q[:, 2], 'o', label='q3 Datos')
        axs[0].plot(u, q3_eval, '-', label='q3 Interpolado')
        axs[0].set_ylabel('Posición (grados)')
        axs[0].set_title('Interpolacion cubica de las variables articulares')
        axs[0].legend()
        axs[0].grid()
        
        # Gráfico de velocidades articulares
        axs[1].plot(u, q1_speed, '-', label='q1 Velocidad')
        axs[1].plot(u, q2_speed, '-', label='q2 Velocidad')
        axs[1].plot(u, q3_speed, '-', label='q3 Velocidad')
        axs[1].set_ylabel('Velocidad (grados/s)')
        axs[1].legend()
        axs[1].grid()
        
        # Gráfico de aceleraciones articulares
        axs[2].plot(u, q1_accel, '-', label='q1 Aceleracion')
        axs[2].plot(u, q2_accel, '-', label='q2 Aceleracion')
        axs[2].plot(u, q3_accel, '-', label='q3 Aceleracion')
        axs[2].set_xlabel('Indice')
        axs[2].set_ylabel('Aceleración (grados/s²)')
        axs[2].legend()
        axs[2].grid()
        
        # Ajustar el espaciado entre subgráficas
        plt.tight_layout()
        plt.show()

    else:
        print("Archivo no encontrado.")

def GTR():
    global interpolation_points
    if interpolation_points.shape[0] > 0:
        q1, q2, q3 = interpolation_points[0]
        interpolation_points = interpolation_points[1:]
        result = f"wp {q1:.{decimal_places}f} {q2:.{decimal_places}f} {q3:.{decimal_places}f}"
        com.write(result.encode() + b'\n')
        output_text.insert(tk.END, f" USER >> {result}\n")
    else:
        output_text.insert(tk.END, f" USER >> Trayectoria finalizada.\n")

def find_file(directory, file_name):
    for root, dirs, files in os.walk(directory):
        if file_name in files:
            return os.path.join(root, file_name)
    return None

def read_coordinates(txt_file):
    matrix = []
    with open(txt_file, 'r') as file:
        for line in file:
            line = line.strip()
            if line:
                coordinates = line.split()
                matrix.append([float(coord) for coord in coordinates])
    return matrix



def direct_model(q):
    theta = q + offset
    T = np.eye(4)
    for i in range(DoF):
        A_i = np.array([[np.cos(theta[i]), -np.cos(alpha[i])*np.sin(theta[i]),  np.sin(alpha[i])*np.sin(theta[i]), a[i]*np.cos(theta[i])],
                        [np.sin(theta[i]),  np.cos(alpha[i])*np.cos(theta[i]), -np.sin(alpha[i])*np.cos(theta[i]), a[i]*np.sin(theta[i])],
                        [0,                 np.sin(alpha[i]),                   np.cos(alpha[i]),                  d[i]                 ],
                        [0,                 0,                                  0,                                 1                    ]])
        A_i = np.around(A_i, decimal_places)
        T = T@A_i
    return T

def direct_kinematics(q, n):
    theta = q + offset
    T = np.eye(4)
    for i in range(n):
        A_i = np.array([[np.cos(theta[i]), -np.cos(alpha[i])*np.sin(theta[i]),  np.sin(alpha[i])*np.sin(theta[i]), a[i]*np.cos(theta[i])],
                        [np.sin(theta[i]),  np.cos(alpha[i])*np.cos(theta[i]), -np.sin(alpha[i])*np.cos(theta[i]), a[i]*np.sin(theta[i])],
                        [0,                 np.sin(alpha[i]),                   np.cos(alpha[i]),                  d[i]                 ],
                        [0,                 0,                                  0,                                 1                    ]])
        A_i = np.around(A_i, decimal_places)
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

def wp(x, y, z):
    
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

def P1(x, y, z, alpha, beta, gamma, type):
    
    P = np.array([x, y, z])
    print(f"El punto de la herramienta es: {P[0]}, {P[1]}, {P[2]}", end="\n\n")

    print("Matriz de Rotacion:")
    R06 = rotation(alpha, beta, gamma)
    print(R06, end="\n\n")

    na = R06[0:3,2]

    Pm = P - L4*na

    Pmx = Pm[0]
    Pmy = Pm[1]
    Pmz = Pm[2]

    print(f"El punto muñeca es: {Pmx}, {Pmy}, {Pmz}", end="\n\n")

    try:
        # Proyección en el plano XY
        r = math.sqrt(Pmx**2 + Pmy**2)

        # Distancia efectiva desde la base al punto objetivo
        #d = math.sqrt(r**2 + (Pmz - L1)**2)
        
        # Verificación de alcance
        #if d > (L2 + L3):
        #    print(f"El punto ({x}, {y}, {z}) está fuera del alcance.")
        #    return None

        # Cálculo del ángulo θ1
        q1 = math.atan2(Pmy, Pmx)
        
        # Cálculo del ángulo θ2
        a = math.atan2(Pmz - L1, r)
        u = math.sqrt(r**2 + (Pmz - L1)**2)
        q2 = math.pi/2 - (math.acos((L2**2 + u**2 - L3**2) / (2 * L2 * u)) + a)

        # Cálculo del ángulo θ3
        q3 = math.pi - math.acos((L2**2 + L3**2 - u**2) / (2 * L2 * L3))

        q = np.array([q1, q2, q3, 0, 0, 0])
        R03 = direct_kinematics(q, 3)[0:3, 0:3]

        print("Matriz R36:")
        R36 = np.linalg.inv(R03) @ R06
        print(R36, end="\n\n")

        q4 = math.atan2(-R36[1,2], R36[0,2])
        q5 = -math.acos(R36[2,2])
        q6 = math.atan2(-R36[2,1], R36[2,0])

        q = np.array([q1, q2, q3, q4, q5, q6])
    
        print("Matriz de Transformacion:")
        T = direct_model(q)
        print(T, end="\n\n")
        
        # Retornar ángulos en grados
        return math.degrees(q1), math.degrees(q2), math.degrees(q3), math.degrees(q4), math.degrees(q5), math.degrees(q6)
    
    except ValueError as e:
        # Captura de errores matemáticos
        print(f"Error en el cálculo de cinemática inversa: {e}")
        return None

def send_data(event=None):
    s = input_text.get().strip()
    if s:
        if s.startswith("G1"):
            try:
                parts = s.split()
                x, y, z, type = float(parts[1]), float(parts[2]), float(parts[3]), int(parts[4])
                angles = wp(x, y, z)
                if angles:
                    q1, q2, q3 = angles
                    if type == 1:
                        result = f"wp {q1:.{decimal_places}f} {q2:.{decimal_places}f} {q3:.{decimal_places}f}"
                        com.write(result.encode() + b'\n')
                        output_text.insert(tk.END, f" USER >> {result}\n")
                    elif type == 2:
                        default_speed = 10
                        time = [abs(q) / default_speed for q in [q1, q2, q3]]
                        t_max = max(time)
                        adjusted_speeds = [abs(q) / t_max for q in [q1, q2, q3]]
                        result = f"wp {q1:.{decimal_places}f} {q2:.{decimal_places}f} {q3:.{decimal_places}f} {int(adjusted_speeds[0])} {int(adjusted_speeds[1])} {int(adjusted_speeds[2])}"
                        com.write(result.encode() + b'\n')
                        output_text.insert(tk.END, f" USER >> {result}\n")
            except Exception as e:
                output_text.insert(tk.END, f" ERROR >> {e}\n")
        elif s.startswith("STR"):
            STR()
        elif s.startswith("GTR"):
            GTR()
        elif s.startswith("wp"):
            try:
                parts = s.split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                angles = wp(x, y, z)
                if angles:
                    q1, q2, q3 = angles
                    result = f"wp {q1:.2f} {q2:.2f} {q3:.2f}"
                    com.write(result.encode() + b'\n')
                    output_text.insert(tk.END, f" USER >> {result}\n")
                else:
                    output_text.insert(tk.END, " USER >> El punto está fuera del alcance del brazo.\n")
            except Exception as e:
                output_text.insert(tk.END, f" ERROR >> {e}\n")
        else:
            com.write(s.encode() + b'\n')
            output_text.insert(tk.END, " USER >> " + s + "\n")
        input_text.delete(0, tk.END)
        output_text.see(tk.END)

def receive_data():
    while is_running:
        response = com.readline().decode().strip()
        if response:
            output_text.insert(tk.END, " THOR << " + response + "\n")
            output_text.see(tk.END)

def close_app():
    global is_running
    is_running = False
    com.close()
    root.destroy()
    root.quit()

if __name__ == "__main__":
    root = tk.Tk()

    root.title("GUI")
    root.iconbitmap('resources/primogem.ico')
    font_style = ("IBM Plex Mono Italic", 10)

    # Frame para el texto de salida
    text_frame = tk.Frame(root)
    text_frame.pack(fill=tk.BOTH, expand=True)

    output_text = tk.Text(text_frame, bg="black", fg="white", font=font_style, wrap=tk.WORD)
    output_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    scrollbar = tk.Scrollbar(text_frame, command=output_text.yview)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    output_text.config(yscrollcommand=scrollbar.set)

    # Frame para la entrada y el botón "Send"
    input_frame = tk.Frame(root)
    input_frame.pack(fill=tk.X)

    input_text = tk.Entry(input_frame, font=font_style)
    input_text.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.X, expand=True)

    send_button = tk.Button(input_frame, text="Send", command=send_data, font=font_style)
    send_button.pack(side=tk.LEFT, padx=5, pady=5)

    # Asignar la función `send_data` al evento de la tecla `Enter`
    input_text.bind('<Return>', send_data)

    root.protocol("WM_DELETE_WINDOW", close_app)

    main()

    root.mainloop()