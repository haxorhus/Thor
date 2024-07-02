import serial
import threading
import tkinter as tk
import math
import numpy as np
from scipy.interpolate import CubicSpline
import matplotlib.pyplot as plt

PORT = 'COM4'
BAUD_RATE = 115200

com = None
is_running = True
serial_data_receiver = None
interpolation_points = []
waiting_for_done = False

# Parámetros geométricos (en mm)
L1 = 202.0
L2 = 160.0
L3 = 195.0
L4 = 67.15

def main():
    global com, serial_data_receiver

    com = serial.Serial(PORT, BAUD_RATE)
    serial_data_receiver = threading.Thread(target=receive_data)
    serial_data_receiver.start()

def send_data():
    s = input_text.get().strip()
    if s:
        if s.startswith("wp"):
            try:
                parts = s.split()
                x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                angles = inverse_kinematics(x, y, z)
                if angles:
                    q1, q2, q3 = angles
                    result = f"wp {q1:.2f} {q2:.2f} {q3:.2f}"
                    com.write(result.encode() + b'\n')
                    output_text.insert(tk.END, f" USER >> {result}\n")
                else:
                    output_text.insert(tk.END, " USER >> El punto está fuera del alcance del brazo.\n")
            except Exception as e:
                output_text.insert(tk.END, f" ERROR >> {e}\n")
        elif s.startswith("t"):
            try:
                parts = s.split()
                x1, y1, z1 = float(parts[1]), float(parts[2]), float(parts[3])
                x2, y2, z2 = float(parts[4]), float(parts[5]), float(parts[6])
                angles1 = inverse_kinematics(x1, y1, z1)
                angles2 = inverse_kinematics(x2, y2, z2)
                if angles1 and angles2:
                    q11, q12, q13 = angles1
                    q21, q22, q23 = angles2
                    interpolate_trajectory([q11, q12, q13], [q21, q22, q23])
                    send_next_interpolation_point()
                else:
                    output_text.insert(tk.END, " USER >> Uno de los puntos está fuera del alcance del brazo.\n")
            except Exception as e:
                output_text.insert(tk.END, f" ERROR >> {e}\n")
        else:
            com.write(s.encode() + b'\n')
            output_text.insert(tk.END, " USER >> " + s + "\n")
        input_text.delete(0, tk.END)
        output_text.see(tk.END)

def send_next_interpolation_point():
    global interpolation_points, waiting_for_done
    if interpolation_points and not waiting_for_done:
        q1, q2, q3 = interpolation_points.pop(0)
        default_speed = 10

        # Calcular tiempos t
        times = [abs(q) / default_speed for q in [q1, q2, q3]]

        # Determinar el tiempo máximo
        t_max = max(times)

        # Calcular velocidades ajustadas v = [q1/t_max, q2/t_max, q3/t_max]
        adjusted_speeds = [abs(q) / t_max for q in [q1, q2, q3]]

        # Crear el comando serial
        result = f"wp {q1:.2f} {q2:.2f} {q3:.2f} {int(adjusted_speeds[0])} {int(adjusted_speeds[1])} {int(adjusted_speeds[2])}"
        com.write(result.encode() + b'\n')
        output_text.insert(tk.END, f" USER >> {result}\n")
        waiting_for_done = True

def receive_data():
    global waiting_for_done
    while is_running:
        response = com.readline().decode().strip()
        if response:
            output_text.insert(tk.END, " THOR << " + response + "\n")
            output_text.see(tk.END)
            if response == "done":
                waiting_for_done = False
                send_next_interpolation_point()

def close_app():
    global is_running
    is_running = False
    com.close()
    root.destroy()
    root.quit()

def inverse_kinematics(x, y, z):
    try:
        # Proyección en el plano XY
        r = math.sqrt(x**2 + y**2)
        # Distancia efectiva desde la base al punto objetivo
        d = math.sqrt(r**2 + (z - L1)**2)
        
        # Verificación de alcance
        if d > (L2 + L3):
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

    root.protocol("WM_DELETE_WINDOW", close_app)

    main()

    root.mainloop()
