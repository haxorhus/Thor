import serial
import threading
import tkinter as tk
import math

PORT = 'COM4'
BAUD_RATE = 115200

com = None
is_running = True
serial_data_receiver = None

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
        else:
            com.write(s.encode() + b'\n')
            output_text.insert(tk.END, " USER >> " + s + "\n")
        input_text.delete(0, tk.END)
        output_text.see(tk.END)

def receive_data():
    global com
    while is_running:
        response = com.readline().decode().strip()
        if response:
            output_text.insert(tk.END, " THOR << " + response + "\n")
            output_text.see(tk.END)
        if not is_running:
            break 

def close_app():
    global is_running, serial_data_receiver

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
        if d > (L2 + L3 + L4):
            return None

        # Cálculo del ángulo θ1
        q1 = math.atan2(y, x)
        
        # Cálculo del ángulo θ2
        a = math.atan2(z - L1, r)
        u = math.sqrt(r**2 + (z - L1)**2)
        q2 = math.pi/2 - (math.acos((L2**2 + u**2 - (L3+L4)**2) / (2 * L2 * u)) + a)

        # Cálculo del ángulo θ3
        q3 = math.pi - math.acos((L2**2 + (L3+L4)**2 - u**2) / (2 * L2 * (L3+L4)))
        
        # Retornar ángulos en grados
        return math.degrees(q1), math.degrees(q2), math.degrees(q3)
    
    except ValueError as e:
        # Captura de errores matemáticos
        print(f"Error en el cálculo de cinemática inversa: {e}")
        return None

if __name__ == "__main__":
    root = tk.Tk()

    root.title("GUI")
    root.iconbitmap('resources/stardew.ico')
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
