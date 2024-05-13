import serial
import threading
import tkinter as tk

PORT = 'COM4'
BAUD_RATE = 9600

com = None
is_running = True
serial_data_receiver = None

def main():

    global com, serial_data_receiver

    com = serial.Serial(PORT, BAUD_RATE)
    serial_data_receiver = threading.Thread(target=receive_data)
    serial_data_receiver.start()

def send_data():
    s = input_text.get().strip()
    if s:
        com.write(s.encode() + b'\n')
        input_text.delete(0, tk.END)
        output_text.insert(tk.END, " USER >> " + s + "\n")
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
