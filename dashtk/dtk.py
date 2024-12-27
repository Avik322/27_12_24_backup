import serial
import sqlite3
import threading
from datetime import datetime
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

humidity_data = []
temperature_data = []
ec_data = []
timestamps = []
green_band_width_hum = 0.0
green_band_center_hum = 0.0
green_band_width_temp = 0.0
green_band_center_temp = 0.0
green_band_width_ec = 0.0
green_band_center_ec = 0.0

def read_com_port():
    ser = serial.Serial('COM12', baudrate=115200, timeout=1)
    try:
        while True:
            if ser.in_waiting > 0:
                pre_data = ser.readline().decode('utf-8').strip()
                data = parse_data_from(pre_data)
                add_inf_db(data)
                get_last_15_from_db(current_dev_id)
    except KeyboardInterrupt:
        print("Программа завершена")
    finally:
        ser.close()


def parse_data_from(data):
    pre_parts = data.split(': ')
    if len(pre_parts) > 1:
        parts = pre_parts[1].split(' ')
        if len(parts) >= 4:
            return parts
    return []


def create_data_base():
    connection = sqlite3.connect("sensor_data")
    cursor = connection.cursor()
    cursor.execute(''' 
        CREATE TABLE IF NOT EXISTS sensor_data (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            dev_id INTEGER,
            humidity REAL,
            temperature REAL,
            ec REAL,
            timestamp TEXT
        )
    ''')
    connection.commit()
    connection.close()


def add_inf_db(data):
    if len(data) < 4:
        print("Недостаточно данных для записи в базу:", data)
        return

    connection = sqlite3.connect("sensor_data")
    cursor = connection.cursor()
    try:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        cursor.execute(
            "INSERT INTO sensor_data(dev_id, humidity, temperature, ec, timestamp) VALUES (?, ?, ?, ?, ?)",
            (int(data[0]), float(data[1]), float(data[2]), float(data[3]), timestamp)
        )
        connection.commit()
    except ValueError as e:
        print("Ошибка в данных:", e, data)
    finally:
        connection.close()


def get_last_15_from_db(dev_id):
    global humidity_data, temperature_data, ec_data, timestamps
    connection = sqlite3.connect("sensor_data")
    cursor = connection.cursor()
    cursor.execute("""
        SELECT humidity, temperature, ec, timestamp
        FROM sensor_data
        WHERE dev_id = ?
        ORDER BY id DESC
        LIMIT 15
    """, (dev_id,))
    rows = cursor.fetchall()
    connection.close()
    humidity_data.clear()
    temperature_data.clear()
    ec_data.clear()
    timestamps.clear()
    for row in reversed(rows):
        humidity_data.append(row[0])
        temperature_data.append(row[1])
        ec_data.append(row[2])
        timestamps.append(row[3])


def get_all_device_ids():
    connection = sqlite3.connect("sensor_data")
    cursor = connection.cursor()
    cursor.execute("SELECT DISTINCT dev_id FROM sensor_data")
    device_ids = [row[0] for row in cursor.fetchall()]
    connection.close()
    return device_ids


def get_all_data_from_db():
    connection = sqlite3.connect("sensor_data")
    cursor = connection.cursor()
    cursor.execute("""
        SELECT dev_id, humidity, temperature, ec, timestamp
        FROM sensor_data
        ORDER BY id DESC
    """)
    rows = cursor.fetchall()
    connection.close()
    return rows


def animate(i, axes):
    get_last_15_from_db(current_dev_id)
    if len(humidity_data) > 0:
        for ax in axes.flatten():
            ax.clear()
        axes[0, 0].plot(timestamps, humidity_data, label="Humidity", marker='o')
        lower_bound_hum  = green_band_center_hum - green_band_width_hum / 2
        upper_bound_hum  = green_band_center_hum + green_band_width_hum / 2
        axes[0, 0].fill_between(timestamps, lower_bound_hum , upper_bound_hum , color='lightgreen', alpha=0.5)
        axes[0, 0].set_title("Humidity")
        axes[0, 0].set_ylabel("Humidity (%)")
        axes[0, 0].grid(True)
        axes[0, 0].legend()
        axes[0, 0].set_xticks(range(len(timestamps)))
        axes[0, 0].set_xticklabels(timestamps, rotation=45, fontsize=4)

        axes[0, 1].plot(timestamps, temperature_data, label="Temperature", marker='o')
        lower_bound_temp = green_band_center_temp - green_band_width_temp / 2
        upper_bound_temp = green_band_center_temp + green_band_width_temp / 2
        axes[0, 1].fill_between(timestamps, lower_bound_temp, upper_bound_temp, color='lightgreen', alpha=0.5)
        axes[0, 1].set_title("Temperature")
        axes[0, 1].set_ylabel("Temperature (°C)")
        axes[0, 1].grid(True)
        axes[0, 1].legend()
        axes[0, 1].set_xticks(range(len(timestamps)))
        axes[0, 1].set_xticklabels(timestamps, rotation=45, fontsize=4)

        axes[1, 0].plot(timestamps, ec_data, label="EC", marker='s')
        lower_bound_ec = green_band_center_ec - green_band_width_ec / 2
        upper_bound_ec = green_band_center_ec + green_band_width_ec / 2
        axes[1, 0].fill_between(timestamps, lower_bound_ec, upper_bound_ec, color='lightgreen', alpha=0.5)
        axes[1, 0].set_title("EC")
        axes[1, 0].set_ylabel("EC (µS/cm)")
        axes[1, 0].grid(True)
        axes[1, 0].legend()
        axes[1, 0].set_xticks(range(len(timestamps)))
        axes[1, 0].set_xticklabels(timestamps, rotation=45, fontsize=4)

        axes[1, 1].axis('off')
        plt.tight_layout()


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("Sensor Data Viewer")

        self.notebook = ttk.Notebook(root)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.graphs_tab = ttk.Frame(self.notebook)
        self.database_tab = ttk.Frame(self.notebook)
        self.device_tab = ttk.Frame(self.notebook)

        self.notebook.add(self.graphs_tab, text="Graphs")
        self.notebook.add(self.database_tab, text="Database")
        self.notebook.add(self.device_tab, text="Device Data")

        self.setup_graphs_tab()
        self.setup_database_tab()
        self.setup_device_tab()

    def setup_graphs_tab(self):
        frame = tk.Frame(self.graphs_tab)
        frame.pack(fill=tk.BOTH, expand=True)

        self.figure, self.axes = plt.subplots(2, 2, figsize=(10, 6))
        self.canvas = FigureCanvasTkAgg(self.figure, master=frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        inputs_frame1 = tk.Frame(frame)
        inputs_frame1.pack(pady = 10)

        tk.Label(inputs_frame1, text = "Center humidity:").grid(row = 0, column = 1, padx = 5, pady = 2)
        self.center_humidity_entry = tk.Entry(inputs_frame1, width = 10)
        self.center_humidity_entry.grid(row = 1, column = 1, padx = 5)
        tk.Label(inputs_frame1, text = "Width humidity:").grid(row = 2, column = 1, padx = 5, pady = 2)
        self.width_humidity_entry = tk.Entry(inputs_frame1, width = 10)
        self.width_humidity_entry.grid(row = 3, column = 1, padx = 5)

        tk.Label(inputs_frame1, text = "Center temperature:").grid(row = 0, column = 2, padx = 5, pady = 2)
        self.center_temperature_entry = tk.Entry(inputs_frame1, width = 10)
        self.center_temperature_entry.grid(row = 1, column = 2, padx = 5)
        tk.Label(inputs_frame1, text = "Width temperature:").grid(row = 2, column = 2, padx = 5, pady = 2)
        self.width_temperature_entry = tk.Entry(inputs_frame1, width = 10)
        self.width_temperature_entry.grid(row = 3, column = 2, padx = 5)

        tk.Label(inputs_frame1, text = "Center EC:").grid(row = 0, column = 3, padx = 5, pady = 2)
        self.center_ec_entry = tk.Entry(inputs_frame1, width = 10)
        self.center_ec_entry.grid(row = 1, column = 3, padx = 5)
        tk.Label(inputs_frame1, text = "Width EC:").grid(row = 2, column = 3, padx = 5, pady = 2)
        self.width_ec_entry = tk.Entry(inputs_frame1, width = 10)
        self.width_ec_entry.grid(row = 3, column = 3, padx = 5)


        self.update_boundaries_button = tk.Button(inputs_frame1, text="Update Boundaries", command=self.update_boundaries)
        self.update_boundaries_button.grid(row = 2, column =6, pady=10)

        device_ids = get_all_device_ids()
        self.device_combo = ttk.Combobox(inputs_frame1, values=device_ids)
        self.device_combo.grid(row=2, column=0, padx=5, pady=5)
        self.device_combo.bind("<<ComboboxSelected>>", self.update_graphs)

        self.ani = FuncAnimation(self.figure, animate, fargs=(self.axes,), interval=1000)

    def update_boundaries(self):
        try:
            # Получаем значения из полей ввода
            global green_band_center_hum, green_band_width_hum
            global green_band_center_temp, green_band_width_temp
            global green_band_center_ec, green_band_width_ec

            green_band_center_hum = float(self.center_humidity_entry.get())
            green_band_width_hum = float(self.width_humidity_entry.get())
            green_band_center_temp = float(self.center_temperature_entry.get())
            green_band_width_temp = float(self.width_temperature_entry.get())
            green_band_center_ec = float(self.center_ec_entry.get())
            green_band_width_ec = float(self.width_ec_entry.get())

            # Обновляем графики
            self.update_graphs()
        except ValueError:
            messagebox.showerror("Error", "Please enter valid numerical values for all fields.")

    def setup_database_tab(self):
        frame = tk.Frame(self.database_tab)
        frame.pack(fill=tk.BOTH, expand=True)

        self.tree = ttk.Treeview(frame, columns=("Device ID", "Humidity", "Temperature", "EC", "Timestamp"), show="headings")
        self.tree.heading("Device ID", text="Device ID")
        self.tree.heading("Humidity", text="Humidity")
        self.tree.heading("Temperature", text="Temperature")
        self.tree.heading("EC", text="EC")
        self.tree.heading("Timestamp", text="Timestamp")
        self.tree.pack(fill=tk.BOTH, expand=True)

        button_frame = tk.Frame(frame)
        button_frame.pack(fill=tk.X)

        clear_button = tk.Button(button_frame, text="Clear Database", command=self.clear_database)
        clear_button.pack(side=tk.LEFT)

        refresh_button = tk.Button(button_frame, text="Refresh", command=self.load_database)
        refresh_button.pack(side=tk.LEFT)

        self.load_database()

    def load_database(self):
        for i in self.tree.get_children():
            self.tree.delete(i)

        rows = get_all_data_from_db()
        for row in rows:
            self.tree.insert("", tk.END, values=row)

    def clear_database(self):
        connection = sqlite3.connect("sensor_data")
        cursor = connection.cursor()
        cursor.execute("DELETE FROM sensor_data")
        connection.commit()
        connection.close()
        self.load_database()

    def update_graphs(self, event=None):
        global current_dev_id
        current_dev_id = int(self.device_combo.get())
        self.canvas.draw()

    def setup_device_tab(self):
        frame = tk.Frame(self.device_tab)
        frame.pack(fill=tk.BOTH, expand=True)

        # Выпадающий список для выбора Device
        filter_frame = tk.Frame(frame)
        filter_frame.pack(fill=tk.X, padx=10, pady=5)

        tk.Label(filter_frame, text="Select Device ID:").pack(side=tk.LEFT, padx=5)

        # Получаем список всех уникальных Device из базы данных
        device_ids = get_all_device_ids()
        self.device_id_combo = ttk.Combobox(filter_frame, values=device_ids, state="readonly")
        self.device_id_combo.pack(side=tk.LEFT, padx=5)
        self.device_id_combo.bind("<<ComboboxSelected>>", self.filter_by_device_id)

        # Кнопка для очистки фильтра и отображения всех данных
        clear_filter_button = tk.Button(filter_frame, text="Show All Devices", command=self.load_all_devices)
        clear_filter_button.pack(side=tk.LEFT, padx=5)

        # Таблица для отображения данных
        self.device_tree = ttk.Treeview(frame, columns=("Device ID", "Humidity", "Temperature", "EC", "Timestamp"),
                                        show="headings")
        self.device_tree.heading("Device ID", text="Device ID")
        self.device_tree.heading("Humidity", text="Humidity")
        self.device_tree.heading("Temperature", text="Temperature")
        self.device_tree.heading("EC", text="EC")
        self.device_tree.heading("Timestamp", text="Timestamp")
        self.device_tree.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Загрузить все данные при инициализации
        self.load_all_devices()

    def load_all_devices(self):
        # Очистить таблицу перед загрузкой новых данных
        for i in self.device_tree.get_children():
            self.device_tree.delete(i)

        # Получить все данные из базы
        rows = get_all_data_from_db()

        # Заполнить таблицу данными
        for row in rows:
            self.device_tree.insert("", tk.END, values=row)

    def get_data_by_device_id(self, device_id):
        connection = sqlite3.connect("sensor_data")
        cursor = connection.cursor()
        query = "SELECT dev_id, humidity, temperature, ec, timestamp FROM sensor_data WHERE dev_id = ?"
        cursor.execute(query, (device_id,))
        rows = cursor.fetchall()
        connection.close()
        return rows

    def filter_by_device_id(self, event=None):
        selected_id = self.device_id_combo.get()
        if selected_id:
            try:
                self.load_data_for_device(int(selected_id))
            except ValueError:
                messagebox.showerror("Error", "Invalid Device ID selected")

    def load_data_for_device(self, device_id):
        # Очистить таблицу перед загрузкой данных
        for i in self.device_tree.get_children():
            self.device_tree.delete(i)

        # Получить данные только для выбранного Device ID
        rows = self.get_data_by_device_id(device_id)


        # Заполнить таблицу отфильтрованными данными
        for row in rows:
            self.device_tree.insert("", tk.END, values=row)

    def load_all_devices(self):
        # Очистить текущую таблицу
        for i in self.device_tree.get_children():
            self.device_tree.delete(i)

        # Получить все данные из базы
        rows = get_all_data_from_db()  # Эта функция должна быть определена отдельно

        # Заполнить таблицу данными
        for row in rows:
            self.device_tree.insert("", tk.END, values=row)




if __name__ == "__main__":
    create_data_base()
    current_dev_id = 18
    root = tk.Tk()
    app = App(root)
    threading.Thread(target=read_com_port, daemon=True).start()
    root.mainloop()
