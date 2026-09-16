"""
ESP32 Buck Converter Monitor - Fast Desktop GUI (Ethernet version)
====================================================================
Connects to the ESP32 over a wired Ethernet link for fast, live,
two-way communication. The ESP32 itself does NOT run WiFi anymore -
instead its UART is wired to a USR-TCP232 serial<->Ethernet bridge
module (the small board with the STM32H750 + RJ45 jack). That module
transparently turns everything the ESP32 prints on its UART into TCP
bytes on the network, and vice versa, so this app just opens a normal
TCP socket to the module instead of a WebSocket to the ESP32.

SETUP:
1. No extra pip packages needed - this version uses Python's built-in
   `socket` module only (you can uninstall websocket-client if you like).
2. Configure the USR-TCP232 module ONCE using USR's config tool
   (USR-VCOM / web page at the module's own IP, factory default is
   usually 192.168.0.7):
     - Work Mode: TCP Server
     - Local Port: e.g. 8899
     - A static IP on your LAN (recommended) or note its DHCP IP
     - Serial: 115200 baud, 8 data bits, No parity, 1 stop bit
       (must match ETH_BAUD in the .ino sketch)
3. Change MODULE_IP / MODULE_PORT below to match that configuration.
4. Run: python kedar_esp1_ethernet.py
"""

import tkinter as tk
import threading
import json
import socket

# ================= CONFIG =================
MODULE_IP = "192.168.0.7"     # <-- CHANGE THIS to the USR-TCP232 module's configured IP
MODULE_PORT = 8899             # <-- CHANGE THIS to match the module's configured Local Port
GUI_REFRESH_MS = 30            # How often the on-screen labels repaint (screens can't usefully
                                # redraw faster than this anyway, even if the ESP32 sends every 8ms)


class ESP32MonitorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("ESP32 Buck Converter Monitor (Ethernet)")
        self.root.configure(bg="#111111")
        self.root.geometry("420x480")

        self.sock = None
        self._sock_lock = threading.Lock()
        self.labels = {}

        # Latest parsed data, updated as fast as packets arrive (every ~8ms).
        # The GUI itself only reads this on its own fixed timer (see
        # _schedule_gui_refresh), so a fast network doesn't flood Tkinter
        # with more repaint requests than the screen can actually show.
        self._latest_data = None
        self._data_lock = threading.Lock()

        self._build_ui()

        # Connect to the module in a background thread so the GUI never freezes
        threading.Thread(target=self._connect_loop, daemon=True).start()

        # Start the fixed-rate GUI repaint loop
        self._schedule_gui_refresh()

    def _build_ui(self):
        title = tk.Label(
            self.root, text="ESP32 Buck Converter Monitor",
            fg="#0af", bg="#111111", font=("Segoe UI", 14, "bold")
        )
        title.pack(pady=(15, 5))

        self.status_label = tk.Label(
            self.root, text="Connecting...", fg="orange",
            bg="#111111", font=("Segoe UI", 10)
        )
        self.status_label.pack(pady=(0, 15))

        # --- Local dashboard section ---
        self._section_header("Dashboard (Local)")
        self._value_row("Output Status", "outputActive")
        self._value_row("Output Voltage", "voltage", " V")
        self._value_row("Output Current", "current", " A")
        self._value_row("Output Power", "power", " W")
        self._value_row("Target Vref", "vref", " V")

        # --- C2000 section ---
        self._section_header("C2000 Board Data")
        self._value_row("Voltage", "c2000Voltage", " V")
        self._value_row("Current", "c2000Current", " A")
        self._value_row("Power", "c2000Power", " W")

        # --- Controls ---
        self._section_header("Controls")

        status_row = tk.Frame(self.root, bg="#111111")
        status_row.pack(fill="x", padx=20, pady=(4, 8))

        tk.Label(status_row, text="Page:", fg="#ccc", bg="#111111",
                 font=("Segoe UI", 9)).pack(side="left")
        self.page_label = tk.Label(status_row, text="--", fg="#fce000",
                                    bg="#111111", font=("Segoe UI", 9, "bold"))
        self.page_label.pack(side="left", padx=(4, 20))

        tk.Label(status_row, text="Cursor digit:", fg="#ccc", bg="#111111",
                 font=("Segoe UI", 9)).pack(side="left")
        self.cursor_label = tk.Label(status_row, text="--", fg="#fce000",
                                      bg="#111111", font=("Segoe UI", 9, "bold"))
        self.cursor_label.pack(side="left", padx=4)

        btn_frame = tk.Frame(self.root, bg="#111111")
        btn_frame.pack(pady=8)

        # Row 1: ON/OFF and WAVE (page switch) - same as physical buttons
        self._make_button(btn_frame, "Toggle ON/OFF", self.toggle_output, 0, 0)
        self._make_button(btn_frame, "Next Page (WAVE)", self.next_page, 0, 1)

        # Row 2: CURSOR, INC, DEC - same as physical buttons
        self._make_button(btn_frame, "Move Cursor", self.move_cursor, 1, 0)
        self._make_button(btn_frame, "Increment (+)", self.inc_digit, 1, 1)
        self._make_button(btn_frame, "Decrement (-)", self.dec_digit, 2, 0)

        # Row 3: SAVE - same as physical button
        self._make_button(btn_frame, "Save", self.save_vref, 2, 1, bg="#0af", fg="#000")

        tk.Label(
            self.root, text="Direct Vref entry:", fg="#9cf", bg="#111111",
            font=("Segoe UI", 10)
        ).pack(pady=(14, 2))

        entry_frame = tk.Frame(self.root, bg="#111111")
        entry_frame.pack(pady=2)

        self.vref_entry = tk.Entry(entry_frame, width=8, font=("Segoe UI", 10))
        self.vref_entry.pack(side="left", padx=(0, 6))

        tk.Button(
            entry_frame, text="Send Vref", command=self.set_vref,
            bg="#333", fg="#fff", font=("Segoe UI", 9),
            relief="flat", padx=8, pady=3
        ).pack(side="left")

    def _make_button(self, parent, text, command, row, col, bg="#333", fg="#fff"):
        btn = tk.Button(
            parent, text=text, command=command,
            bg=bg, fg=fg, font=("Segoe UI", 9, "bold"),
            relief="flat", width=16, pady=6
        )
        btn.grid(row=row, column=col, padx=6, pady=4)

    def _section_header(self, text):
        header = tk.Label(
            self.root, text=text, fg="#9cf", bg="#111111",
            font=("Segoe UI", 11, "bold")
        )
        header.pack(anchor="w", padx=20, pady=(10, 2))
        separator = tk.Frame(self.root, bg="#333333", height=1)
        separator.pack(fill="x", padx=20)

    def _value_row(self, label_text, key, unit=""):
        row = tk.Frame(self.root, bg="#111111")
        row.pack(fill="x", padx=20, pady=2)

        tk.Label(
            row, text=label_text, fg="#ccc", bg="#111111",
            font=("Segoe UI", 10), width=16, anchor="w"
        ).pack(side="left")

        val_label = tk.Label(
            row, text="--", fg="#0f0", bg="#111111",
            font=("Segoe UI", 10, "bold"), anchor="w"
        )
        val_label.pack(side="left")

        self.labels[key] = (val_label, unit)

    # ================= TCP SOCKET HANDLING =================
    def _connect_loop(self):
        """Keeps trying to (re)connect to the USR-TCP232 module, same
        reconnect-forever spirit as the old WebSocket version's
        run_forever(reconnect=5)."""
        while True:
            try:
                self.root.after(0, lambda: self.status_label.config(
                    text="Connecting...", fg="orange"))

                s = socket.create_connection((MODULE_IP, MODULE_PORT), timeout=5)
                s.settimeout(None)  # Back to blocking mode for the read loop below

                with self._sock_lock:
                    self.sock = s

                self.root.after(0, lambda: self.status_label.config(
                    text="Connected", fg="#0f0"))

                self._read_loop(s)  # Blocks here until the connection drops

            except (OSError, socket.timeout) as e:
                self.root.after(0, lambda e=e: self.status_label.config(
                    text=f"Error: {e}", fg="red"))

            with self._sock_lock:
                if self.sock is not None:
                    try:
                        self.sock.close()
                    except OSError:
                        pass
                self.sock = None

            self.root.after(0, lambda: self.status_label.config(
                text="Disconnected - retrying...", fg="red"))

            import time
            time.sleep(2)   # Wait a couple seconds before retrying, like reconnect=5 did

    def _read_loop(self, s):
        """Reads newline-terminated JSON lines the ESP32 streams out (every
        ~8ms) and stores the latest parsed dict. Runs as fast as the network
        allows - the GUI repaints separately on its own timer, see
        _schedule_gui_refresh()."""
        buffer = ""
        while True:
            chunk = s.recv(4096)
            if not chunk:
                return  # Connection closed by the module/ESP32
            buffer += chunk.decode("utf-8", errors="ignore")

            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                line = line.strip()
                if not line:
                    continue
                try:
                    data = json.loads(line)
                except json.JSONDecodeError:
                    continue  # Ignore any malformed/partial packets

                with self._data_lock:
                    self._latest_data = data

    def _schedule_gui_refresh(self):
        """Repaints the labels from the latest received packet, then
        reschedules itself. Runs on a fixed cadence (GUI_REFRESH_MS)
        independent of how fast the ESP32 is actually sending data."""
        with self._data_lock:
            data = self._latest_data
        if data is not None:
            self._update_labels(data)
        self.root.after(GUI_REFRESH_MS, self._schedule_gui_refresh)

    def _update_labels(self, data):
        for key, (label, unit) in self.labels.items():
            if key not in data:
                continue
            value = data[key]
            if key == "outputActive":
                label.config(text="ON" if value else "OFF",
                             fg="#0f0" if value else "#777")
            else:
                label.config(text=f"{value}{unit}")

        if "page" in data:
            self.page_label.config(text=data["page"])
        if "cursorIndex" in data:
            self.cursor_label.config(text=str(data["cursorIndex"]))

    # ================= SENDING COMMANDS =================
    def _send(self, command):
        with self._sock_lock:
            s = self.sock
        if s is None:
            return
        try:
            s.sendall((command + "\n").encode())  # Newline-delimited, matches checkEthernetInput()
        except OSError:
            pass  # Connection likely dropped; the reconnect loop will handle it

    def toggle_output(self):
        self._send("TOGGLE_OUTPUT")   # Same as pressing physical BTN_ONOFF

    def next_page(self):
        self._send("NEXT_PAGE")       # Same as pressing physical BTN_WAVE

    def move_cursor(self):
        self._send("MOVE_CURSOR")     # Same as pressing physical BTN_CURSOR

    def inc_digit(self):
        self._send("INC_DIGIT")       # Same as pressing physical BTN_INC

    def dec_digit(self):
        self._send("DEC_DIGIT")       # Same as pressing physical BTN_DEC

    def save_vref(self):
        self._send("SAVE")            # Same as pressing physical BTN_SAVE

    def set_vref(self):
        try:
            value = float(self.vref_entry.get())
        except ValueError:
            return  # Ignore invalid input silently
        self._send(f"SET_VREF:{value:.2f}")


if __name__ == "__main__":
    root = tk.Tk()
    app = ESP32MonitorApp(root)
    root.mainloop()