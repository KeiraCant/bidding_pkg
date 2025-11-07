#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import tkinter as tk
from tkinter import scrolledtext
from threading import Thread
import json
import re
from collections import defaultdict

class FireUIDashboard(Node):
    def __init__(self):
        super().__init__('fire_ui_dashboard')

        # ---------------- ROS Setup ----------------
        qos = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(String, '/fire_planner_log', self.log_callback, qos)
        
        # ---------------- Data Storage ----------------
        self.log_messages = []
        self.downstream_tasks = []

        # Store messages by drone_id
        self.controller_messages_by_drone = defaultdict(list)
        self.mission_progress_by_drone = defaultdict(str)

        # ---------------- GUI Setup ----------------
        self.root = tk.Tk()
        self.root.title("Fire Planner Dashboard")
        self.root.configure(bg='black')
        # Linux-friendly fullscreen
        self.root.attributes('-zoomed', True)

        # -------- Setup Header --------
        self.setup_header_frame = tk.Frame(self.root, bg='black', height=30)
        self.setup_header_frame.pack(side='top', fill='x')

        self.setup_label = tk.Label(
            self.setup_header_frame,
            text="Setup: Fire Planner Dashboard",
            bg='black',
            fg='white',
            font=("Helvetica", 10, "italic")
        )
        self.setup_label.pack(side='left', padx=10, pady=5)

        # ---------------- Layout Frames ----------------
        self.main_frame = tk.Frame(self.root, bg='black')
        self.main_frame.pack(fill='both', expand=True)

        # Top-left: Mission Progress (1/4 width, 1/4 height)
        self.mission_frame = tk.Frame(self.main_frame, bg='black', bd=2, relief='groove')
        self.mission_frame.place(relx=0, rely=0, relwidth=0.25, relheight=0.25)

        # Below Mission: Controller (1/4 width, 3/4 height)
        self.controller_frame = tk.Frame(self.main_frame, bg='black', bd=2, relief='groove')
        self.controller_frame.place(relx=0, rely=0.25, relwidth=0.25, relheight=0.75)

        # Right side: General Log (3/8 width, full height)
        self.general_frame = tk.Frame(self.main_frame, bg='black', bd=2, relief='groove')
        self.general_frame.place(relx=0.25, rely=0, relwidth=0.375, relheight=1.0)

        # Right-most: Downstream Tasks (3/8 width, full height)
        self.downstream_frame = tk.Frame(self.main_frame, bg='black', bd=2, relief='groove')
        self.downstream_frame.place(relx=0.625, rely=0, relwidth=0.375, relheight=1.0)

        # ---------------- Mission Progress Area Setup ----------------
        tk.Label(self.mission_frame, text="Mission Progress", bg='black', fg='white', font=("Helvetica", 14, "bold")).pack(pady=5)

        # Scrollable canvas/frame to hold multiple mission progress boxes
        self.mission_canvas = tk.Canvas(self.mission_frame, bg='black', highlightthickness=0)
        self.mission_scrollbar = tk.Scrollbar(self.mission_frame, orient="vertical", command=self.mission_canvas.yview)
        self.mission_inner_frame = tk.Frame(self.mission_canvas, bg='black')

        self.mission_inner_frame.bind(
            "<Configure>",
            lambda e: self.mission_canvas.configure(
                scrollregion=self.mission_canvas.bbox("all")
            )
        )

        self.mission_canvas.create_window((0, 0), window=self.mission_inner_frame, anchor="nw")
        self.mission_canvas.configure(yscrollcommand=self.mission_scrollbar.set)

        self.mission_canvas.pack(side="left", fill="both", expand=True)
        self.mission_scrollbar.pack(side="right", fill="y")

        # Map drone_id -> label widget for mission progress
        self.mission_boxes = {}

        # ---------------- Controller Area Setup ----------------
        tk.Label(self.controller_frame, text="Controller", bg='black', fg='white', font=("Helvetica", 14, "bold")).pack(pady=5)

        # Scrollable canvas/frame to hold multiple controller boxes
        self.controller_canvas = tk.Canvas(self.controller_frame, bg='black', highlightthickness=0)
        self.controller_scrollbar = tk.Scrollbar(self.controller_frame, orient="vertical", command=self.controller_canvas.yview)
        self.controller_inner_frame = tk.Frame(self.controller_canvas, bg='black')

        self.controller_inner_frame.bind(
            "<Configure>",
            lambda e: self.controller_canvas.configure(
                scrollregion=self.controller_canvas.bbox("all")
            )
        )

        self.controller_canvas.create_window((0, 0), window=self.controller_inner_frame, anchor="nw")
        self.controller_canvas.configure(yscrollcommand=self.controller_scrollbar.set)

        self.controller_canvas.pack(side="left", fill="both", expand=True)
        self.controller_scrollbar.pack(side="right", fill="y")

        # Map drone_id -> scrolledtext box widget
        self.controller_boxes = {}

        # ---------------- General Log ----------------
        tk.Label(self.general_frame, text="General Log", bg='black', fg='white', font=("Helvetica", 14, "bold")).pack(pady=5)
        self.log_box = scrolledtext.ScrolledText(self.general_frame, bg='black', fg='white', font=("Helvetica", 12), state='disabled')
        self.log_box.pack(fill='both', expand=True, padx=5, pady=5)

        # ---------------- Downstream Tasks ----------------
        tk.Label(self.downstream_frame, text="Fire Data", bg='black', fg='white', font=("Helvetica", 14, "bold")).pack(pady=5)
        self.downstream_box = scrolledtext.ScrolledText(self.downstream_frame, bg='black', fg='yellow', font=("Helvetica", 12), state='disabled')
        self.downstream_box.pack(fill='both', expand=True, padx=5, pady=5)

        # ---------------- ROS Spin Thread ----------------
        self.ros_thread = Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # Start GUI update loop
        self.root.after(500, self.update_gui)
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.mainloop()

    # ---------------- ROS Callbacks ----------------
    def log_callback(self, msg):
        text = msg.data.strip()

        # Parse drone-tagged messages: [drone_id] ...
        match = re.match(r'^\[(.+?)\] (.*)$', text)
        if match:
            drone_id = match.group(1)
            content = match.group(2)

            # Check if it's a mission progress message
            if content.startswith("[MISSION]"):
                mission_text = content[len("[MISSION] "):]
                self.mission_progress_by_drone[drone_id] = mission_text
                return

            # Check if it's a controller message
            elif content.startswith("[Controller]"):
                controller_msg = content[len("[Controller] "):]
                self.controller_messages_by_drone[drone_id].append(controller_msg)
                if len(self.controller_messages_by_drone[drone_id]) > 100:
                    self.controller_messages_by_drone[drone_id] = self.controller_messages_by_drone[drone_id][-100:]
                return

            # Check if it's downstream/wind/rain data
            elif content.startswith("[DOWNSTREAM]") or content.startswith("[Wind]") or content.startswith("[Rain]"):
                data_content = content.split(" ", 1)[1] if " " in content else ""
                try:
                    parsed = json.loads(data_content)
                    if isinstance(parsed, list):
                        self.downstream_tasks.extend(parsed)
                    else:
                        self.downstream_tasks.append(str(parsed))
                except:
                    self.downstream_tasks.append(data_content)
                self.downstream_tasks = self.downstream_tasks[-100:]
                return

        # Otherwise treat as general log
        self.log_messages.append(text)
        if len(self.log_messages) > 200:
            self.log_messages = self.log_messages[-200:]

    # ---------------- GUI Update ----------------
    def update_gui(self):
        # Mission Progress - update each drone box
        for drone_id, progress_text in self.mission_progress_by_drone.items():
            if drone_id not in self.mission_boxes:
                # Create new frame and label for this drone's mission progress
                frame = tk.Frame(self.mission_inner_frame, bg='black', bd=1, relief='sunken')
                title_label = tk.Label(frame, text=f"Mission: {drone_id}", bg='black', fg='green', font=("Helvetica", 11, "bold"))
                title_label.pack(fill='x')
                
                progress_label = tk.Label(frame, text="No updates yet", bg='black', fg='green', font=("Helvetica", 10), wraplength=200, justify='left')
                progress_label.pack(fill='both', expand=True, padx=5, pady=5)
                
                frame.pack(fill='both', expand=True, pady=5, padx=5)
                self.mission_boxes[drone_id] = progress_label

            # Update the progress text
            self.mission_boxes[drone_id].configure(text=progress_text if progress_text else "No updates yet")

        # Controller - update each drone box
        for drone_id, messages in self.controller_messages_by_drone.items():
            if drone_id not in self.controller_boxes:
                # Create new label and scrolledtext for this drone
                frame = tk.Frame(self.controller_inner_frame, bg='black', bd=1, relief='sunken')
                label = tk.Label(frame, text=f"Controller: {drone_id}", bg='black', fg='cyan', font=("Helvetica", 12, "bold"))
                label.pack(fill='x')
                text_box = scrolledtext.ScrolledText(frame, bg='black', fg='cyan', font=("Helvetica", 10), height=10, state='disabled')
                text_box.pack(fill='both', expand=True)
                frame.pack(fill='both', expand=True, pady=5)
                self.controller_boxes[drone_id] = text_box

            box = self.controller_boxes[drone_id]
            box.configure(state='normal')
            box.delete(1.0, tk.END)
            box.insert(tk.END, "\n".join(messages[-50:]))
            box.configure(state='disabled')
            box.see(tk.END)

        # General Log
        self.log_box.configure(state='normal')
        self.log_box.delete(1.0, tk.END)
        self.log_box.insert(tk.END, "\n".join(self.log_messages[-200:]))
        self.log_box.configure(state='disabled')
        self.log_box.see(tk.END)

        # Downstream
        self.downstream_box.configure(state='normal')
        self.downstream_box.delete(1.0, tk.END)
        self.downstream_box.insert(tk.END, "\n".join(self.downstream_tasks[-100:]))
        self.downstream_box.configure(state='disabled')
        self.downstream_box.see(tk.END)

        # Schedule next update
        self.root.after(500, self.update_gui)

    # ---------------- ROS Spin ----------------
    def ros_spin(self):
        try:
            rclpy.spin(self)
        except Exception as e:
            self.get_logger().error(f"ROS spin error: {e}")

    def on_closing(self):
        # Cleanup and shutdown ROS when GUI closes
        self.get_logger().info("Shutting down dashboard node...")
        rclpy.shutdown()
        self.root.destroy()

def main():
    rclpy.init()
    FireUIDashboard()
    # rclpy.shutdown()  # shutdown handled in on_closing()

if __name__ == "__main__":
    main()
