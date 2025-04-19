import tkinter as tk
import cv2
from PIL import Image, ImageTk

class RoboticArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm Control GUI")
        self.root.geometry("1000x600")

        # Configure grid layout
        self.root.grid_columnconfigure(0, weight=3) # Camera display
        self.root.grid_columnconfigure(1, weight=1) # Controls

        # Camera display frame (left side)
        self.camera_frame = tk.Frame(root, bg="black", width=600, height=600)
        self.camera_frame.grid(row=0, column=0, sticky="nsew", padx=10, pady=10)

        # Camera label
        self.camera_label = tk.Label(self.camera_frame, bg="black")
        self.camera_label.pack(fill=tk.BOTH, expand=True)

        # Controls frame (right side)
        self.controls_frame = tk.Frame(root, bg="#f0f0f0", width=400, height=600)
        self.controls_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)

        # Status display
        self.status_label = tk.Label(self.controls_frame, text="Robotic Arm Status:", font=("Arial", 14), bg="#f0f0f0")
        self.status_label.pack(pady=(20, 10))

        self.servo_status = tk.Label(self.controls_frame, text="No servo activated", font=("Arial", 12), bg="white",
                                  width=25, height=2, relief=tk.SUNKEN)
        self.servo_status.pack(pady=(0, 20))

        # Servo control buttons
        self.create_button_frame("BLACK", ["B1", "B2", "B3"], "#333333", "white")
        self.create_button_frame("BLUE", ["B1", "B2", "B3"], "#3366cc", "white")

        # Dictionary to track button states (pressed or released)
        self.button_states = {}

        # Initialize camera
        self.init_camera()

    def create_button_frame(self, group_name, button_names, bg_color, fg_color):
        frame = tk.Frame(self.controls_frame, bg="#f0f0f0")
        frame.pack(pady=10, fill=tk.X)

        group_label = tk.Label(frame, text=f"{group_name} SERVOS", font=("Arial", 12, "bold"), bg="#f0f0f0")
        group_label.pack(pady=(0, 5))

        button_container = tk.Frame(frame, bg="#f0f0f0")
        button_container.pack()

        for i, name in enumerate(button_names):
            button_frame = tk.Frame(button_container, bg="#f0f0f0")
            button_frame.grid(row=0, column=i, padx=10)
            
            label = tk.Label(button_frame, text=name, font=("Arial", 10, "bold"), bg="#f0f0f0")
            label.pack(pady=(0, 5))
            
            # Left button (counterclockwise)
            left_btn = tk.Button(button_frame, text="←", bg=bg_color, fg=fg_color,
                               font=("Arial", 10, "bold"), width=3, height=2)
            left_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind mouse events to the left button
            button_id = f"{group_name} {name} CCW"
            left_btn.bind("<ButtonPress-1>", lambda event, id=button_id: self.button_press(id))
            left_btn.bind("<ButtonRelease-1>", lambda event, id=button_id: self.button_release(id))
            
            # Right button (clockwise)
            right_btn = tk.Button(button_frame, text="→", bg=bg_color, fg=fg_color,
                               font=("Arial", 10, "bold"), width=3, height=2)
            right_btn.pack(side=tk.LEFT, padx=2)
            
            # Bind mouse events to the right button
            button_id = f"{group_name} {name} CW"
            right_btn.bind("<ButtonPress-1>", lambda event, id=button_id: self.button_press(id))
            right_btn.bind("<ButtonRelease-1>", lambda event, id=button_id: self.button_release(id))

    def button_press(self, button_id):
        # Mark button as pressed
        self.button_states[button_id] = True
        
        # Update status display
        self.servo_status.config(text=f"Activated: {button_id}")
        
        # Print information
        print(f"{button_id}")
        
        # Start continuous status updates
        self.update_button_status(button_id)

    def update_button_status(self, button_id):
        # Only proceed if button is still pressed
        if button_id in self.button_states and self.button_states[button_id]:
            print(f"ACTIVE: {button_id}")
            
            # Schedule the next update only if button is still pressed
            self.root.after(100, lambda: self.update_button_status(button_id))

    def button_release(self, button_id):
        # Check if this button was active
        if button_id in self.button_states:
            # Mark button as released
            self.button_states[button_id] = False
            
            # Print release information
            print(f"RELEASED: {button_id}")
            
            # Remove from button states
            del self.button_states[button_id]
            
            # Update status to show no active servo
            if not self.button_states:
                self.servo_status.config(text="No servo activated")

    def init_camera(self):
        # Try to connect to camera (0 is usually the default webcam)
        self.cap = cv2.VideoCapture(0)
        
        if not self.cap.isOpened():
            # If no camera is available, show a placeholder
            placeholder = Image.new('RGB', (640, 480), color='darkgray')
            self.photo = ImageTk.PhotoImage(placeholder)
            self.camera_label.config(image=self.photo)
            self.camera_label.image = self.photo
            
            # Add text overlay
            text_label = tk.Label(self.camera_label, text="Camera Not Available",
                                font=("Arial", 18), bg="darkgray", fg="white")
            text_label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)
        else:
            # If camera is available, update frames
            self.update_camera()

    def update_camera(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert from BGR (OpenCV format) to RGB (PIL format)
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            
            # Convert to PIL Image and then to Tkinter PhotoImage
            image = Image.fromarray(frame)
            # Resize to fit our display if needed
            image = image.resize((600, 450), Image.LANCZOS)
            self.photo = ImageTk.PhotoImage(image)
            
            # Update the label
            self.camera_label.config(image=self.photo)
            self.camera_label.image = self.photo # Keep a reference
        
        # Update every 10ms (approximately 100 fps max)
        self.root.after(10, self.update_camera)

    def cleanup(self):
        # Clear any button states
        self.button_states.clear()
        
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmGUI(root)
    
    # Close camera properly when window is closed
    root.protocol("WM_DELETE_WINDOW", lambda: [app.cleanup(), root.destroy()])
    
    root.mainloop()

    