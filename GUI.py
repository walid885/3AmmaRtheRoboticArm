import tkinter as tk
import cv2
from PIL import Image, ImageTk

class RoboticArmGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Robotic Arm Control GUI")
        self.root.geometry("1000x600")
        
        # Configure grid layout
        self.root.grid_columnconfigure(0, weight=3)  # Camera display
        self.root.grid_columnconfigure(1, weight=1)  # Controls
        
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
                                     width=20, height=2, relief=tk.SUNKEN)
        self.servo_status.pack(pady=(0, 20))
        
        # Servo control buttons
        self.create_button_frame("BLACK", ["B1", "B2", "B3"], "#333333", "white")
        self.create_button_frame("BLUE", ["B1", "B2", "B3"], "#3366cc", "white")
        
        # Keyboard binding
        self.root.bind("<Key>", self.key_press)
        
        # Keyboard instructions
        instruction_text = """
        Keyboard Controls:
        Q, W, E - BLACK 1, 2, 3
        A, S, D - BLUE 1, 2, 3
        """
        self.instructions = tk.Label(self.controls_frame, text=instruction_text, font=("Arial", 12), 
                                     bg="#f0f0f0", justify=tk.LEFT)
        self.instructions.pack(pady=20)
        
        # Initialize camera
        self.init_camera()

    def create_button_frame(self, group_name, button_names, bg_color, fg_color):
        # Create frame for button group
        frame = tk.Frame(self.controls_frame, bg="#f0f0f0")
        frame.pack(pady=10, fill=tk.X)
        
        # Group label
        group_label = tk.Label(frame, text=f"{group_name} SERVOS", font=("Arial", 12, "bold"), bg="#f0f0f0")
        group_label.pack(pady=(0, 5))
        
        # Button container
        button_container = tk.Frame(frame, bg="#f0f0f0")
        button_container.pack()
        
        # Create buttons
        for i, name in enumerate(button_names):
            button_id = f"{group_name}{name}"
            button = tk.Button(button_container, text=name, bg=bg_color, fg=fg_color,
                              font=("Arial", 10, "bold"), width=8, height=2,
                              command=lambda id=button_id: self.button_clicked(id))
            button.grid(row=0, column=i, padx=5)
    
    def button_clicked(self, button_id):
        self.servo_status.config(text=f"Activated: {button_id}")
        print(f"Button clicked: {button_id}")
    
    def key_press(self, event):
        key_map = {
            'q': 'BLACKB1',
            'w': 'BLACKB2',
            'e': 'BLACKB3',
            'a': 'BLUEB1',
            's': 'BLUEB2',
            'd': 'BLUEB3'
        }
        
        if event.char.lower() in key_map:
            self.button_clicked(key_map[event.char.lower()])
    
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
            self.camera_label.image = self.photo  # Keep a reference
        
        # Update every 10ms (approximately 100 fps max)
        self.root.after(10, self.update_camera)
    
    def cleanup(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

if __name__ == "__main__":
    root = tk.Tk()
    app = RoboticArmGUI(root)
    
    # Close camera properly when window is closed
    root.protocol("WM_DELETE_WINDOW", lambda: [app.cleanup(), root.destroy()])
    
    root.mainloop()
    