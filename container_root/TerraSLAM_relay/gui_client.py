import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import numpy as np
from relay_client import ImageClient
import threading
import queue
import os
import glob
import time
import tkintermapview

class SLAMViewer(tk.Tk):
    def __init__(self, image_folder, server_ip='localhost', server_port=43322):
        super().__init__()

        self.title("SLAM-GPS Viewer")
        self.geometry("1400x700")

        # Create main container
        self.main_container = ttk.Frame(self)
        self.main_container.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Create top frame for coordinates
        self.coord_frame = ttk.Frame(self.main_container)
        self.coord_frame.pack(fill=tk.X, pady=(0, 10))
        self.coord_frame.grid_columnconfigure(0, weight=1)
        self.coord_frame.grid_columnconfigure(1, weight=1)
        self.coord_frame.grid_columnconfigure(2, weight=1)

        # Style configuration for coordinate blocks
        style = ttk.Style()
        style.configure('Title.TLabel', font=('Arial', 16, 'bold'), anchor='center', padding=(0, 5))
        style.configure('Value.TLabel', font=('Arial', 16), anchor='center', padding=(0, 5))
        style.configure('CoordinateFrame.TFrame', relief='solid', borderwidth=1)

        # Create coordinate blocks
        self.lat_frame = ttk.Frame(self.coord_frame, style='CoordinateFrame.TFrame')
        self.lon_frame = ttk.Frame(self.coord_frame, style='CoordinateFrame.TFrame')
        self.alt_frame = ttk.Frame(self.coord_frame, style='CoordinateFrame.TFrame')

        # Configure grid weights for coordinate frames
        for frame in [self.lat_frame, self.lon_frame, self.alt_frame]:
            frame.grid_columnconfigure(0, weight=1)
            frame.grid_rowconfigure(0, weight=1)
            frame.grid_rowconfigure(1, weight=2)

        self.lat_frame.grid(row=0, column=0, sticky='nsew', padx=5)
        self.lon_frame.grid(row=0, column=1, sticky='nsew', padx=5)
        self.alt_frame.grid(row=0, column=2, sticky='nsew', padx=5)

        # Create title labels (static)
        ttk.Label(self.lat_frame, text="Latitude", style='Title.TLabel').grid(row=0, column=0, sticky='nsew')
        ttk.Label(self.lon_frame, text="Longitude", style='Title.TLabel').grid(row=0, column=0, sticky='nsew')
        ttk.Label(self.alt_frame, text="Altitude", style='Title.TLabel').grid(row=0, column=0, sticky='nsew')

        # Create variables for coordinate values
        self.lat_var = tk.StringVar(value="XXXXXXXXXX")
        self.lon_var = tk.StringVar(value="XXXXXXXXXX")
        self.alt_var = tk.StringVar(value="XXXXXXXXXX")

        # Create value labels
        ttk.Label(self.lat_frame, textvariable=self.lat_var, style='Value.TLabel').grid(row=1, column=0, sticky='nsew')
        ttk.Label(self.lon_frame, textvariable=self.lon_var, style='Value.TLabel').grid(row=1, column=0, sticky='nsew')
        ttk.Label(self.alt_frame, textvariable=self.alt_var, style='Value.TLabel').grid(row=1, column=0, sticky='nsew')

        # Create bottom frame for image and map
        self.bottom_frame = ttk.Frame(self.main_container)
        self.bottom_frame.pack(fill=tk.BOTH, expand=True)
        self.bottom_frame.grid_columnconfigure(0, weight=2)  # Image frame gets less space
        self.bottom_frame.grid_columnconfigure(1, weight=3)  # Map frame gets more space

        # Create image frame
        self.image_frame = ttk.Frame(self.bottom_frame)
        self.image_frame.grid(row=0, column=0, sticky='nsew', padx=(0, 5))

        # Create image label
        self.image_label = ttk.Label(self.image_frame)
        self.image_label.pack(fill=tk.BOTH, expand=True)

        # Create map frame
        self.map_frame = ttk.Frame(self.bottom_frame)
        self.map_frame.grid(row=0, column=1, sticky='nsew', padx=(5, 0))

        # Create map widget with tall and narrow aspect ratio
        self.map_widget = tkintermapview.TkinterMapView(self.map_frame, width=400, height=550)
        self.map_widget.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Store current marker and map initialization state
        self.current_marker = None
        self.map_initialized = False
        self.last_marker_position = None  # Store last marker position to reduce updates
        self.update_threshold = 0.00001  # Minimum distance change to update marker (in degrees)
        self.last_marker_update_time = 0  # Last time marker was updated
        self.marker_update_interval = 0.2  # Minimum interval between marker updates (seconds)
        
        # Store trajectory points
        self.trajectory_points = []
        self.path = None

        # Initialize client - Server now handles GPS conversion and sends GPS coordinates directly
        self.client = ImageClient(server_ip=server_ip, server_port=server_port)
        if not self.client.connect():
            self.destroy()
            return

        # Initialize image processing
        self.image_folder = image_folder
        self.image_files = sorted(glob.glob(os.path.join(image_folder, '*.jpg')))
        self.current_image_index = 0
        
        # Create a queue for thread communication
        self.queue = queue.Queue()
        
        # Start processing thread
        self.processing_thread = threading.Thread(target=self.process_images, daemon=True)
        self.processing_thread.start()
        
        # Start queue checking
        self.check_queue()

    def update_coordinates(self, lat, lon, alt):
        self.lat_var.set(f"{lat}")
        self.lon_var.set(f"{lon}")
        self.alt_var.set(f"{alt}m")
        
        # Store trajectory point
        self.trajectory_points.append((lat, lon))
        
        # Check if we need to update marker position (reduce flickering)
        should_update_marker = True
        current_time = time.time()
        
        # Time-based throttling
        if current_time - self.last_marker_update_time < self.marker_update_interval:
            should_update_marker = False
        
        # Position-based throttling
        if should_update_marker and self.last_marker_position is not None:
            last_lat, last_lon = self.last_marker_position
            # Calculate distance change
            lat_diff = abs(lat - last_lat)
            lon_diff = abs(lon - last_lon)
            # Only update if change is significant enough
            if lat_diff < self.update_threshold and lon_diff < self.update_threshold:
                should_update_marker = False
        
        # Update map marker only when necessary
        if should_update_marker:
            if self.current_marker:
                self.current_marker.delete()
            self.current_marker = self.map_widget.set_marker(lat, lon)
            self.last_marker_position = (lat, lon)
            self.last_marker_update_time = current_time
        
        # Update trajectory path
        if self.path:
            self.path.delete()
        if len(self.trajectory_points) >= 2:
            self.path = self.map_widget.set_path(self.trajectory_points, 
                                               color="blue",  # 轨迹颜色
                                               width=2,       # 轨迹宽度
                                               )
        
        # Only set initial position and zoom once
        if not self.map_initialized:
            self.map_widget.set_position(lat, lon)
            self.map_widget.set_zoom(18)
            self.map_initialized = True
            

    def show_initializing(self):
        self.lat_var.set("Initializing")
        self.lon_var.set("Initializing")
        self.alt_var.set("Initializing")
        
        # Clear marker if exists
        if self.current_marker:
            self.current_marker.delete()
            self.current_marker = None
            self.last_marker_position = None  # Reset position tracking

    def show_tracking_lost(self):
        self.lat_var.set("Tracking Lost")
        self.lon_var.set("Tracking Lost")
        self.alt_var.set("Tracking Lost")
        
        # Keep marker but don't add to trajectory
        
    def show_not_initialized(self):
        self.lat_var.set("Not Initialized") 
        self.lon_var.set("Not Initialized")
        self.alt_var.set("Not Initialized")
        
        # Clear marker if exists
        if self.current_marker:
            self.current_marker.delete()
            self.current_marker = None
            self.last_marker_position = None  # Reset position tracking

    def update_image(self, image_path):
        # Read and resize image to fit the window
        image = cv2.imread(image_path)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Calculate new size while maintaining aspect ratio
        window_width = self.image_frame.winfo_width()
        window_height = self.image_frame.winfo_height()
        
        if window_width > 0 and window_height > 0:
            img_height, img_width = image.shape[:2]
            aspect_ratio = img_width / img_height
            
            if window_width / window_height > aspect_ratio:
                new_height = window_height
                new_width = int(window_height * aspect_ratio)
            else:
                new_width = window_width
                new_height = int(window_width / aspect_ratio)
                
            image = cv2.resize(image, (new_width, new_height))
            
            # Convert to PhotoImage
            image = Image.fromarray(image)
            photo = ImageTk.PhotoImage(image)
            
            # Update label
            self.image_label.configure(image=photo)
            self.image_label.image = photo

    def process_images(self):
        for image_path in self.image_files:
            try:
                # Send image and get pose
                if not self.client.send_image(image_path):
                    break
                
                # Get the latest pose data
                pose_data = self.client.latest_pose
                
                # Put the results in queue
                self.queue.put((image_path, pose_data))
                
                time.sleep(0.1)  # Small delay between images
                
            except Exception as e:
                print(f"Error processing image {image_path}: {e}")
                break

    def check_queue(self):
        try:
            while True:
                image_path, pose_data = self.queue.get_nowait()
                
                # Update image
                self.update_image(image_path)
                
                # Update coordinates - now pose_data contains GPS coordinates directly from server
                if pose_data:
                    lat, lon, alt = pose_data
                    
                    # Check for special status values as defined in relay.py
                    if lat == -3.0 and lon == -3.0 and alt == -3.0:
                        # Tracking lost status
                        self.show_tracking_lost()
                    elif lat == -1.0 and lon == -1.0 and alt == -1.0:
                        # Not initialized status  
                        self.show_not_initialized()
                    elif lat == 0.0 and lon == 0.0 and alt == 0.0:
                        # No images yet / initializing status
                        self.show_initializing()
                    else:
                        # Valid GPS coordinates - display directly
                        self.update_coordinates(lat, lon, alt)
                
                self.queue.task_done()
                
        except queue.Empty:
            pass
        
        # Schedule the next queue check
        self.after(10, self.check_queue)

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='SLAM-GPS Viewer')
    parser.add_argument('image_folder', help='Path to the folder containing images')
    parser.add_argument('--server', '-s', default='127.0.0.1', help='Server IP address (default: localhost)')
    parser.add_argument('--port', '-p', type=int, default=43322, help='Server port (default: 43322)')
    parser.add_argument('--google-api-key', default=None, help='Google Maps API key (default: None)')
    parser.add_argument('--map-type', default='hybrid', choices=['roadmap', 'satellite', 'hybrid'],
                      help='Map type when using Google Maps (default: hybrid)')
    
    args = parser.parse_args()
    
    # Check if folder exists
    if not os.path.exists(args.image_folder):
        print(f"Folder not found: {args.image_folder}")
        return
        
    app = SLAMViewer(args.image_folder, args.server, args.port)
    
    # Set up Google Maps with the API key
    if args.google_api_key:
        try:
            # Map type parameters:
            # m = roadmap (default)
            # s = satellite only
            # y = hybrid (satellite + labels)
            map_type_param = {
                'roadmap': 'm',
                'satellite': 's',
                'hybrid': 'y'
            }[args.map_type]
            
            app.map_widget.set_tile_server(
                f"https://mt0.google.com/vt/lyrs={map_type_param}&hl=en&x={{x}}&y={{y}}&z={{z}}&s=Ga",
                max_zoom=22
            )
            print(f"Using Google Maps ({args.map_type} view)")
        except Exception as e:
            print(f"Failed to initialize Google Maps: {e}")
            print("Falling back to OpenStreetMap")
    else:
        print("Using OpenStreetMap (provide --google-api-key to use Google Maps)")
    
    app.mainloop()

if __name__ == '__main__':
    main() 