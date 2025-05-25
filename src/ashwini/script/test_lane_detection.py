#!/usr/bin/env python3

import cv2
import numpy as np
import pyrealsense2 as rs
import time

class LaneDetectionTest:
    def __init__(self):
        # Configure RealSense pipeline
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        # Configure streams
        self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        # Start streaming
        try:
            self.pipeline.start(self.config)
            print("RealSense camera initialized successfully!")
        except Exception as e:
            print(f"Failed to initialize RealSense camera: {e}")
            print("Make sure the camera is connected and not being used by another process")
            exit(1)
        
        # Lane detection parameters (default values)
        self.white_lower = np.array([0, 0, 200])
        self.white_upper = np.array([180, 30, 255])
        self.yellow_lower = np.array([15, 100, 100])
        self.yellow_upper = np.array([35, 255, 255])
        
        # Color picker variables
        self.white_hsv_values = []
        self.yellow_hsv_values = []
        self.picker_mode = "white"  # "white" or "yellow"
        self.is_picking_colors = False
        
        # Current image data
        self.current_image = None
        self.current_hsv = None
        
        # Zoom and pan variables
        self.zoom_factor = 1.0
        self.pan_x = 0
        self.pan_y = 0
        self.is_panning = False
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        self.min_zoom = 0.5
        self.max_zoom = 5.0
        self.mouse_x = 0
        self.mouse_y = 0
        
        print("Lane Detection Test with HSV Color Picker")
        print("=========================================")
        print("Choose your workflow:")
        print("1. Pick colors first (recommended)")
        print("2. Use default HSV values")
        
        # Display toggles
        self.show_white = True
        self.show_yellow = True
        self.show_edges = True
        self.show_roi = True
    
    def start_color_picker(self):
        """Start the color picking mode"""
        self.is_picking_colors = True
        print("\nCOLOR PICKER MODE")
        print("=================")
        print("Instructions:")
        print("1. Click on WHITE lane markings to collect white HSV values")
        print("2. Press 'y' to switch to YELLOW mode")
        print("3. Click on YELLOW lane markings to collect yellow HSV values")
        print("4. Press 'w' to switch back to WHITE mode")
        print("5. Press 'c' to calculate and apply HSV ranges")
        print("6. Press 'r' to reset collected values")
        print("7. Press 'd' to finish picking and start detection test")
        print("8. Press 'q' to quit")
        print("\nZoom Controls:")
        print("- Mouse wheel: Zoom in/out")
        print("- Right click + drag: Pan around")
        print("- Press 'z' to reset zoom and pan")
        print(f"\nCurrent mode: {self.picker_mode.upper()}")
        
        # Set up mouse callback for color picking
        cv2.namedWindow('Lane Detection Test')
        cv2.setMouseCallback('Lane Detection Test', self.color_picker_mouse_callback)
        
        return self.color_picker_loop()
    
    def color_picker_mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks for color picking with zoom and pan support"""
        if not self.is_picking_colors or self.current_hsv is None:
            return
        
        # Update mouse position for crosshair
        self.mouse_x = x
        self.mouse_y = y
            
        # Handle mouse wheel for zooming
        if event == cv2.EVENT_MOUSEWHEEL:
            # Get zoom center point
            center_x = x
            center_y = y
            
            # Calculate zoom
            if flags > 0:  # Scroll up - zoom in
                new_zoom = min(self.zoom_factor * 1.2, self.max_zoom)
            else:  # Scroll down - zoom out
                new_zoom = max(self.zoom_factor / 1.2, self.min_zoom)
            
            # Adjust pan to keep the mouse position centered
            zoom_ratio = new_zoom / self.zoom_factor
            self.pan_x = center_x - (center_x - self.pan_x) * zoom_ratio
            self.pan_y = center_y - (center_y - self.pan_y) * zoom_ratio
            
            self.zoom_factor = new_zoom
            print(f"Zoom: {self.zoom_factor:.2f}x")
            
        # Handle right click for panning
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.is_panning = True
            self.last_mouse_x = x
            self.last_mouse_y = y
            
        elif event == cv2.EVENT_RBUTTONUP:
            self.is_panning = False
            
        elif event == cv2.EVENT_MOUSEMOVE and self.is_panning:
            # Pan the image
            dx = x - self.last_mouse_x
            dy = y - self.last_mouse_y
            self.pan_x += dx
            self.pan_y += dy
            self.last_mouse_x = x
            self.last_mouse_y = y
            
        # Handle left click for color picking
        elif event == cv2.EVENT_LBUTTONDOWN:
            # Transform screen coordinates to original image coordinates
            orig_x, orig_y = self.screen_to_image_coords(x, y)
            
            # Check if coordinates are within image bounds
            if (0 <= orig_x < self.current_hsv.shape[1] and 
                0 <= orig_y < self.current_hsv.shape[0]):
                
                # Get HSV value at clicked point
                hsv_value = self.current_hsv[orig_y, orig_x]
                
                if self.picker_mode == "white":
                    self.white_hsv_values.append(hsv_value)
                    print(f"White HSV collected at ({orig_x},{orig_y}): {hsv_value} (Total: {len(self.white_hsv_values)})")
                    color = (255, 255, 255)
                else:
                    self.yellow_hsv_values.append(hsv_value)
                    print(f"Yellow HSV collected at ({orig_x},{orig_y}): {hsv_value} (Total: {len(self.yellow_hsv_values)})")
                    color = (0, 255, 255)
                
                # Draw a circle at clicked point on the original image
                cv2.circle(self.current_image, (orig_x, orig_y), 3, color, -1)
            else:
                print(f"Click outside image bounds: ({orig_x},{orig_y})")
    
    def screen_to_image_coords(self, screen_x, screen_y):
        """Convert screen coordinates to original image coordinates"""
        # Reverse the zoom and pan transformation
        orig_x = int((screen_x - self.pan_x) / self.zoom_factor)
        orig_y = int((screen_y - self.pan_y) / self.zoom_factor)
        return orig_x, orig_y
    
    def image_to_screen_coords(self, img_x, img_y):
        """Convert image coordinates to screen coordinates"""
        screen_x = int(img_x * self.zoom_factor + self.pan_x)
        screen_y = int(img_y * self.zoom_factor + self.pan_y)
        return screen_x, screen_y
    
    def apply_zoom_and_pan(self, image):
        """Apply zoom and pan transformation to image"""
        if self.zoom_factor == 1.0 and self.pan_x == 0 and self.pan_y == 0:
            return image
        
        # Create transformation matrix
        height, width = image.shape[:2]
        
        # Scale
        scaled_width = int(width * self.zoom_factor)
        scaled_height = int(height * self.zoom_factor)
        scaled_image = cv2.resize(image, (scaled_width, scaled_height), interpolation=cv2.INTER_NEAREST)
        
        # Create canvas
        canvas = np.zeros((height, width, 3), dtype=np.uint8)
        
        # Calculate paste position
        start_x = max(0, int(self.pan_x))
        start_y = max(0, int(self.pan_y))
        end_x = min(width, start_x + scaled_width)
        end_y = min(height, start_y + scaled_height)
        
        # Calculate source region
        src_start_x = max(0, -int(self.pan_x))
        src_start_y = max(0, -int(self.pan_y))
        src_end_x = src_start_x + (end_x - start_x)
        src_end_y = src_start_y + (end_y - start_y)
        
        # Paste scaled image onto canvas
        if src_end_x > src_start_x and src_end_y > src_start_y:
            canvas[start_y:end_y, start_x:end_x] = scaled_image[src_start_y:src_end_y, src_start_x:src_end_x]
        
        return canvas
    
    def calculate_hsv_ranges_from_samples(self):
        """Calculate HSV ranges from collected samples"""
        ranges_updated = False
        
        if len(self.white_hsv_values) > 0:
            white_array = np.array(self.white_hsv_values)
            white_min = np.min(white_array, axis=0)
            white_max = np.max(white_array, axis=0)
            
            # Add tolerance
            self.white_lower = np.maximum([0, 0, 0], white_min - [10, 30, 30])
            self.white_upper = np.minimum([179, 255, 255], white_max + [10, 30, 30])
            
            print(f"\nWHITE ranges updated:")
            print(f"Lower: {self.white_lower}")
            print(f"Upper: {self.white_upper}")
            ranges_updated = True
        
        if len(self.yellow_hsv_values) > 0:
            yellow_array = np.array(self.yellow_hsv_values)
            yellow_min = np.min(yellow_array, axis=0)
            yellow_max = np.max(yellow_array, axis=0)
            
            # Add tolerance
            self.yellow_lower = np.maximum([0, 0, 0], yellow_min - [5, 50, 50])
            self.yellow_upper = np.minimum([179, 255, 255], yellow_max + [5, 50, 50])
            
            print(f"\nYELLOW ranges updated:")
            print(f"Lower: {self.yellow_lower}")
            print(f"Upper: {self.yellow_upper}")
            ranges_updated = True
        
        if ranges_updated:
            print("\nHSV ranges have been updated based on your selections!")
        else:
            print("\nNo samples collected yet. Click on lane markings first.")
    
    def color_picker_loop(self):
        """Main loop for color picking"""
        try:
            while self.is_picking_colors:
                # Get frame
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    continue
                
                # Convert to numpy array
                color_image = np.asanyarray(color_frame.get_data())
                self.current_image = color_image.copy()
                self.current_hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
                
                # Create display image
                display_image = self.current_image.copy()
                
                # Apply zoom and pan transformation
                display_image = self.apply_zoom_and_pan(display_image)
                
                # Add overlay information
                mode_color = (255, 255, 255) if self.picker_mode == "white" else (0, 255, 255)
                cv2.putText(display_image, f"COLOR PICKER - Mode: {self.picker_mode.upper()}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, mode_color, 2)
                
                cv2.putText(display_image, f"White samples: {len(self.white_hsv_values)}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(display_image, f"Yellow samples: {len(self.yellow_hsv_values)}", 
                           (10, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                
                # Zoom info
                cv2.putText(display_image, f"Zoom: {self.zoom_factor:.2f}x", 
                           (10, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Current HSV ranges
                cv2.putText(display_image, f"White HSV: {self.white_lower} - {self.white_upper}", 
                           (10, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                cv2.putText(display_image, f"Yellow HSV: {self.yellow_lower} - {self.yellow_upper}", 
                           (10, 160), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
                
                # Instructions
                cv2.putText(display_image, "LClick=pick | RClick+drag=pan | Wheel=zoom | 'z'=reset | 'd'=detect", 
                           (10, display_image.shape[0] - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
                
                # Draw crosshair at mouse position
                if 0 <= self.mouse_x < display_image.shape[1] and 0 <= self.mouse_y < display_image.shape[0]:
                    # Draw crosshair
                    cv2.line(display_image, (self.mouse_x - 10, self.mouse_y), (self.mouse_x + 10, self.mouse_y), (0, 255, 0), 1)
                    cv2.line(display_image, (self.mouse_x, self.mouse_y - 10), (self.mouse_x, self.mouse_y + 10), (0, 255, 0), 1)
                    
                    # Get pixel value at mouse position
                    orig_x, orig_y = self.screen_to_image_coords(self.mouse_x, self.mouse_y)
                    if (0 <= orig_x < self.current_hsv.shape[1] and 0 <= orig_y < self.current_hsv.shape[0]):
                        pixel_hsv = self.current_hsv[orig_y, orig_x]
                        pixel_bgr = self.current_image[orig_y, orig_x]
                        
                        # Display pixel info
                        info_text = f"Pixel({orig_x},{orig_y}): HSV{pixel_hsv} BGR{pixel_bgr}"
                        cv2.putText(display_image, info_text, 
                                   (10, display_image.shape[0] - 45), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
                
                cv2.imshow('Lane Detection Test', display_image)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    return False  # Exit completely
                elif key == ord('w'):
                    self.picker_mode = "white"
                    print(f"Switched to WHITE mode")
                elif key == ord('y'):
                    self.picker_mode = "yellow"
                    print(f"Switched to YELLOW mode")
                elif key == ord('c'):
                    self.calculate_hsv_ranges_from_samples()
                elif key == ord('r'):
                    self.white_hsv_values = []
                    self.yellow_hsv_values = []
                    print("All HSV values reset!")
                elif key == ord('z'):
                    # Reset zoom and pan
                    self.zoom_factor = 1.0
                    self.pan_x = 0
                    self.pan_y = 0
                    print("Zoom and pan reset!")
                elif key == ord('d'):
                    # Calculate final ranges and start detection
                    self.calculate_hsv_ranges_from_samples()
                    self.is_picking_colors = False
                    print("\n" + "="*50)
                    print("SWITCHING TO DETECTION MODE...")
                    print("="*50)
                    return True  # Continue to detection
                    
        except KeyboardInterrupt:
            print("\nColor picking interrupted")
            return False
    
    def detect_lane_markings(self, color_image):
        """Detect lane markings using computer vision"""
        # Convert to HSV for better color filtering
        hsv = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
        
        # Create mask for white and yellow lane markings
        white_mask = cv2.inRange(hsv, self.white_lower, self.white_upper)
        yellow_mask = cv2.inRange(hsv, self.yellow_lower, self.yellow_upper)
        
        # Combine masks
        lane_mask = cv2.bitwise_or(white_mask, yellow_mask)
        
        # Apply Gaussian blur to reduce noise
        lane_mask = cv2.GaussianBlur(lane_mask, (5, 5), 0)
        
        # Apply morphological operations to clean up the mask
        kernel = np.ones((3, 3), np.uint8)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
        lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
        
        # Edge detection
        edges = cv2.Canny(lane_mask, 50, 150)
        
        # Define region of interest (lower half of image)
        height, width = edges.shape
        roi_vertices = np.array([[(0, height), (0, height//2), 
                                 (width, height//2), (width, height)]], dtype=np.int32)
        
        roi_mask = np.zeros_like(edges)
        cv2.fillPoly(roi_mask, roi_vertices, 255)
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        return white_mask, yellow_mask, lane_mask, edges, masked_edges, roi_vertices
    
    def create_debug_image(self, color_image, white_mask, yellow_mask, lane_mask, edges, masked_edges, roi_vertices):
        """Create debug visualization"""
        debug_image = color_image.copy()
        
        # Create overlay images
        overlay = debug_image.copy()
        
        # Show white mask in blue
        if self.show_white:
            white_colored = np.zeros_like(debug_image)
            white_colored[:, :, 0] = white_mask  # Blue channel
            overlay = cv2.addWeighted(overlay, 0.8, white_colored, 0.4, 0)
        
        # Show yellow mask in yellow
        if self.show_yellow:
            yellow_colored = np.zeros_like(debug_image)
            yellow_colored[:, :, 1] = yellow_mask  # Green channel
            yellow_colored[:, :, 2] = yellow_mask  # Red channel (Green + Red = Yellow)
            overlay = cv2.addWeighted(overlay, 0.8, yellow_colored, 0.4, 0)
        
        # Show edges in green
        if self.show_edges:
            edge_colored = np.zeros_like(debug_image)
            edge_colored[:, :, 1] = masked_edges  # Green channel
            overlay = cv2.addWeighted(overlay, 0.8, edge_colored, 0.6, 0)
        
        # Draw ROI
        if self.show_roi:
            cv2.polylines(overlay, roi_vertices, True, (255, 0, 0), 2)
        
        # Add text information
        y_offset = 30
        cv2.putText(overlay, f"DETECTION MODE", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        y_offset += 30
        cv2.putText(overlay, f"White pixels: {np.sum(white_mask > 0)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        cv2.putText(overlay, f"Yellow pixels: {np.sum(yellow_mask > 0)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        cv2.putText(overlay, f"Edge pixels: {np.sum(masked_edges > 0)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y_offset += 25
        cv2.putText(overlay, f"Total lane pixels: {np.sum(lane_mask > 0)}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        # Add parameter info
        y_offset += 40
        cv2.putText(overlay, f"White HSV: {self.white_lower} - {self.white_upper}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        y_offset += 20
        cv2.putText(overlay, f"Yellow HSV: {self.yellow_lower} - {self.yellow_upper}", 
                   (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        
        return overlay
    
    def run_detection_test(self):
        """Main detection test loop"""
        frame_count = 0
        
        print("\nDETECTION TEST MODE")
        print("==================")
        print("Controls:")
        print("  'q' - Quit")
        print("  's' - Save current frame")
        print("  'w' - Toggle white mask")
        print("  'y' - Toggle yellow mask")
        print("  'e' - Toggle edge detection")
        print("  'r' - Toggle ROI")
        print("  'p' - Go back to color picker")
        
        # Set up mouse callback for detection mode (no clicking)
        cv2.setMouseCallback('Lane Detection Test', lambda *args: None)
        
        try:
            while True:
                # Get frames
                frames = self.pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame:
                    continue
                
                # Convert to numpy arrays
                color_image = np.asanyarray(color_frame.get_data())
                
                # Process lane detection
                white_mask, yellow_mask, lane_mask, edges, masked_edges, roi_vertices = self.detect_lane_markings(color_image)
                
                # Create debug visualization
                debug_image = self.create_debug_image(color_image, white_mask, yellow_mask, 
                                                    lane_mask, edges, masked_edges, roi_vertices)
                
                # Create separate mask windows
                mask_display = np.hstack([
                    cv2.cvtColor(white_mask, cv2.COLOR_GRAY2BGR),
                    cv2.cvtColor(yellow_mask, cv2.COLOR_GRAY2BGR),
                    cv2.cvtColor(lane_mask, cv2.COLOR_GRAY2BGR),
                    cv2.cvtColor(masked_edges, cv2.COLOR_GRAY2BGR)
                ])
                
                # Add labels to mask display
                cv2.putText(mask_display, "White", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(mask_display, "Yellow", (170, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(mask_display, "Combined", (330, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(mask_display, "Edges", (490, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                
                # Display images
                cv2.imshow('Lane Detection Test', debug_image)
                cv2.imshow('Masks', mask_display)
                
                # Handle key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    timestamp = int(time.time())
                    cv2.imwrite(f'lane_test_{timestamp}.jpg', debug_image)
                    cv2.imwrite(f'masks_{timestamp}.jpg', mask_display)
                    print(f"Saved frames with timestamp {timestamp}")
                elif key == ord('w'):
                    self.show_white = not self.show_white
                    print(f"White mask: {'ON' if self.show_white else 'OFF'}")
                elif key == ord('y'):
                    self.show_yellow = not self.show_yellow
                    print(f"Yellow mask: {'ON' if self.show_yellow else 'OFF'}")
                elif key == ord('e'):
                    self.show_edges = not self.show_edges
                    print(f"Edge detection: {'ON' if self.show_edges else 'OFF'}")
                elif key == ord('r'):
                    self.show_roi = not self.show_roi
                    print(f"ROI: {'ON' if self.show_roi else 'OFF'}")
                elif key == ord('p'):
                    # Go back to color picker
                    cv2.destroyWindow('Masks')
                    self.start_color_picker()
                    if not self.is_picking_colors:  # If user chose to continue detection
                        continue
                    else:
                        break  # If user quit from color picker
                
                frame_count += 1
                
        except KeyboardInterrupt:
            print("\nTest interrupted by user")
        
        finally:
            print(f"Processed {frame_count} frames")
    
    def run(self):
        """Main entry point"""
        try:
            # Ask user what they want to do
            choice = input("Choose option (1=Pick colors first, 2=Use defaults): ").strip()
            
            if choice == "1":
                # Start with color picker
                print("Starting color picker...")
                picker_result = self.start_color_picker()
                print(f"Color picker returned: {picker_result}")
                if picker_result:
                    # User finished picking colors, start detection
                    print("Starting detection test...")
                    self.run_detection_test()
                else:
                    print("Color picker was cancelled or quit.")
            else:
                # Use default values and start detection
                print("Using default HSV values...")
                print(f"White HSV: {self.white_lower} - {self.white_upper}")
                print(f"Yellow HSV: {self.yellow_lower} - {self.yellow_upper}")
                self.run_detection_test()
                
        finally:
            # Clean up
            self.pipeline.stop()
            cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        tester = LaneDetectionTest()
        tester.run()
        
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure you have:")
        print("1. RealSense D455 connected")
        print("2. pyrealsense2 installed: pip install pyrealsense2")
        print("3. OpenCV installed: pip install opencv-python") 