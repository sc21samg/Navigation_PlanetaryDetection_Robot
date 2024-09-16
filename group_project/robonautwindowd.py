import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import time
import signal
from group_project import coordinates
from geometry_msgs.msg import Twist
import math
import os
from nav_msgs.msg import Odometry
import shutil


import os
import cv2
import torch
import sys
import numpy as np

# Assuming your module is in the directory "src" relative to the script
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'src')))



from .detect_image_rename import is_folder_empty, delete_planet_images, clear_directory, locate_and_crop, load_model, predict_image, detect_and_crop_planets, detect_and_crop_planets2, calculate_distance_to_planet, calculate_pixel_distance, load_celestial_images
from .good_stitching import crop_white_borders, process_folder, find_and_load_image, resize_image_to_match, crop_image, stitch_images, find_target_rectangle

class RoboNaut(Node):
    def __init__(self):
        super().__init__('robotnaut')
        
        # Initialize the CvBridge and the subscription to the camera images
          # Initialize the CvBridge and the subscription to the camera images
        self.bridge = CvBridge()
        self.create_subscription(Image, '/camera/image_raw', self.camera_callback, 10)
        
        # Parameters and state variables
        self.sign_detected = False
        self.entrance_attempts = 0
        self.coordinates_file_path = self.declare_parameter('coordinates_file_path', '').get_parameter_value().string_value
        self.coordinates = coordinates.get_module_coordinates(self.coordinates_file_path)
        
        # Action client for navigation
        self.navigation_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rate = self.create_rate(10)  # 10 Hz
        self.angle_increment = 15 * (math.pi /180)

        self.navigation_goal_handle = None
        self.latest_image_data = None
        self.current_entrance = 1
        self.sign_detected_post_rotation = False
        
        #for window implementation
        self.in_green_room = False
        self.window_screenshot_counter = 0    
        self.robot_stoped = False   
        self.window_seen = False
        self.flag = False
        self.robot_close = False
        self.screenshot_taken = False
        self.contour = 0
        self.flag_idk = False
        self.latest_cv_image = None
        
        self.create_subscription(Odometry, '/odom', self.odometry_callback, 10)
        self.last_movement_time = time.time()  # Timestamp of the last detected movement
        # Define speed and angular speed thresholds
        self.speed_threshold = 0.01  # Adjust this value as needed
        self.angular_speed_threshold = 0.01  # Adjust this value as needed

        self.total_rotation_radians = 0
        self.full_rotation = False
        self.windows_ss_dir = '../../ros2_ws/src/group-project-group-13/group13'
        self.flag_empty_dir = False
        
        self.flag_window_processing0 = 0
        self.flag_window_processing1 = 0
        self.flag_window_processing2 = 0




    def navigate_to_point(self, x, y):
        self.get_logger().info(f'Navigating to point: {x}, {y}')
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.orientation.w = 1.0
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.navigation_client.wait_for_server()
        self.navigation_goal_handle = self.navigation_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.navigation_goal_handle.add_done_callback(self.goal_response_callback)
        
    def camera_callback(self, data):
        self.latest_image_data = data  # Update with the latest received image data
        
        if self.sign_detected:
            return
            
        cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        self.latest_cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        cv2.namedWindow('camera_Feed',cv2.WINDOW_NORMAL)
        cv2.imshow('camera_Feed', cv_image)
        cv2.resizeWindow('camera_Feed',320,240)
        cv2.waitKey(3)
        red_mask = self.color_filter(cv_image, 'red')
        green_mask = self.color_filter(cv_image, 'green')
    

        
        red_detected = self.detect_sign(red_mask)
        green_detected = self.detect_sign(green_mask)
        
        if red_detected or green_detected:
            self.sign_detected = True
            self.handle_sign_detection('green' if green_detected else 'red')
        
            
        if self.in_green_room:
            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 120, 255, cv2.THRESH_BINARY)
            
            # Find contours
            contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours)>0: 
                c = max(contours, key=cv2.contourArea)
                #print(cv2.contourArea(c))      
                if cv2.contourArea(c) > 9000:
                    # Too close to object, need to move backwards
                    # Set a flag to tell the robot to move backwards when in the main loop
                    self.robot_close = True
                if cv2.contourArea(c) < 9000:
                    # Too close to object, need to move backwards
                    # Set a flag to tell the robot to move backwards when in the main loop
                    self.robot_close = False
                    
            for contour in contours:
                # Approximate contour to polygon
                epsilon = 0.01 * cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, epsilon, True)
                
                # If the contour has 4 points, it's a rectangle
                if len(approx) == 4:
                    x, y, w, h = cv2.boundingRect(approx)
                    
                    # Define a region of interest (ROI) that's slightly larger than the rectangle
                    # to check for the white border
                    border_thickness = 10
                    x_start, y_start = max(x - border_thickness, 0), max(y - border_thickness, 0)
                    x_end, y_end = min(x + w + border_thickness, cv_image.shape[1]), min(y + h + border_thickness, cv_image.shape[0])
                    
                    # The region of interest including potential white border
                    roi_with_border = gray[y_start:y_end, x_start:x_end]
                    
                    # Threshold to find white areas
                    _, white_thresh = cv2.threshold(roi_with_border, 220, 255, cv2.THRESH_BINARY)
                    
                    # Check for white pixels around the approximated polygon
                    has_white_border = np.any(white_thresh[:border_thickness, :] == 255) and \
                                       np.any(white_thresh[-border_thickness:, :] == 255) and \
                                       np.any(white_thresh[:, :border_thickness] == 255) and \
                                       np.any(white_thresh[:, -border_thickness:] == 255)
                                       
                    # Now, check if the inside of the rectangle is mostly black
                    roi = gray[y:y+h, x:x+w]
                    _, black_thresh = cv2.threshold(roi, 50, 255, cv2.THRESH_BINARY_INV)
                    black_area = np.sum(black_thresh == 255)
                    total_area = roi.size
                    
                    # Define the ratio of black to total area (set your own threshold)
                    black_ratio = black_area / total_area
                    
                    # If we have a white border and the black_ratio is above a threshold, consider it a window
                    if has_white_border and black_ratio > 0.5:
                        #self.get_logger().info(f"Found window at {x}, {y}, {w}x{h}")
                        # Construct the unique filename with an incrementing counter
                        filename = f'../../ros2_ws/src/group-project-group-13/group13/window{self.window_screenshot_counter}.png'
                        self.window_screenshot_counter += 1  # Increment the screenshot counter
                        
                        # Save the image with the detected window
                        #cv2.imwrite(filename, cv_image)
                        #self.get_logger().info(f"Screenshot saved as {filename}")

                        if x > 0:
                            self.window_seen = True
                            self.get_logger().info("Robot can see window")

                        else:
                            self.window_seen = False
                            self.get_logger().info("Robot can not see window")


                        # Navigate to the window or perform any other action as required
                        #self.navigate_to_window(x, y, w, h, cv_image)
                        # Note: You'll need to implement this navigate_to_window method

                        break  # Stop after finding the first valid window
                    
        if self.in_green_room and self.robot_stoped and not self.full_rotation:
            self.handle_robot_stop()
        elif self.full_rotation:
            self.get_logger().info("Robot finished taking screenshots of the windows.")
            rclpy.shutdown()
            
        if self.flag_window_processing0 == 2:
            self.detect_image_rename()
            
        if self.flag_window_processing1 == 2:
            self.good_stitching()
            
        if self.flag_window_processing2 == 2:
            self.planet_measurements()

                    
                    
    def handle_robot_stop(self):
        if self.in_green_room:
            if self.window_seen:
                self.get_logger().info("Window seen, attempting to capture...")
                self.attempt_capture()
            else:
                self.get_logger().info("No window seen, starting search rotation...")
                self.search_for_window()
                
                    
    def attempt_capture(self):
        if not self.screenshot_taken and self.contour != 0:
                self.search_for_window()            
        if self.robot_close:
            # Assuming robot_close is correctly updated elsewhere
            self.get_logger().info("Proper distance, taking screenshot.")
            self.stop()
            #ist taken picture doen't correctly check if there is a window or not so we ignore it
            # if self.contour == 0:
            #     self.prepare_capture()
            if self.contour >= 0:
                filename = f'../../ros2_ws/src/group-project-group-13/group13/window{self.contour}.png'
                cv2.imwrite(filename, self.latest_cv_image)  # Ensure latest_image_data is updated in camera_callback
                self.get_logger().info(f"Screenshot saved as {filename}.")
            self.contour += 1
            # Reset flags as necessary, prepare for next window or stop
            self.screenshot_taken = False
            self.window_seen = False
        else:
            self.get_logger().info("Too far from the window. Adjusting position...")
            # Adjust position as necessary. This might involve small backward movement or repositioning.
            #self.walk_forward_window(0.2, 2)
            
    def prepare_capture(self):
        """
        Prepares for a new sequence of window captures by clearing the windows_ss folder if necessary
        and initializing the contour counter.
        """
        if self.contour == 0 and not self.prepare_capture_called:
            # Check if the folder exists and contains files
            if os.path.exists(self.windows_ss_dir) and os.listdir(self.windows_ss_dir):
                # Clear the folder
                for filename in os.listdir(self.windows_ss_dir):
                    file_path = os.path.join(self.windows_ss_dir, filename)
                    try:
                        if os.path.isfile(file_path) or os.path.islink(file_path):
                            os.unlink(file_path)
                        elif os.path.isdir(file_path):
                            shutil.rmtree(file_path)
                    except Exception as e:
                        self.get_logger().info(f"Failed to delete {file_path}. Reason: {e}")
            self.get_logger().info("windows_ss folder cleared for new captures.")
            self.prepare_capture_called = True


        
    def walk_forward_window(self, target_speed, ramp_duration):
        """
        Makes the robot walk forward, gradually ramping up to the target speed.

        :param target_speed: The target speed in meters per second.
        :param ramp_duration: The duration over which to ramp up the speed in seconds.
        """
        start_speed = 0.0  # Starting speed
        ramp_step = 0.02  # Speed increment for each step. Adjust as needed.
        ramp_interval = 0.1  # Time between each speed increment in seconds.

        current_speed = start_speed
        steps = int(ramp_duration / ramp_interval)
        speed_increment = (target_speed - start_speed) / steps

        self.get_logger().info("Starting to walk forward.")
        for step in range(steps):
            current_speed += speed_increment
            desired_velocity = Twist()
            desired_velocity.linear.x = current_speed
            self.publisher.publish(desired_velocity)
            time.sleep(ramp_interval)  # Wait a bit before the next increment

        # Ensure the robot reaches the exact target speed at the end of the ramp-up
        desired_velocity.linear.x = target_speed
        self.publisher.publish(desired_velocity)
        self.get_logger().info(f"Reached target speed of {target_speed} m/s.")

    def search_for_window(self):
        # Rotate a bit and check for the window. Repeat as necessary.
        self.rotate_for_duration(-0.7, 1.2)  # Slower and shorter rotation for finer control
        # After rotation, you should allow some time for camera_callback to potentially update window_seen
        time.sleep(0.5)  # Wait a bit after rotation for the camera to catch up and detect any potential window


    def rotate_for_duration(self, rotation_speed, duration):
        """
        Rotates the robot for a given duration and at a given speed, 
        cumulatively tracking the total rotation and logging when a full 360-degree rotation is achieved.

        :param rotation_speed: Angular velocity in radians per second. Positive for clockwise rotation, negative for counter-clockwise.
        :param duration: Duration of the rotation in seconds.
        """
        self.get_logger().info('Starting rotation.')
        if self.contour == 0:
            self.total_rotation_radians = 0
        # Calculate the rotation for this call
        rotation_this_call = rotation_speed * duration

        # Update the total cumulative rotation
        self.total_rotation_radians += rotation_this_call

        # Check if the total rotation has reached or exceeded 360 degrees (2 * pi radians)
        if abs(self.total_rotation_radians) >= 2 * math.pi:
            self.get_logger().info("Cumulative rotation has reached or exceeded 360 degrees.")
            self.full_rotation = True
            # Reset the total rotation for subsequent rotations
            #self.total_rotation_radians = 0

        # Create a Twist message to send rotation commands
        rotation_velocity = Twist()
        rotation_velocity.angular.z = rotation_speed

        # Publish the rotation command and sleep for the specified duration
        self.publisher.publish(rotation_velocity)
        time.sleep(duration)

        # Stop the robot after rotating
        rotation_velocity.angular.z = 0.0
        self.publisher.publish(rotation_velocity)
        self.get_logger().info('Rotation completed.')
        


    
    def odometry_callback(self, msg):
        # Assuming 'msg' is of type nav_msgs/Odometry
        current_speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 + msg.twist.twist.linear.z**2)
        current_angular_speed = abs(msg.twist.twist.angular.z)

        # Update the last movement time if there's significant movement
        movement_threshold = 0.01  # Adjust based on your robot's specifics
        if current_speed > movement_threshold or current_angular_speed > movement_threshold:
            self.last_movement_time = time.time()


    def walk_forward(self):
        desired_velocity = Twist()
        desired_velocity.linear.x = 0.2  # Adjust speed as needed
        self.publisher.publish(desired_velocity)
            
    #check if the robot has stopped                   
    def odometry_callback(self, msg):
        # Assuming 'msg' is of type nav_msgs/Odometry
        current_speed = math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        current_angular_speed = abs(msg.twist.twist.angular.z)
        
            
        if current_speed <= 0.009  and current_angular_speed <= 0.09:
            # The robot has stopped moving
            #self.get_logger().info("CCCCCCCCCCC.") 
            self.robot_stoped = True
            #self.get_logger().info("CCCCCAAAAAAAACCCCCC.") 
        else:
            self.robot_stoped = False
    ###################################
    
    #####################################
    def detect_image_rename(self):
        #CHANGE TO 1
        if self.flag_window_processing0 == 1:
            #empty folders
            save_directory = '../../ros2_ws/src/group-project-group-13/group13'
            #clear_directory(save_directory)           
            
            folder_path = '../../ros2_ws/src'
            delete_planet_images(folder_path)

            folder_path = '../../ros2_ws/src/group-project-group-13/image_detection/extra_images'
            clear_directory(folder_path)  
            
            # self.get_logger().info("CCCCCAAAAAAAACCCCCC.")
            # print(f"work bitch.")
            model_path = '../../ros2_ws/src/group-project-group-13/data_training/model.pth'
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            model = load_model(model_path, num_classes=4)
            model.to(device)
            #add the actual ss windows folder location here
            image_directory = '../../ros2_ws/src/group-project-group-13/image_detection/test_windows'
            images = [os.path.join(image_directory, f) for f in os.listdir(image_directory) if f.endswith('.jpg')]

            for image_path in images:
                img = cv2.imread(image_path, cv2.IMREAD_COLOR)
                if img is None:
                    print(f"Failed to load image from {image_path}.")
                    continue

                image_height_pixels, image_width_pixels, _ = img.shape
                circles, planet_details = detect_and_crop_planets2(image_path)

                if not planet_details:
                    print("No planets detected in the image.")
                    continue

                classes = ['Earth', 'Mars', 'Mercury', 'Moon']
                planet_diameters = {'Earth': 12742, 'Mars': 6794, 'Mercury': 4878, 'Moon': 3475}
                sensor_height = 3
                focal_length = 3
                planet_positions = []
                planet_names = []

                for detail in planet_details:
                    filename = f"planet_{len(planet_positions)+1}.png"
                    
                    cropped_image = locate_and_crop(filename)
                    cropped_image_path = "cropped_planet.jpg"
                    cropped_image.save(cropped_image_path)  # Save the cropped image for prediction

                    class_index = predict_image(cropped_image_path, model, device)
                    os.remove(cropped_image_path)

                    planet_type = classes[class_index]
                    real_diameter_km = planet_diameters[planet_type]
                    distance_km = calculate_distance_to_planet(focal_length, real_diameter_km, image_height_pixels, detail['height'], sensor_height)

                    planet_positions.append((detail['x'], detail['y'], distance_km, planet_type, detail['height']))
                    planet_names.append(planet_type)

                # Create a filename from the detected planet names
                unique_filename = '_'.join(planet_names) + ".png"
                new_image_directory = '../../ros2_ws/src/group-project-group-13/group13'
                save_path = os.path.join(new_image_directory, unique_filename)

                # Save the processed image
                cv2.imwrite(save_path, img)
                print(f"Saved processed image with detected planets at: {save_path}")
                
                self.flag_window_processing0 = 0
                self.flag_window_processing1 = 1

        
    def good_stitching(self):
        #CHANGE TO 1
        if self.flag_window_processing1 == 1:
            # Directory containing the images
            folder_path = '../../ros2_ws/src/group-project-group-13/group13'
            process_folder(folder_path)
            
            # Define file paths and order of celestial bodies
            folder_path = '../../ros2_ws/src/group-project-group-13/group13'
            celestial_bodies = ['Earth', 'Moon', 'Mars', 'Mercury']
            images = {name: find_and_load_image(folder_path, name) for name in celestial_bodies}

            # Initialize the base image and stitch the images
            base_image = None
            for name, img in images.items():
                if img is not None:
                    if base_image is None:
                        base_image = img
                        print(f"Initialized panorama with {name}.")
                    else:
                        base_image = stitch_images(base_image, img)
                        print(f"Added {name} to panorama.")

            if base_image is not None:
                output_path = os.path.join(folder_path, 'panorama.png')
                cv2.imwrite(output_path, base_image)
                print(f"Final panorama saved as '{output_path}'.")
            else:
                print("No images were found for stitching.")

            # Load the image
            image_path = '../../ros2_ws/src/group-project-group-13/group13/panorama.png'
            #image_path = '../../ros2_ws/src/final_panorama (1).jpg'
            image = cv2.imread(image_path)

            # Check if the image was loaded correctly
            if image is None:
                print("Failed to load image.")
            else:
                print("Image loaded successfully.")
                
            self.flag_window_processing1 = 0
            self.flag_window_processing2 = 1
            self.count_images = 0
            
            
    def planet_measurements(self):
        #CHANGE TO 1
        if self.flag_window_processing2 == 1:
    
            folder_path = '../../ros2_ws/src/group-project-group-13/group13'
            celestial_bodies = ['Earth', 'Moon', 'Mars', 'Mercury']
            loaded_images_count = load_celestial_images(folder_path, celestial_bodies)

            
            #print(f"work bitch222.")
            model_path = '../../ros2_ws/src/group-project-group-13/data_training/model.pth'
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            model = load_model(model_path, num_classes=4)
            model.to(device)
            
            #print(f"work bitch22333333333333333332.")
            

            image_path = '../../ros2_ws/src/group-project-group-13/group13/panorama.png'
            #image_path = '../../ros2_ws/src/final_panorama (1).jpg'
            #"merc+moon" "earth+moon" "moon+merc" "mars+moon+earth" "moon+earth" "merc+moon+earth" 
            img = cv2.imread(image_path, cv2.IMREAD_COLOR)
            if img is None:
                print("Failed to load image.")
                return
            save_directory = '../../ros2_ws/src'
            image_height_pixels, image_width_pixels, _ = img.shape
            circles, planet_details = detect_and_crop_planets(image_path, model, device, save_directory)

            # Clear the file at the start of the script
            with open("../../ros2_ws/src/group-project-group-13/group13/measurements.txt", "w") as file:
                file.write("")

            if planet_details:
                classes = ['Earth', 'Mars', 'Mercury', 'Moon']
                planet_diameters = {'Earth': 12742, 'Mars': 6794, 'Mercury': 4878, 'Moon': 3475}
                sensor_height = 3
                focal_length = 3

                planet_positions = []

                for detail in planet_details:
                    filename = f"planet_{len(planet_positions)+1}.png"
                    cropped_image = locate_and_crop(filename)
                    cropped_image_path = "../../ros2_ws/src/group-project-group-13/image_detection/extra_images/cropped_planet.jpg"
                    cropped_image.save(cropped_image_path)  # Save the cropped image for prediction

                    class_index = predict_image(cropped_image_path, model, device)
                    os.remove(cropped_image_path)
                    
                    planet_type = classes[class_index]
                    real_diameter_km = planet_diameters[planet_type]
                    distance_km = calculate_distance_to_planet(focal_length, real_diameter_km, image_height_pixels, detail['height'], sensor_height)

                    planet_positions.append((detail['x'], detail['y'], distance_km, planet_type, detail['height']))

                # Sort planets by x-coordinate
                planet_positions.sort(key=lambda x: x[0])
                
                colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]  # Red, Green, Blue, Yellow
                for index, pos in enumerate(planet_positions):
                    color = colors[index % len(colors)]
                    cv2.circle(img, (pos[0], pos[1]), pos[4]//2, color, 2)
                    cv2.line(img, (pos[0], 0), (pos[0], image_height_pixels), color, 2)
                    cv2.line(img, (0, pos[1]), (image_width_pixels, pos[1]), color, 2)
                    with open("../../ros2_ws/src/group-project-group-13/group13/measurements.txt", "a") as file:
                        self.count_images = self.count_images + 1
                        file.write(f"{pos[3]}: {int(pos[2])} km\n")
                        print(f"{pos[3]}: {int(pos[2])} km") 
                        print(f"Detected {pos[3]} at (x={pos[0]}, y={pos[1]}) with height {pos[4]} pixels and distance {int(pos[2])} km. \n")
                        print(f"ZZZZZZZZZZZZZZZZZZZZZZZ: {int(self.count_images)} km")
                        print(f"loaded_images_count: {int(loaded_images_count)} km")
                        # if loaded_images_count == self.count_images:
                        #     self.flag_window_processing2 = 0

                for i in range(len(planet_positions)):
                    for j in range(i + 1, len(planet_positions)):
                        pixel_distance = calculate_pixel_distance(planet_positions[i][0], planet_positions[i][1],
                                                                planet_positions[j][0], planet_positions[j][1])
                        avg_real_distance_km = (planet_positions[i][2] + planet_positions[j][2]) / 2
                        real_distance_km = avg_real_distance_km * (pixel_distance / image_width_pixels)
                        with open("../../ros2_ws/src/group-project-group-13/group13/measurements.txt", "a") as file:
                            if len(planet_positions) == 2: 
                                file.write(f"Distance: {int(real_distance_km)} km")
                                print(f"Distance: {int(real_distance_km)} km")
                            else:
                                file.write(f"Distance between {planet_positions[i][3]} and {planet_positions[j][3]}: {int(real_distance_km)} km\n")
                                print(f"Distance between {planet_positions[i][3]} and {planet_positions[j][3]}: {int(real_distance_km)} km\n")
        
                
                cv2.imwrite('../../ros2_ws/src/group-project-group-13/image_detection/extra_images/detected_planets.png', img)
                #self.flag_window_processing2 = 0
                folder_path = '../../ros2_ws/src'
                #delete_planet_images(folder_path)
                
            if not is_folder_empty(folder_path):
                folder_path = '../../ros2_ws/src/group-project-group-13/image_detection/extra_images/'
                self.flag_window_processing2 = 0

        
    def stop(self):
        twist_msg = Twist()  # Stop any movement
        self.publisher.publish(twist_msg)   

            
    def color_filter(self, img, color):
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if color == 'red':
            # Red color range
            lower_bound1 = np.array([0, 120, 70])
            upper_bound1 = np.array([10, 255, 255])
            lower_bound2 = np.array([170, 120, 70])
            upper_bound2 = np.array([180, 255, 255])
            mask1 = cv2.inRange(hsv, lower_bound1, upper_bound1)
            mask2 = cv2.inRange(hsv, lower_bound2, upper_bound2)
            mask = mask1 + mask2
        elif color == 'green':
            # Green color range
            lower_bound = np.array([36, 25, 25])
            upper_bound = np.array([86, 255, 255])
            mask = cv2.inRange(hsv, lower_bound, upper_bound)
        else:
            mask = np.zeros(img.shape[:2], dtype="uint8")
        return mask
    
    def detect_sign(self, mask):
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            area = cv2.contourArea(cnt)
            perimeter = cv2.arcLength(cnt, True)
            if perimeter == 0:  # To avoid division by zero
                continue
            circularity = 4 * np.pi * area / (perimeter ** 2)
            
            # Adjust these thresholds as necessary
            if area > 500 and 0.7 <= circularity <= 1.2:  # This range might need tuning
                return True
        return False

    
    def rotate_robot(self):
        self.get_logger().info('Initiating rotationnnn.')
        rotation_velocity = Twist()
        rotation_velocity.angular.z = math.pi / 6  # Rotate with 30 degrees per second

        rotate_time = 6  # Adjust the duration of rotation as necessary
        end_time = time.time() + rotate_time

        while rclpy.ok() and time.time() < end_time:
            #self.get_logger().info('Publishing rotation command.')
            self.publisher.publish(rotation_velocity)
            time.sleep(0.1)  # Sleep to simulate a reasonable command rate

        # Stop the robot after rotating
        rotation_velocity.angular.z = 0.0
        self.publisher.publish(rotation_velocity)
        self.get_logger().info('Rotation completeddddddAAAAAAAAAAAAAAddddddddddddddd.')


                    

    def handle_sign_detection(self, sign_color):
        self.get_logger().info(f'{sign_color.capitalize()} sign detected.')

        if sign_color == 'red':
            # Rotate the robot to check for green signs
            self.rotate_robot()  # Rotate 360 degrees to scan for green signs
            # Set the flag to true to indicate the rotation was for a red sign detection
            self.sign_detected_post_rotation = True
            # After rotation, reprocess the latest image data to check for a green sign
            # This is done by invoking a separate method to handle post-rotation processing
            self.process_latest_image_for_green()
            self.current_entrance = 2

        elif sign_color == 'green':
            if self.sign_detected_post_rotation:
                # Handle green sign detected after rotation
                self.get_logger().info('Green sign detected post rotation. Proceeding to the center.')
                self.proceed_to_center(self.current_entrance)
                self.sign_detected_post_rotation = False  # Reset the flag
                self.current_entrance += 1
            else:
                # Handle green sign detection not related to post-rotation
                self.get_logger().info(f'Proceeding to the center of entrance {self.current_entrance}.')
                self.proceed_to_center(self.current_entrance)
                self.current_entrance += 1
                if self.current_entrance > 2:
                    self.in_green_room = True
                    self.get_logger().info('Reached end goal')

        self.sign_detected = False  # Reset the flag after handling



    def process_latest_image_for_green(self):
        # Simulate a brief wait for the rotation to settle and to capture a fresh image
        time.sleep(2)
        # Assuming the camera callback continuously updates self.latest_image_data
        green_detected = self.detect_green_circle(self.latest_image_data)

        if green_detected:
            self.get_logger().info('Green sign detected after rotation. Proceeding to the center.')
            self.proceed_to_center(self.current_entrance)
            self.current_entrance = 2
        else:
            self.get_logger().info('No green sign detected after rotation. Moving to the next entrance.')
            # Navigate to the next entrance
            self.current_entrance = 2 if self.current_entrance == 1 else 1
            self.get_logger().info(f'Navigating to entrance: {self.current_entrance}')
            self.navigate_to_entrance(self.current_entrance)



    def detect_green_circle(self, image_data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image_data, 'bgr8')
        except CvBridgeError as e:
            self.get_logger().error(f'Failed to convert image: {str(e)}')
            return False

        green_mask = self.color_filter(cv_image, 'green')
        return self.detect_sign(green_mask)

    def proceed_to_center(self, entrance_number):
        # Navigate to the center based on the entrance number
        if entrance_number == 1:
            self.navigate_to_point(self.coordinates.module_1.center.x, self.coordinates.module_1.center.y)
        elif entrance_number == 2:
            self.navigate_to_point(self.coordinates.module_2.center.x, self.coordinates.module_2.center.y)
        else:
            self.get_logger().info('Invalid entrance number for center navigation.')

    def navigate_to_entrance(self, entrance_number):
        # Navigate to the entrance based on the entrance number
        if entrance_number == 1:
            self.navigate_to_point(self.coordinates.module_1.entrance.x, self.coordinates.module_1.entrance.y)
        elif entrance_number == 2:
            self.navigate_to_point(self.coordinates.module_2.entrance.x, self.coordinates.module_2.entrance.y)
        else:
            self.get_logger().info('Invalid entrance number for entrance navigation.')

    def goal_response_callback(self, future):
            self.navigation_goal_handle = future.result()
            if not self.navigation_goal_handle.accepted:
                self.get_logger().info('Goal rejected')
                return
            self.get_logger().info('Goal accepted')
            self.flag_window_processing0 = 1
            self.navigation_goal_handle.get_result_async().add_done_callback(self.get_result_callback)

        
    def get_result_callback(self, future):
        result = future.result().result
        if result == 'nav2_msgs.action.NavigateToPose_Result(result=std_msgs.msg.Empty())':
            self.get_logger().info('zzzzzzzzzzzzzzzzzzzzzzzzzzzzzz')
            self.flag = True
        self.get_logger().info(f'Navigation result: {result}')
        # Decide next action based on navigation result and state
        
    def feedback_callback(self, feedback_msg):
        # This function can be used to provide feedback during navigation
        pass

def main(args=None):
    rclpy.init(args=args)
    robotnaut = RoboNaut()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    
    robotnaut.navigate_to_entrance(1)  # Start by navigating to the first entrance
    
    rclpy.spin(robotnaut)
    
    robotnaut.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()