import torch
from torchvision import models, transforms
from PIL import Image
import numpy as np
import cv2
from scipy import ndimage
import os
from math import sqrt

import os
import glob

def is_folder_empty(folder_path):
    """Check if the specified folder is empty.
    
    Args:
    folder_path (str): The path to the folder to check.
    
    Returns:
    bool: True if the folder is empty, False otherwise.
    """
    # Check if the folder exists and is a directory
    if not os.path.isdir(folder_path):
        raise NotADirectoryError(f"The specified path is not a directory: {folder_path}")
    
    # Check if the directory is empty
    return len(os.listdir(folder_path)) == 0

def delete_planet_images(folder_path):
    """
    Deletes all files in the specified folder that start with 'planet'.
    
    Args:
    folder_path (str): The path to the folder from which files should be deleted.
    """
    # Construct the search pattern to match all files starting with 'planet'
    search_pattern = os.path.join(folder_path, 'planet*')
    
    # Use glob to find all files matching the pattern
    files_to_delete = glob.glob(search_pattern)
    
    # Loop through the list of files and remove each one
    for file_path in files_to_delete:
        try:
            os.remove(file_path)
            print(f"Deleted {file_path}")
        except Exception as e:
            print(f"Failed to delete {file_path}: {e}")



# def find_and_load_image(folder_path, image_name):
#     """Find and load an image with either JPG or PNG extension."""
#     for ext in ['jpg', 'png']:
#         image_path = os.path.join(folder_path, f"{image_name}.{ext}")
#         if os.path.exists(image_path):
#             return cv2.imread(image_path)
#     return None

def find_and_load_image(folder_path, image_name):
    """Find and load an image with the PNG extension."""
    image_path = os.path.join(folder_path, f"view{image_name}.png")
    if os.path.exists(image_path):
        return cv2.imread(image_path)
    return None

def load_celestial_images(folder_path, celestial_bodies):
    """Loads images of celestial bodies and counts how many were successfully loaded."""
    loaded_images = {}
    loaded_count = 0

    for body in celestial_bodies:
        image = find_and_load_image(folder_path, body)
        if image is not None:
            loaded_images[body] = image
            loaded_count += 1
        else:
            print(f"Failed to load image for {body}")

    print(f"Total images loaded: {loaded_count}")
    return loaded_count

def locate_and_crop(image_path, threshold=100):
    ''' Locate the planet in the image and return a cropped image centered on the planet. '''
    img = Image.open(image_path)
    img_array = np.array(img)
    gray = cv2.cvtColor(img_array, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY)
    
    # Find contours and the largest blob
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    
    # Crop the image around the detected planet
    crop_img = img.crop((x, y, x + w, y + h))
    return crop_img


# #loading the model weights to detect the planets
# def load_model(path_to_weights, num_classes=4):

#     try:
#         # Load model logic
#         print(f"Model loaded successfully.")
#     except Exception as e:
#         print(f"Failed to load model", exc_info=True)
        
def load_model(path_to_weights, num_classes=4):
    model = models.resnet18(pretrained=False)
    num_ftrs = model.fc.in_features
    model.fc = torch.nn.Linear(num_ftrs, num_classes)
    model.load_state_dict(torch.load(path_to_weights, map_location=torch.device('cpu')))
    model.eval()
    return model

    
        
    # Initialize the model (assuming using ResNet18 here)
    #model = models.resnet18(pretrained=False)
    from torchvision.models import ResNet18_Weights
    model = models.resnet18(weights=None)
    
    
def detect_and_crop_planets(image_path, model, device, save_directory):
    # Load the image
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        print("Error: Failed to load the image.")
        return None, []

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply GaussianBlur to reduce noise and improve circle detection
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # Apply adaptive threshold to create a binary image
    _, binary = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
    
    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50, param1=100, param2=30, minRadius=20, maxRadius=100)
    
    planet_details = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for i, (x, y, r) in enumerate(circles):
            # Further filtering based on circle characteristics
            if r > 20 and r < 100 and x > r and y > r and x + r < img.shape[1] and y + r < img.shape[0]:
                height_pixels = 2 * r  # Calculate the height in pixels
                # Crop the image around the detected planet
                left = max(x - r, 0)
                top = max(y - r, 0)
                right = min(x + r, img.shape[1])
                bottom = min(y + r, img.shape[0])
                cropped_image = img[top:bottom, left:right]
                pil_image = Image.fromarray(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
                cropped_image_path = os.path.join(save_directory, f"planet_candidate_{i+1}.png")
                pil_image.save(cropped_image_path)

                # Use the model to predict and filter out non-planet objects
                class_index = predict_image(cropped_image_path, model, device)
                os.remove(cropped_image_path)

                # If the detected object is a planet, keep it
                if class_index in [0, 1, 2, 3]:  # Assuming 0, 1, 2, 3 correspond to planets
                    planet_details.append({'x': x, 'y': y, 'r': r, 'height': height_pixels})
                    pil_image.save(os.path.join(save_directory, f"planet_{len(planet_details)}.png"))

    return circles, planet_details
    # Modify the final layer to match the number of classes (assuming 4 planet types)
    num_ftrs = model.fc.in_features
    model.fc = torch.nn.Linear(num_ftrs, num_classes)
    # Load the model weights
    model.load_state_dict(torch.load(path_to_weights, map_location=torch.device('cpu')))
    model.eval()  # Set the model to evaluation mode
    return model

def predict_image(image_path, model, device):
    # Image transformations
    transform = transforms.Compose([
        transforms.Resize((224, 224)),  # Resize the image to fit the model input
        transforms.ToTensor(),  # Convert the image to a tensor
        transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])  # Normalize to match the model training
    ])
    
    # Load image
    image = Image.open(image_path)
    image = transform(image).unsqueeze(0)  # Add batch# Saving in the 'window' folder dimension
    image = image.to(device)  # Move image to the device (GPU/CPU)
    
    # Predict
    with torch.no_grad():
        outputs = model(image)
        _, predicted = torch.max(outputs, 1)
        class_index = predicted.item()
    
    return class_index

def clear_directory(directory):
    """Remove all files in the specified directory."""
    for item in os.listdir(directory):
        file_path = os.path.join(directory, item)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print(f"Failed to delete {file_path}. Reason: {e}")


#detection of image, for the model
def detect_and_crop_planets2(image_path):
    # Load the image
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        print("Error: Failed to load the image.")
        return None, []

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply GaussianBlur to reduce noise and improve circle detection
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # Detect circles
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, 1.2, 100, param1=50, param2=30, minRadius=30, maxRadius=100)
    
    planet_details = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for (x, y, r) in circles:
            height_pixels = 2 * r  # Calculate the height in pixels
            planet_details.append({'x': x, 'y': y, 'r': r, 'height': height_pixels})
            # Crop and save each planet image
            left = max(x - r, 0)
            top = max(y - r, 0)
            right = min(x + r, img.shape[1])
            bottom = min(y + r, img.shape[0])
            cropped_image = img[top:bottom, left:right]
            pil_image = Image.fromarray(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
            filename = (f"planet_{len(planet_details)}.png")  
            pil_image.save(filename)

    return circles, planet_details



def detect_and_crop_planets(image_path, model, device, save_directory):
    # Load the image
    img = cv2.imread(image_path, cv2.IMREAD_COLOR)
    if img is None:
        print("Error: Failed to load the image.")
        return None, []

    # Convert to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Apply GaussianBlur to reduce noise and improve circle detection
    blurred = cv2.GaussianBlur(gray, (9, 9), 2)
    
    # Apply adaptive threshold to create a binary image
    _, binary = cv2.threshold(blurred, 50, 255, cv2.THRESH_BINARY)
    
    # Detect circles using HoughCircles
    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50, param1=100, param2=30, minRadius=20, maxRadius=100)
    
    planet_details = []
    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for i, (x, y, r) in enumerate(circles):
            # Further filtering based on circle characteristics
            if r > 20 and r < 100 and x > r and y > r and x + r < img.shape[1] and y + r < img.shape[0]:
                height_pixels = 2 * r  # Calculate the height in pixels
                # Crop the image around the detected planet
                left = max(x - r, 0)
                top = max(y - r, 0)
                right = min(x + r, img.shape[1])
                bottom = min(y + r, img.shape[0])
                cropped_image = img[top:bottom, left:right]
                pil_image = Image.fromarray(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB))
                cropped_image_path = os.path.join(save_directory, f"planet_candidate_{i+1}.png")
                pil_image.save(cropped_image_path)

                # Use the model to predict and filter out non-planet objects
                class_index = predict_image(cropped_image_path, model, device)
                os.remove(cropped_image_path)

                # If the detected object is a planet, keep it
                if class_index in [0, 1, 2, 3]:  # Assuming 0, 1, 2, 3 correspond to planets
                    planet_details.append({'x': x, 'y': y, 'r': r, 'height': height_pixels})
                    pil_image.save(os.path.join(save_directory, f"planet_{len(planet_details)}.png"))

    return circles, planet_details


#calculate distance to a planet
def calculate_distance_to_planet(focal_length, real_diameter_km, image_height_pixels, planet_height_pixels, sensor_height):
    if planet_height_pixels == 0:
        return float('inf')  # Avoid division by zero
    distance_km = (focal_length * real_diameter_km * image_height_pixels) / (planet_height_pixels * sensor_height)
    return distance_km

#calculate distance between 2 planets
def calculate_pixel_distance(x1, y1, x2, y2):
    return sqrt((x2 - x1)**2 + (y2 - y1)**2)



