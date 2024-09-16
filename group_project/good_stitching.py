import cv2
import numpy as np
import os


def crop_white_borders(image_path, output_path, black_threshold=30):
    # Read the image
    image = cv2.imread(image_path)
    if image is None:
        raise FileNotFoundError(f"The specified image file does not exist or could not be loaded: {image_path}")
    
    # Convert to grayscale
    grayscale = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    # Invert the grayscale image
    inverted = cv2.bitwise_not(grayscale)
    
    # Get the bounding box of the non-black regions
    contours, _ = cv2.findContours(inverted, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
    else:
        x, y, w, h = 0, 0, grayscale.shape[1], grayscale.shape[0]
    
    # Refine the bounding box to ensure majority black edges
    left, upper, right, lower = x, y, x + w, y + h
    while left < right and np.mean(grayscale[:, left]) > black_threshold:
        left += 1
    while right > left and np.mean(grayscale[:, right-1]) > black_threshold:
        right -= 1
    while upper < lower and np.mean(grayscale[upper, :]) > black_threshold:
        upper += 1
    while lower > upper and np.mean(grayscale[lower-1, :]) > black_threshold:
        lower -= 1
    
    # Ensure we do not crop the entire image to an empty one
    if left >= right or upper >= lower:
        left, upper, right, lower = 0, 0, grayscale.shape[1], grayscale.shape[0]
    
    # Crop the original image using the refined bounding box
    cropped_image = image[upper:lower, left:right]
    
    # Save the cropped image
    cv2.imwrite(output_path, cropped_image)
    
    return output_path


# def crop_white_borders(image_path, output_path):
#     image = cv2.imread(image_path)
#     if image is None:
#         raise FileNotFoundError(f"The specified image file does not exist or could not be loaded: {image_path}")

#     gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     _, thresholded = cv2.threshold(gray_image, 250, 255, cv2.THRESH_BINARY_INV)

#     kernel = np.ones((5, 5), np.uint8)
#     cleaned = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel)
#     cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)

#     contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
crop_white_borders
#     if contours:
#         largest_contour = max(contours, key=cv2.contourArea)
#         x, y, w, h = cv2.boundingRect(largest_contour)
#         cropped_image = image[y:y+h, x:x+w]
#     else:
#         cropped_image = image  # Return original if no contours found

#     cv2.imwrite(output_path, cropped_image)

def process_folder(folder_path):
    """Processes all images in the given folder, crops white borders, and replaces original files with cropped versions."""
    file_paths = [os.path.join(folder_path, file) for file in os.listdir(folder_path) if file.lower().endswith(('.png', '.jpg', '.jpeg'))]
    for file_path in file_paths:
        try:
            crop_white_borders(file_path, file_path)  # Overwrite the original image
            print(f"Cropped and replaced: {file_path}")
        except FileNotFoundError as e:
            print(e)

# # Directory containing the images
# folder_path = '../../ros2_ws/src/group-project-group-13/windows'
# process_folder(folder_path)

#################

import cv2
import numpy as np
import os

def find_and_load_image(folder_path, image_name):
    """Find and load an image with either JPG or PNG extension."""
    for ext in ['jpg', 'png']:
        image_path = os.path.join(folder_path, f"{image_name}.{ext}")
        if os.path.exists(image_path):
            return cv2.imread(image_path)
    return None

def resize_image_to_match(image, reference_height):
    """Resize the image to maintain aspect ratio based on the reference height."""
    (h, w) = image.shape[:2]
    scaling_factor = reference_height / float(h)
    return cv2.resize(image, (int(w * scaling_factor), reference_height), interpolation=cv2.INTER_AREA)

def crop_image(image):
    """Crop white borders from the image based on contour detection."""
    gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresholded = cv2.threshold(gray_image, 250, 255, cv2.THRESH_BINARY_INV)
    kernel = np.ones((5,5), np.uint8)
    cleaned = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, kernel)
    cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        cropped_image = image[y:y+h, x:x+w]
    else:
        cropped_image = image
    return cropped_image

def stitch_images(base_image, add_image):
    """Stitch two images horizontally."""
    if base_image is None or add_image is None:
        print("Error: One of the images is not available for stitching.")
        return None

    height = min(base_image.shape[0], add_image.shape[0])
    new_width = base_image.shape[1] + add_image.shape[1]
    stitched_image = np.zeros((height, new_width, 3), dtype=np.uint8)
    stitched_image[:height, :base_image.shape[1]] = base_image[:height, :]
    stitched_image[:height, base_image.shape[1]:] = add_image[:height, :]
    return stitched_image



 # Function to filter and find the desired rectangle
def find_target_rectangle(contours):
    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the shape is a rectangle
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            # Further check for the ratio and size if needed, here we assume the correct one is unique enough
            return x, y, w, h
    return None
# # Define file paths and order of celestial bodies
# folder_path = '../../ros2_ws/src/group-project-group-13/windows'
# celestial_bodies = ['earth', 'moon', 'mars', 'mercury']
# images = {name: find_and_load_image(folder_path, name) for name in celestial_bodies}

# # Initialize the base image and stitch the images
# base_image = None
# for name, img in images.items():
#     if img is not None:
#         if base_image is None:
#             base_image = img
#             print(f"Initialized panorama with {name}.")
#         else:
#             base_image = stitch_images(base_image, img)
#             print(f"Added {name} to panorama.")

# if base_image is not None:
#     output_path = os.path.join(folder_path, 'final_panorama.jpg')
#     cv2.imwrite(output_path, base_image)
#     print(f"Final panorama saved as '{output_path}'.")
# else:
#     print("No images were found for stitching.")
# #######################

# import cv2
# import numpy as np

# # Load the image
# image_path = '../../ros2_ws/src/group-project-group-13/windows/final_panorama.jpg'
# image = cv2.imread(image_path)

# # Check if the image was loaded correctly
# if image is None:
#     print("Failed to load image.")
# else:
#     print("Image loaded successfully.")

#     # Convert to grayscale
#     gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

#     # Apply threshold to find white regions
#     _, thresholded = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)

#     # Show the thresholded image for debugging
#     cv2.imshow("Thresholded Image", thresholded)
#     cv2.waitKey(0)
#     cv2.destroyAllWindows()

#     # Find contours in the thresholded image
#     contours, _ = cv2.findContours(thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#     print(f"Found {len(contours)} contours.")

#     # Function to filter and find the desired rectangle
#     def find_target_rectangle(contours):
#         for contour in contours:
#             # Approximate the contour to a polygon
#             epsilon = 0.02 * cv2.arcLength(contour, True)
#             approx = cv2.approxPolyDP(contour, epsilon, True)

#             # Check if the shape is a rectangle
#             if len(approx) == 4:
#                 x, y, w, h = cv2.boundingRect(approx)
#                 # Further check for the ratio and size if needed, here we assume the correct one is unique enough
#                 return x, y, w, h
#         return None

#     # Find the target rectangle
#     target = find_target_rectangle(contours)
#     print(f"Target rectangle: {target}")


#     # Crop and save the image if rectangle found
#     if target:
#         x, y, w, h = target
#         cropped_image = image[y:y+h, x:x+w]
#         cropped_path = '../../ros2_ws/src/group-project-group-13/windows/cropped_window.jpg'
#         cv2.imwrite(cropped_path, cropped_image)
#         print(f"Cropped image saved as {cropped_path}")
#     else:
#         print("No target rectangle found with the specified characteristics.")
# #############################

# import cv2
# import numpy as np

# # Load the image with the small resolution
# image_path = '../../ros2_ws/src/group-project-group-13/windows/cropped_window.jpg'
# image = cv2.imread(image_path)

# # Check if the image is loaded correctly
# if image is None:
#     print("Failed to load image.")
# else:
#     print("Image loaded successfully.")

#     # Resize the image to a higher resolution while trying to maintain quality
#     # Let's increase the size by 4 times both dimensions
#     new_width = image.shape[1] * 4
#     new_height = image.shape[0] * 4

#     # Use interpolation method to preserve details as much as possible
#     resized_image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_CUBIC)

#     # Save the resized image
#     resized_image_path = '../../ros2_ws/src/group-project-group-13/windows/resized_cropped_window.jpg'
#     cv2.imwrite(resized_image_path, resized_image)

#     resized_image_path
