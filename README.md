Navigation_PlanetaryDetection_Robot 
=====================

## Overview
PlanetaryDetection_NavigationRobot is a robotic system that autonomously navigates a spacecraft and detects planets like Earth, Moon, Mars, and Mercury through windows. Using image stitching, machine learning, and LaserScan data, it calculates distances between planets to determine the spacecraft's location. The system is tested in both simulations and real-world environments.

## Team Members
This project was developed collaboratively by Ayesha Rahman, Sandra Guran, Natalie Leung and Geeyoon Lim. Together, we implemented various components of the 3D rendering application as part of our coursework. In the attached report, a table outlining the distribution of individual and collaborative tasks is presented.

## Aim
Develop a robotic system capable of:
- Navigating the spacecraft's environment using input coordinates.
- Detecting celestial bodies such as Earth, Moon, Mars, and Mercury from windows.
- Stitching together panoramic images to calculate distances between planets and determine the spacecraft's position.
  
## Tasks Breakdown

1. Navigation & Obstacle Avoidance
Input Coordinates: The robot navigates specific points within the spacecraft using input from a YAML file. It adapts dynamically to different environments.
Wall Navigation: Uses LaserScan data and PID controllers for real-time wall-following and obstacle avoidance.
Grid Navigation & Heuristics: The robot predicts window locations based on room layout heuristics for efficient exploration.
2. Sign Detection
Red and Green Sign Detection: Detects signs at entrances, using image processing to decide whether the robot should enter the room or bypass it.
3. Window Detection & Image Stitching
Window Detection: Detects windows by identifying rectangular shapes with specific characteristics like a white border and black interiors.
Screenshot Capture: Aligns the robot with the window, captures images, and stitches them to form a panorama of the environment.
4. Planet Detection & Measurement
Planet Detection: Detects Earth, Moon, Mars, and Mercury within the captured panorama using a machine learning model.
Distance Calculation: Measures distances between detected planets using pixel data and known planet diameters to determine the spacecraft's location.
5. Machine Learning Model for Planet Detection
Model: A custom ResNet18-based CNN trained on planetary images. It detects celestial bodies in the panorama images.
Training & Accuracy: Trained on a dataset from GitHub and the internet, the model achieves 92.59% accuracy in planet detection.
6. Simulation & Real-World Testing
Simulation Worlds: The system was tested in simulated environments with varying complexity (Easy, Moderate, Hard, and Custom worlds).
Real Robot Testing: The final version was tested on a real robot for navigation, window detection, and planet detection tasks.

## Project Structure
src/: Source code for robot control, image processing, and navigation.
models/: Pre-trained ResNet18 model used for planet detection.
worlds/: Simulation worlds used for testing the system.
scripts/: Helper scripts for window detection, image stitching, and planet distance calculation.
requirements.txt: Required Python libraries for running the project.

## Results & Limitations
The system successfully navigates the spacecraft, detects windows, and generates a panorama for planet detection.
Planet detection and distance measurement have been validated, but limitations exist in real-world data, especially with noisy LaserScan data and low-resolution images.
A potential future enhancement includes implementing LaserScan Stretching to handle gaps in LaserScan data during real-world testing.
