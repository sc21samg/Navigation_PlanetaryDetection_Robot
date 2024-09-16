import sys
import yaml
from dataclasses import dataclass

@dataclass
class Coordinate:
    x: float
    y: float

@dataclass
class ModuleCoordinates:
    name: str
    entrance: Coordinate
    center: Coordinate

@dataclass
class Modules:
    module_1: ModuleCoordinates
    module_2: ModuleCoordinates


def get_module_coordinates(coordinates_file_path):
    try:
        with open(coordinates_file_path) as file:
            worlds_dict = yaml.safe_load(file)

            module1_entrance_x = worlds_dict['module1']['entrance_x']
            module1_entrance_y = worlds_dict['module1']['entrance_y']
            module1_center_x = worlds_dict['module1']['center_x']
            module1_center_y = worlds_dict['module1']['center_y']
            module1 = ModuleCoordinates(
                'module1', 
                Coordinate(module1_entrance_x, module1_entrance_y),
                Coordinate(module1_center_x, module1_center_y)
            )

            module2_entrance_x = worlds_dict['module2']['entrance_x']
            module2_entrance_y = worlds_dict['module2']['entrance_y']
            module2_center_x = worlds_dict['module2']['center_x']
            module2_center_y = worlds_dict['module2']['center_y']
            module2 = ModuleCoordinates(
                'module2', 
                Coordinate(module2_entrance_x, module2_entrance_y),
                Coordinate(module2_center_x, module2_center_y)
            )

            return Modules(module1, module2)

    except FileNotFoundError:
        sys.exit("The specified YAML file was not found.")
    except yaml.YAMLError as exc:
        sys.exit("An error occurred while parsing the YAML file:", exc)

