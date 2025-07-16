
import importlib_resources as pkg_resources
from pathlib import Path

def load_urdf_string():
    with pkg_resources.path("robot_kinemic.model.urdfs", "l_arm_v1.urdf") as urdf_path:
        print(f"urdf: {urdf_path}")
        with open(urdf_path, "r") as f:
            return f.read()