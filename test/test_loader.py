# tests/test_loader.py
from robot_kinemic.loader import load_urdf_string

def test_urdf_load():
    urdf = load_urdf_string()
    print(f"urdf: {urdf}")
    assert "<robot" in urdf  # 简单验证URDF格式



if __name__ == "__main__":
    test_urdf_load()