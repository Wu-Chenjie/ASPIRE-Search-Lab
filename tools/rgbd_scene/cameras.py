"""Compatibility imports; implementation in aspire_camera.gazebo."""
from pathlib import Path
import sys

# Historical standalone tools import this module with only the checkout on sys.path.
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from aspire_camera.gazebo import (
    set_text,
    configure_camera,
    transforms,
    tf_arguments,
)
