"""Settings files of the launch files."""

import os
import tempfile

import yaml

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def arguments(default_file, defaults=None):
    """The `config` launch argument and one override argument per top-level value of the
    file and of `defaults`, the values a file may leave out."""
    with open(default_file) as f:
        settings = {**(defaults or {}), **yaml.safe_load(f)}
    return [DeclareLaunchArgument("config", default_value=default_file,
                                  description="settings file")] + [
        DeclareLaunchArgument(key, default_value="",
                              description=f"overrides `{key}` in the settings file")
        for key, value in settings.items() if not isinstance(value, (dict, list))]


def load(context, defaults=None):
    """`defaults`, then the settings file named by `config`, then the overrides from the
    command line."""
    with open(LaunchConfiguration("config").perform(context)) as f:
        settings = {**(defaults or {}), **yaml.safe_load(f)}
    for key, value in settings.items():
        override = context.launch_configurations.get(key, "")
        if override != "" and not isinstance(value, (dict, list)):
            settings[key] = yaml.safe_load(override)
    return settings


def controller_file(settings):
    """Path of a controller parameter file with the settings' `controller` values, loaded
    after params.yaml."""
    out = tempfile.NamedTemporaryFile("w", prefix="camslab_controller_", suffix=".yaml",
                                      delete=False)
    with out:
        yaml.safe_dump({"/**/controller": {"ros__parameters": settings.get("controller") or {}}},
                       out)
    return out.name


def default_polytope_file(launch_file):
    """config/polytope.yaml in the source tree, where the web UI saves the safety polytope.
    Under colcon --symlink-install `launch_file` links into the source tree, so the file
    is found without a rebuild; otherwise it is the installed config directory."""
    launch_dir = os.path.dirname(os.path.realpath(launch_file))
    return os.path.normpath(os.path.join(launch_dir, "..", "config", "polytope.yaml"))
