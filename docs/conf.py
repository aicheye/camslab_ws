
project = "CAMS Lab Gazebo Simulator"
author = "CAMS Lab"
html_show_copyright = False

extensions = ["myst_parser"]
myst_enable_extensions = ["colon_fence", "dollarmath", "amsmath", "deflist"]
myst_heading_anchors = 3

source_suffix = {".md": "markdown"}
exclude_patterns = ["_build", "requirements.txt", "ode-hints.md", "estimation-hints.md",
                    "path-estimation-hints.md"]

html_theme = "sphinx_rtd_theme"
html_title = "CAMS Lab Gazebo Simulator"
html_theme_options = {
    "navigation_depth": 3,
    "collapse_navigation": False,
}
html_context = {
    "display_github": True,
    "github_user": "aicheye",
    "github_repo": "camslab_ws",
    "github_version": "main",
    "conf_py_path": "/docs/",
}
