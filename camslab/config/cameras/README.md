# Camera calibrations

One file per camera model and image size, named `<model>_<width>x<height>.yaml`, written by

    ros2 run camslab calibrate --name <model>_<width>x<height>

with the lab's tag grid board (6 x 6 tags of 8.8 cm, 2.64 cm apart). Set `camera: <name>` in
`config/survey.yaml` to use one, and commit new files so that every car with the same
camera uses them. See `docs/hardware-survey.md`.
