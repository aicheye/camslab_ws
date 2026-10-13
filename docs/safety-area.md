# Safe area

The safe area (safety polytope) is a polygon on the floor that the car must stay inside. While one is set, the controller refuses paths that come within the margin (0.2 m by default) of its edge, refuses **Start** while the car is outside it, and stops the car if it leaves it.

## Draw it

1. In the web UI's **Safety polytope** box, tick **edit**.
2. Add corners by clicking the map, or by placing the car on a corner and pressing **Add robot position**.
3. Set the **margin**, press **Apply and save**, and untick **edit**.

Place the corners where the rear axle may go; the car's body sticks out about 0.2 m in front of it. The area is saved to `camslab/config/polytope.yaml` and loaded on every start. To remove it, press **Clear**, then **Apply and save**.

```{warning}
Motion capture coordinates move with every calibration. Draw the safe area again after each one.
```
