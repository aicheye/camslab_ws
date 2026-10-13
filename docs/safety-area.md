# Safe area

Define the safe area (safety polytope) as a convex polygon. The controller then:

- rejects paths that come within the margin (0.2 m by default) of the edge,
- won't start if the car is outside the area,
- stops the car if it leaves the area.

## Setting the safe area

```{image} images/polytope.png
:alt: Drawing a safe area in the web UI, shown as a dashed red outline
```

1. In the **Safety polytope** box in the web UI, check **edit**.
2. Add corners by clicking on the map. Alternatively, place the car at a corner and press **Add robot position**.
3. Set the **margin**, press **Apply and save**, then uncheck **edit**.

Set corners for the rear-axle centre. The car front extends about 0.2 m past that point.
The simulator saves the safe area to `camslab/config/polytope.yaml` and loads it at
startup. Remove it by pressing **Clear**, then **Apply and save**.

```{warning}
Reset the safe area after every motion-capture calibration because calibration changes
the coordinates.
```
