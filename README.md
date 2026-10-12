# Line-Following Maze Car

Arduino Mega line follower with 5 IR sensors, one dual motor driver (4 motors, 2 per side).
Sketch: `maze_solver/maze_solver.ino`.

## Turn logic
When a far sensor sees black or the line disappears (a 90° turn / junction):
1. Creep forward so the wheels are over the corner, then stop.
2. Rotate anti-clockwise (up to ~90°) until the center sensor finds a line.
3. If none, rotate clockwise ~180° to check the other side.
4. If still none (dead end), keep rotating clockwise to go back.

## Tuning on the car
1. `LINE_DETECTED` – set to `LOW` if your sensors output LOW on black.
2. `INVERT_LEFT_MOTOR` / `INVERT_RIGHT_MOTOR` – flip a side that runs backwards.
3. `TURN_90_MS` – set `CALIBRATE_TURN = true`, upload, and adjust until each spin is 90°. Then set it back to `false`.
4. `CREEP_MS` – increase if the car starts turning before the wheels reach the corner.
5. `BASE_SPEED`, `KP`, `KD` – line-following smoothness.

Open the Serial Monitor at 9600 baud to see junction decisions.
