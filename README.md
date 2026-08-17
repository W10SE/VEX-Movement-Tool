# VEX Autonomous Path Planner

A desktop GUI tool for designing VEX V5 autonomous movement paths and generating starter code/assets for several VEX programming targets.

## What this does

The application provides a visual field editor where you place robot waypoints and turn them into an autonomous path.

The main workflow is:

1. Draw a path on the field.
2. Move, insert, delete, undo, or clear points.
3. Adjust path simplification and smoothing.
4. Choose the drivetrain and robot settings.
5. Choose an export target.
6. Generate code and either copy it or save it to a file.
7. Optionally save the path itself as JSON so it can be reopened later.

The application keeps three versions of the path internally:

- **Raw waypoints** — the points you place on the field.
- **Simplified path** — raw points reduced with the Ramer-Douglas-Peucker (RDP) algorithm.
- **Smoothed path** — the simplified path interpolated with a Catmull-Rom curve.

The field editor uses meters internally. Exported coordinates can be generated in inches, millimeters, centimeters, or meters.

## Requirements

- Python 3
- `customtkinter` (installed from `requirements.txt`)
- A desktop environment capable of running Tkinter

Install the Python dependency with:

```bash
python -m pip install -r requirements.txt
```

## Start the application

From the project directory:

```bash
python main.py
```

The application opens as **VEX Autonomous Path Planner**.

## How to operate the field editor

The field is the large grid on the left side of the window.

### Mouse controls

- **Left-click on empty space:** add a waypoint.
- **Left-click and drag an existing waypoint:** move that waypoint.
- **Left-click near an existing path segment:** insert a waypoint into that segment.
- **Right-click an existing waypoint:** delete it.

The first waypoint is shown in green and the final waypoint is shown in red.

The axes are:

- `+X` to the right
- `-X` to the left
- `+Y` upward
- `-Y` downward

The field shown by the editor is **3.6 m × 3.6 m**, with 0.6 m grid tiles.

## Path controls

The right-hand **Path Controls** panel contains several groups.

### Path

**Undo Point**

Removes the most recently stored waypoint.

**Clear Path**

Deletes the entire current path.

**Save Path (.json)**

Saves the raw waypoint list to a JSON file.

**Load Path (.json)**

Loads a previously saved JSON path.

A saved path contains the waypoint coordinates; the simplified and smoothed versions are recalculated when the path is loaded.

### Simplify / Smooth

**RDP tolerance (m)**

Controls how aggressively the raw path is simplified.

- Lower value = keeps more corners/detail.
- Higher value = removes more points/corners.

**Curve smoothness (samples/segment)**

Controls how many interpolated samples are generated per curve segment.

- Higher value = more generated samples and a denser smooth path.
- Lower value = fewer generated samples.

**Pure-pursuit point spacing (m)**

Controls the spacing used when generating a LemLib pure-pursuit path file.

This setting affects the exported pure-pursuit path asset rather than the raw waypoint list.

## Robot settings

Before generating code, enter settings that match your robot.

### Common settings

**Track width (in)**

Your drivetrain track width in inches. The setting is stored and included in generated LemLib output comments; verify it against the drivetrain configuration used by your actual robot.

**Wheel diameter (in)**

Wheel diameter in inches. This is especially important for generated movement calculations.

**Start heading (deg)**

Initial robot heading.

The planner uses:

- `0°` = `+Y` / north
- Positive rotation = clockwise

**Max speed (0-127)**

Maximum speed used by generated movement commands where supported.

**Move timeout (ms)**

Timeout value used by generated LemLib movement commands.

### Drivetrain type

The planner supports:

- **Tank / Differential - No Sensor**
- **Tank / Differential - SmartDrive (Inertial/Gyro/GPS)**
- **Holonomic / Mecanum / X-Drive (4-motor)**

For SmartDrive output, the generated code uses heading/sensor-based commands and expects the corresponding sensor setup in the user's VEX project.

For holonomic output, the planner asks for four motor names and an external gear ratio.

### Holonomic motor settings

When holonomic mode is selected, enter:

- Front-left motor name
- Front-right motor name
- Back-left motor name
- Back-right motor name
- External gear ratio

The holonomic generator calculates per-wheel rotation amounts for each path leg.

## Code generation targets

The **Generate Code** section supports these export targets:

### LemLib (PROS C++) - moveToPoint/Pose

Generates a C++ `autonomous()` function using LemLib-style `setPose`, `moveToPoint`, and final `moveToPose` commands.

Default drivetrain object name:

```text
chassis
```

### LemLib Path File (.txt, pure pursuit)

Generates a text path asset for a pure-pursuit style workflow.

The smoothed path is resampled by the selected point spacing and each point is assigned a trapezoidal speed profile.

The generated file ends with:

```text
endData
```

The generated header suggests storing it as a path asset such as:

```text
static/paths/myPath.txt
```

### VEXcode V5 C++ (turn + drive)

Generates C++ movement commands for tank/differential or holonomic configurations.

Tank output uses VEXcode-style turn/drive commands. SmartDrive mode uses heading commands when selected.

### VEXcode V5 Python (turn + drive)

Generates the Python equivalent of the VEXcode movement output.

The generated code is intended as a starting point and should be checked against the exact devices, names, and API configuration in the target VEXcode project.

## Generated code window

After selecting **Generate Code**, the generated output appears in a separate window.

You can:

- **Copy to Clipboard** — copy all generated code.
- **Save As...** — save it with the appropriate `.cpp`, `.py`, or `.txt` extension.

The planner does not upload code to a VEX Brain or robot.

## Typical workflow

A practical workflow is:

1. Start the app with `python main.py`.
2. Pick the robot drivetrain type.
3. Set wheel diameter, track width, starting heading, speed, and timeout.
4. Draw the autonomous route with left-clicks.
5. Drag points to refine the route.
6. Increase RDP tolerance if the path has too many unnecessary corners.
7. Adjust smoothing until the curve looks appropriate.
8. Select the target VEX/LemLib format.
9. Set the drivetrain object/motor names to match the actual project.
10. Click **Generate Code**.
11. Copy or save the result.
12. Review the generated code before putting it into the robot project.
13. Test the autonomous routine carefully on the actual field and tune robot-specific values.

## Project structure

```text
vexUItool/
├── main.py                 # Application entry point
├── config.py               # Field, scale, units, and drivetrain constants
├── requirements.txt        # Python dependencies
├── core/
│   ├── geometry.py         # Geometry/interpolation helpers
│   ├── algorithms.py       # Simplification, smoothing, resampling, speed profile
│   └── state_manager.py   # Path state and JSON save/load
├── codegen/
│   └── codegen.py          # Code/path-file generators
├── ui/
│   ├── app.py              # Main GUI window and application wiring
│   └── widgets/
│       ├── path_canvas.py  # Interactive field editor
│       ├── sidebar.py      # Controls and settings
│       ├── inputs.py       # Reusable input widgets
│       └── code_window.py  # Generated-code window
└── utils/
    └── converters.py       # Coordinate/unit/heading conversion helpers
```

## Path file format

Saved `.json` files contain the raw waypoint coordinates.

Example shape:

```json
{
  "waypoints_m": [
    [0.0, 0.0],
    [0.5, 0.8],
    [1.0, 1.5]
  ]
}
```

Coordinates are stored in meters.

## Important limitations

This tool generates code/templates; it does **not** know the exact hardware configuration of your robot.

Before running generated code:

- Verify motor and drivetrain object names.
- Verify wheel size, gearing, and other mechanical values.
- Verify the selected VEX/LemLib API matches the version installed in your project.
- Check the robot's actual starting position and heading.
- Test movements at safe speeds before a full autonomous run.
- Review generated code rather than treating it as guaranteed production-ready code.

The current LemLib generator also notes that LemLib output is intended for tank-style drivetrains. Selecting holonomic mode while using the LemLib target results in a warning and tank-style LemLib code.

## Troubleshooting

### The app does not start

Make sure you are running it from the project root and that the dependency is installed:

```bash
python -m pip install -r requirements.txt
python main.py
```

If your system has multiple Python installations, use the same Python executable for both commands.

### Generated code is empty

Add at least **two waypoints** before pressing **Generate Code**.

### My path has too many points

Increase the **RDP tolerance**.

### My path is too coarse

Decrease the RDP tolerance and/or increase the curve smoothness.

### A loaded JSON path looks different from before

The JSON stores the raw waypoints only. The simplified and smoothed paths are recalculated from those points using the current smoothing settings.

## Development notes

The application is a local Python desktop program. There is no server or network service required by the code in this project.

The implementation includes algorithms for:

- RDP path simplification
- Catmull-Rom path smoothing
- Distance-based resampling
- Trapezoidal speed profiles
- Heading and unit conversion
- Basic holonomic wheel-distance conversion

Some code comments in the source acknowledge AI-assisted implementation. Review and test any generated robotics code carefully before use.
