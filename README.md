<div align="center">

<img src="assets/icons/apex.png" alt="Apex open-wheel race car" width="200" />

# APEX

### Find your line.

An open-wheel racing simulator in development.<br />
Written in C++20, with custom vehicle physics and OpenGL rendering.

![C++20](https://img.shields.io/badge/C%2B%2B-20-103D43?style=flat-square)
![OpenGL](https://img.shields.io/badge/OpenGL-3.3-ED4023?style=flat-square)
![Status](https://img.shields.io/badge/status-prototype-103D43?style=flat-square)
[![License](https://img.shields.io/badge/license-MIT-ED4023?style=flat-square)](LICENSE)

[Get started](#get-started) &nbsp; / &nbsp; [Controls](#controls) &nbsp; / &nbsp; [Headless simulation](#headless-simulation) &nbsp; / &nbsp; [Under the hood](#under-the-hood)

</div>

---

Apex is being built from the ground up around vehicle motion, steering, and grip. The current prototype is a minimal top-down driving sandbox: a triangle represents the car, a circular circuit provides the road, and leaving the track adds drag and changes the vehicle's color.

Drive with the keyboard, or run the same simulation without a window to record trajectories, try the automatic driver, and inspect the results with Python.

It is an early foundation for an open-wheel racing simulation. Car models, race sessions, and lap timing are still ahead.

## The current drive

- **Throttle and reverse** — accelerate along the vehicle's heading, with drag reducing speed as you coast.
- **Steering that depends on speed** — turning follows forward motion and wheelbase; reversing changes the turning direction.
- **Lateral grip** — sideways velocity is damped independently of forward motion.
- **A circular circuit** — the vehicle appears vermilion on the road and yellow off track, where extra drag slows it down.
- **Physics at 120 Hz** — simulation advances in fixed steps, separately from rendering.
- **Pause, reset, and status** — pause the simulation, reset the vehicle, and read speed and track status in the window title.
- **Headless runs** — export a ten-second simulation to CSV using fixed controls or a feedback-based automatic driver.
- **Trajectory tools** — inspect run statistics and plot the vehicle's path and speed with Python.

## Get started

You will need a **C++20 compiler**, **CMake 3.24+**, **Git**, **Python 3**, and an **OpenGL 3.3 core** capable graphics driver for the interactive application. CMake fetches GLFW and glad during configuration; glad uses Python and Jinja2 to generate its loader. The trajectory scripts require Python 3.9 or newer, with Matplotlib for plotting.

On macOS, the icon build also requires a full Xcode installation with an `actool` version that supports Icon Composer `.icon` documents. `xcode-select -p` should point to that Xcode installation.

```sh
git clone https://github.com/steeve-pix/Apex.git
cd Apex

python3 -m venv .venv
.venv/bin/python -m pip install 'Jinja2>=2.7,<4.0'

cmake -S . -B build-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DPython_EXECUTABLE="$PWD/.venv/bin/python"
cmake --build build-release --config Release --parallel
```

**Launch on macOS**

```sh
open build-release/Apex.app
```

For terminal diagnostics, run `./build-release/Apex.app/Contents/MacOS/Apex` directly.

**Launch on Linux**

With a single-configuration generator such as Ninja or Unix Makefiles:

```sh
./build-release/apex
```

Linux also needs development packages for GLFW's enabled X11 or Wayland backends. macOS is the currently verified build platform.

## Controls

| Key | Action |
| :--- | :--- |
| `W` / `↑` | Apply forward throttle |
| `S` / `↓` | Apply reverse throttle |
| `A` / `←` | Steer left |
| `D` / `→` | Steer right |
| `Space` | Pause / resume |
| `R` | Reset the vehicle and simulation time |
| `T` | Toggle fixed trackside cameras that follow the selected car |
| `F` | Switch between chase views |
| `C` | Show the circuit from above |
| `Esc` | Quit |

Steering takes effect while moving. Reverse throttle slows forward motion before the vehicle begins reversing.

The window title displays speed in simulation units per second, whether the vehicle is on track, and whether the simulation is paused. Resetting preserves the current pause state.

## Headless simulation

`apex_headless` uses the same world and vehicle simulation as the interactive app, without opening a window. Each run advances ten simulated seconds at 120 Hz and writes CSV to standard output; a final time, position, and speed summary goes to standard error.

Run these examples from the repository root after building:

```sh
# Full forward throttle with no steering (the default).
./build-release/apex_headless > build-release/straight.csv

# Constant throttle and steering, each between -1 and 1.
./build-release/apex_headless 0.5 0.3 > build-release/turn.csv

# Automatic driving around the circular circuit.
./build-release/apex_headless --auto > build-release/automatic.csv
```

The automatic driver adjusts throttle and steering using forward speed, heading error, and distance from the track centerline. Its default target speed is 0.30 units/s. Automatic driving is currently available through the headless runner.

Each CSV contains an initial sample followed by 1,200 simulation steps:

```text
time,x,y,vx,vy,heading,on_track
```

Time is in seconds, positions and velocities use simulation units, heading is in radians, and `on_track` is `1` on the road or `0` off it.

### Analyze and plot a run

The analysis script uses Python's standard library. It reports sample count, duration, peak speed, and the first off-road sample, if any.

```sh
.venv/bin/python python/analyze_trajectory.py build-release/automatic.csv
```

Install the plotting dependency, then save a figure showing the vehicle's path and speed over time:

```sh
.venv/bin/python -m pip install -r python/requirements.txt
.venv/bin/python python/plot_trajectory.py build-release/automatic.csv \
  --output build-release/automatic.png
```

Plotting works without a display. These examples use a single-configuration generator; with a multi-configuration generator, use the executable's configuration subdirectory.

## Under the hood

| Layer | Responsibility |
| :--- | :--- |
| `include/math/` | Shared vector math |
| `include/simulation/` · `src/simulation/` | Body integration, vehicle dynamics, world stepping, track observations, and automatic driving |
| `include/graphics/` · `src/graphics/` | Shaders, geometry, and OpenGL resource ownership |
| `include/platform/` · `src/platform/` | GLFW lifecycle |
| `src/main.cpp` | Input, simulation timing, and the render loop |
| `src/headless_main.cpp` | Command-line controls, automatic runs, and CSV export |
| `python/` | Trajectory analysis and plotting |
| `tests/` | Physics, world stepping, track observations, and automatic driver checks |
| `cmake/` · `assets/icons/` | macOS app packaging and artwork |

The simulation builds as a separate library shared by the interactive app and headless runner. GLFW handles windowing and input, glad loads OpenGL functions, and C++ RAII manages graphics resources.

Run the checks after building with CMake's default `BUILD_TESTING=ON`:

```sh
ctest --test-dir build-release -C Release --output-on-failure
```

## Contributing

Bug reports and focused pull requests are welcome. Include your platform and reproduction steps for issues; discuss larger changes before starting work.

---

<div align="center">

[MIT licensed](LICENSE) &nbsp; · &nbsp; Built by [steeve-pix](https://github.com/steeve-pix)

</div>
