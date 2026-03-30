# Robotics-FASE-Toolbox

A symbolic robotics toolbox built in Python using [SymPy](https://www.sympy.org/). It provides tools for modelling, analysing, and controlling robotic systems — both serial manipulators and autonomous mobile robots — using exact symbolic mathematics rather than numerical approximations.

---

## Architecture

```
robot.py                              # Base Robot class
├── manipulator.py                    # Serial manipulator (kinematics + dynamics)
├── ROBOTS/amr/
│   ├── kinematic_model.py            # KinematicModel + KinematicPreset (constraints / presets)
│   └── mobile.py                     # Mobile robot class
└── composed_robot.py                 # Composed / hybrid robot systems

SIMULATIONS/
├── simulator.py                      # KinematicSimulator (Euler / RK4), Dynamic_Simulator
├── environment.py                    # Environment (multi-robot orchestration)
└── displayer.py                      # Displayer (static + animated rendering)

joints.py                             # Joint and link definitions (DH, inertia tensors)
utils.py                              # Helper functions (rotation matrices, skew-symmetric, joint string parser, Pfaffian)
test.py                               # Scratch / experimentation scripts
```

---

## Features

### Manipulator (`manipulator.py`)

Instantiate a manipulator from a **joint string** (e.g. `"RRR"` for a 3-DOF revolute arm, `"2R1P"` for two revolute + one prismatic):

```python
from manipulator import Manipulator, LinkBodyAssumptions

robot = Manipulator("RRR", assumptions=LinkBodyAssumptions.CYLINDRIC)
```

On construction, the following are computed **symbolically and automatically**:

| Attribute | Description |
|-----------|-------------|
| `robot.FKlist` | Forward kinematics chain (list of `4×4` homogeneous transforms) |
| `robot.J` | Geometric Jacobian `(J, Jv, Jw)` — full, linear, and angular parts |
| `robot.T` | Total kinetic energy of the system |
| `robot.M` | Inertia (mass) matrix |
| `robot.c` | Coriolis vector |
| `robot.G` | Gravity vector |

#### Setting DH Parameters

```python
from sympy import pi

robot.setDHParameters(
    param_list=["alpha", "a", "d"],
    index_list=[0, 0, 0],
    value_list=[pi/2, 0, 0]
)
```

All downstream quantities (FK, Jacobian, dynamics) are automatically recomputed.

---

### Joints and Links (`joints.py`)

Each joint holds its DH parameters as SymPy symbols and supports multiple **inertia tensor assumptions** to simplify the dynamic model:

| Assumption | Description |
|---|---|
| `GENERAL` | Full symmetric 3×3 inertia tensor |
| `DIAGONAL` | Off-diagonal terms set to zero |
| `CYLINDRIC` | Diagonal + `Iyy = Ixx` |
| `SPHERE` | Diagonal + `Iyy = Izz = Ixx` |
| `THIN_ROD` | Diagonal + `Iyy = Ixx`, `Izz = 0` |
| `PLANAR` | Diagonal + `Ixx = Iyy = 0` |
| `POINT_MASS` | All inertia terms zero |

---

### Autonomous Mobile Robot (`ROBOTS/amr/`)

#### Kinematic Model (`kinematic_model.py`)

Build the kinematic model `q_dot = G(q) * u` either from explicit Pfaffian constraints or from a built-in preset:

```python
from ROBOTS.amr.kinematic_model import KinematicModel, KinematicPreset

# From preset
model = KinematicModel(preset=KinematicPreset.UNICYCLE)
model = KinematicModel(preset=KinematicPreset.BICYCLE_RWD)

# From constraints
model = KinematicModel(constraints=[C1, C2], coords=['x_f', 'y_f', 'theta', 'phi'])
```

Available presets:

| Preset | Reference point | Coordinates | Inputs |
|---|---|---|---|
| `UNICYCLE` | Centre | `[x, y, θ]` | `[v, ω]` |
| `BICYCLE_RWD` | Rear axle | `[x, y, θ, φ]` | `[v, φ_dot]` |
| `BICYCLE_FWD` | Front axle | `[x_f, y_f, θ, φ]` | `[v_f, φ_dot]` |
| `CAR_WITH_TRAILER` | Car rear axle | `[x, y, θ, φ, β]` | `[v, φ_dot]` |

The `CAR_WITH_TRAILER` preset also exposes an `extra_points` callback, used by the `Displayer` to animate the trailer body alongside the car.

---

#### Simulator (`SIMULATIONS/simulator.py`)

Numerically integrate the kinematic model forward in time:

```python
from SIMULATIONS.simulator import KinematicSimulator, StepType

sim = KinematicSimulator(robot, method=StepType.RK4)
sim.step(robot, u=np.array([1.0, 0.5]), dt=0.01)
```

| Method | Description |
|---|---|
| `StepType.EULER` | First-order Euler integration |
| `StepType.RK4` | Fourth-order Runge-Kutta integration |

The simulator validates that all physical parameters (e.g. wheelbase `l`) have been substituted before lambdifying `G(q)`.

---

#### Environment (`SIMULATIONS/environment.py`)

Orchestrates multiple robots in a shared simulation loop:

```python
from SIMULATIONS.environment import Environment

env = Environment(robotlist=[r1, r2, r3])

env.setCommand(r1, np.array([1.0, 0.1]))
env.setCommand(r2, np.array([0.8, 0.2]))

env.run(steps=200, dt=0.05)
```

Each robot gets its own `KinematicSimulator`. Trajectories are logged automatically at every step and accessible via `env.trajectories`.

---

#### Displayer (`SIMULATIONS/displayer.py`)

Renders trajectories from an `Environment`:

```python
from SIMULATIONS.displayer import Displayer

disp = Displayer(env)
disp.display("Static Plot")        # static trajectory plot
disp.animate_display("Animation")  # frame-by-frame animated plot
```

The `animate_display` method also renders **extra points** (e.g. trailer position) as dashed lines when the kinematic model defines an `extra_points` callback.

---

### Utility Functions (`utils.py`)

| Function | Description |
|---|---|
| `rot_x(θ)` | 3×3 rotation matrix about X |
| `rot_y(θ)` | 3×3 rotation matrix about Y |
| `rot_z(θ)` | 3×3 rotation matrix about Z |
| `skew(v)` | Skew-symmetric matrix for cross-product `v × w` |
| `validate_joint_string(s)` | Validates joint string syntax (e.g. `"2R1P"`) |
| `process_joint_string(s)` | Expands shorthand (e.g. `"2R"` → `"RR"`) |
| `expression_to_sympy(s)` | Parses a constraint string into a SymPy expression |
| `Pfaffian_from_constraints(c, coords)` | Builds the Pfaffian matrix `A^T(q)` from constraint list |
| `KinematicModelFromConstraints(c, coords)` | Full pipeline: constraints → `G(q)`, `q_dot`, velocity symbols |

---

## Installation

```bash
git clone https://github.com/your-repo/Robotics-FASE-toolbox.git
cd Robotics-FASE-toolbox
pip install sympy numpy matplotlib
```

---

## Quick Examples

### Serial Manipulator

```python
from manipulator import Manipulator, LinkBodyAssumptions
from sympy import pprint

# 2-DOF planar revolute arm, planar body assumption
robot = Manipulator("RR", assumptions=LinkBodyAssumptions.PLANAR)

pprint(robot.M)   # inertia matrix
pprint(robot.G)   # gravity vector
```

### Mobile Robot — Single Unicycle

```python
from ROBOTS.amr.kinematic_model import KinematicModel, KinematicPreset
from ROBOTS.amr.mobile import Mobile
from SIMULATIONS.simulator import KinematicSimulator, StepType
import numpy as np

model = KinematicModel(preset=KinematicPreset.UNICYCLE)
robot = Mobile(kinematic_model=model)

sim = KinematicSimulator(robot, method=StepType.RK4)
sim.step(robot, u=np.array([1.0, 0.3]), dt=0.05)
```

### Multi-Robot Environment with Animation

```python
from ROBOTS.amr.kinematic_model import KinematicModel, KinematicPreset
from ROBOTS.amr.mobile import Mobile
from SIMULATIONS.environment import Environment
from SIMULATIONS.displayer import Displayer
from sympy import symbols
import numpy as np

unicycle    = KinematicModel(preset=KinematicPreset.UNICYCLE)
bicycle     = KinematicModel(preset=KinematicPreset.BICYCLE_RWD)
car_trailer = KinematicModel(preset=KinematicPreset.CAR_WITH_TRAILER)

r1 = Mobile(kinematic_model=unicycle)
r2 = Mobile(kinematic_model=bicycle,     physical_parameters={symbols('l'): 1.0})
r3 = Mobile(kinematic_model=car_trailer, physical_parameters={symbols('l'): 2.5, symbols('L'): 3.0})

env = Environment(robotlist=[r1, r2, r3])
env.setCommand(r1, np.array([1.0, 0.1]))
env.setCommand(r2, np.array([1.0, 0.1]))
env.setCommand(r3, np.array([1.0, 0.1]))

env.run(steps=200, dt=0.05)

disp = Displayer(env)
disp.animate_display("Multi-Robot Simulation")
```

---

## Roadmap / To-Do

<details><summary><span style="color:orange">Manipulator</span></summary>

### Manipulator

<details><summary>Minor Test and fixes</summary>

#### Minor Test and fixes

- [ ] Validate that the number of `LinkBodyAssumptions` entries matches the joint count
- [ ] Verbose debug output for intermediate calculation steps
- [ ] Formal verification of Coriolis matrix against textbook formulations
- [ ] Improved readability of symbolic outputs
</details>

<details><summary>Major Updates</summary>

#### Major Updates
- [ ] Reachable workspace computation
- [ ] Dextrous workspace computation
- [ ] Singularity detection and constraint analysis
- [ ] Dynamic parameter linearisation (`getDynamicCoefficients`): extract base parameters using `expand()` + `as_independent()`
- [ ] Inverse kinematics, Numerical
- [ ] Trajectory planning for manipulators
- [ ] Kinematic control
- [ ] Dynamic control
- [ ] Adaptive control
- [ ] Symbolic Recursive Newton-Euler algorithm
- [ ] Inverse kinematics, Analytical
</details>

<details><summary>Performance Improvements</summary>

#### Performance (manipulators)
- [ ] Propagated Jacobian Method, Screw Theory / Lie Algebra formulation
</details>


<details><summary>IN/OUT Updates</summary>

#### IN/OUT Updates
- [ ] URDF file import/export

</details>
</details>


<details><summary><span style="color:orange">Autonomous Mobile Robot</span></summary>
    
### Autonomous Mobile Robot
- [x] Derive kinematic model from constraints (Pfaffian null-space method)
- [x] Kinematic model presets — Unicycle, Bicycle RWD, Bicycle FWD
- [x] Kinematic model preset — Car with trailer (5-DOF, articulation angle `β`)
- [x] `extra_points` callback on `KinematicModel` for articulated body rendering
- [x] Kinematic simulator — Euler and RK4 integration
- [x] Physical parameter substitution and validation at simulator construction
- [x] Multi-robot `Environment` — shared simulation loop, per-robot command assignment, trajectory logging
- [x] `Displayer` — static trajectory plot and frame-by-frame animation (with extra-point support)
- [X] Lie Bracket for controllability analysis: case of 2 vectors
- [X] Lie Bracket for controllability analysis: general case
- [X] Controllability analysis leveraging Lie Bracket results
- [ ] Feedback control : With Manually inputed control law
- [ ] Feedback control : With Preset tunable control laws (e.g. unicycle position control)
- [ ] Path planning — Artificial Potential Fields
- [ ] Path planning — RRT and RRT*
- [ ] Trajectory tracking
- [ ] Kalman Filter & Extended Kalman Filter
- [ ] SLAM

<details><summary>Performance Improvements</summary>

#### Performance (amr)
- [ ] OPTIMIZE ANIMATION WITH SET DATA
</details>
</details>

<details><summary><span style="color:orange">General / Infrastructure</span></summary>

### General / Infrastructure
- [ ] Expand `Composed_Robot` for hybrid mobile-manipulation systems
- [ ] Expand `Robot` base class with shared interface (control loop, state, etc.)
- [ ] Unit tests for kinematics and dynamics outputs
- [ ] Benchmarking symbolic computation time vs. numerical methods

</details>