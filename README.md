# Robotics-FASE-Toolbox

<p align="center">
  <img src="https://img.shields.io/badge/Python-3.10+-3776AB?logo=python&logoColor=white" alt="Python" height="28" />
  <img src="https://img.shields.io/badge/SymPy-symbolic_math-3B5526?logo=sympy&logoColor=white" alt="SymPy" height="28" />
  <img src="https://img.shields.io/badge/NumPy-013243?logo=numpy&logoColor=white" alt="NumPy" height="28" />
  <img src="https://img.shields.io/badge/Matplotlib-visualisation-11557C" alt="Matplotlib" height="28" />
  <img src="https://img.shields.io/badge/Status-In_Progress-yellow" alt="Status" height="28" />
</p>

*Alessandro Carotenuto*

A symbolic robotics toolbox built in Python using SymPy. Provides tools for modelling, analysing, and controlling robotic systems — autonomous mobile robots, underactuated and legged systems, and serial manipulators — using exact symbolic mathematics rather than numerical approximations.

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

### Serial Manipulator

```python
from manipulator import Manipulator, LinkBodyAssumptions
from sympy import pprint

# 2-DOF planar revolute arm, planar body assumption
robot = Manipulator("RR", assumptions=LinkBodyAssumptions.PLANAR)

pprint(robot.M)   # inertia matrix
pprint(robot.G)   # gravity vector
```

---

## Roadmap

### Autonomous Mobile Robot

- [x] Derive kinematic model from constraints (Pfaffian null-space method)
- [x] Kinematic model presets — Unicycle, Bicycle RWD, Bicycle FWD
- [x] Kinematic model preset — Car with trailer (5-DOF, articulation angle `β`)
- [x] `extra_points` callback on `KinematicModel` for articulated body rendering
- [x] Kinematic simulator — Euler and RK4 integration
- [x] Physical parameter substitution and validation at simulator construction
- [x] Multi-robot `Environment` — shared simulation loop, per-robot command assignment, trajectory logging
- [x] `Displayer` — static trajectory plot and frame-by-frame animation (with extra-point support)
- [X] Lie Bracket for controllability analysis — case of 2 vectors
- [X] Lie Bracket for controllability analysis — general case
- [X] Controllability analysis leveraging Lie Bracket results
- [ ] Involutivity check of a distribution — verify closure under Lie bracket (Frobenius theorem)
- [ ] Partial / Complete Integrability (PI / CI) — classify passive dynamics from the involutive closure
- [ ] Small-Time Local Accessibility (STLA) — rank test of the LARC distribution at a given point
- [ ] Small-Time Local Controllability (STLC) — Philip Hall basis conditions on odd-degree brackets
- [ ] Driftless system property — expose `is_driftless` flag on `KinematicModel`
- [ ] Feedback control — manually specified control law
- [ ] Feedback control — preset tunable control laws (e.g. unicycle position control)
- [ ] Path planning — Artificial Potential Fields
- [ ] Path planning — RRT and RRT*
- [ ] Trajectory tracking
- [ ] Kalman Filter & Extended Kalman Filter
- [ ] SLAM

#### Performance
- [ ] Optimize animation with `set_data`

---

### Underactuated & Nonlinear Control

- [ ] Energy-based Control (swing-up) — Lyapunov energy function + switching strategy for underactuated systems
- [ ] Phase plane visualisation — 2D vector field and orbit plotting utility in `Displayer`
- [ ] Virtual Constraints — define and enforce output functions on underactuated coordinates
- [ ] Lie Derivatives (`LfH`, `LgH`) — symbolic `∇h · f` operator, building block for FBL and relative degree
- [ ] Relative degree computation — iterate Lie derivatives until `Lg(Lf^k h) ≠ 0`
- [ ] Feedback Linearization (collocated) — compute the exact linearising law `τ = M(q)v + c + G` symbolically
- [ ] Zero Dynamics — derive internal dynamics after input-output linearisation (normal form coordinates z, η)

---

### Legged Robots

- [ ] `LeggedRobot` base class — floating base with SE(3) configuration (position + quaternion)
- [ ] Zero Moment Point (ZMP) and Support Polygon computation
- [ ] Linear Inverted Pendulum (LIP) model and 3D ZMP dynamics
- [ ] Divergent Component of Motion (DCM / Capture Point) — stable/unstable decomposition
- [ ] Pyramidal friction cone (feasibility region) for contact forces
- [ ] Single-Rigid-Body Dynamics (SRBD) and Centroidal Dynamics
- [ ] Preview Control (Kajita) — CoM jerk as input, ZMP preview horizon

---

### Manipulator

#### Minor fixes & tests
- [ ] Validate that the number of `LinkBodyAssumptions` entries matches the joint count
- [ ] Verbose debug output for intermediate calculation steps
- [ ] Formal verification of Coriolis matrix against textbook formulations
- [ ] Improved readability of symbolic outputs

#### Major updates
- [ ] Reachable workspace computation
- [ ] Dextrous workspace computation
- [ ] Singularity detection and constraint analysis
- [ ] Dynamic parameter linearisation (`getDynamicCoefficients`)
- [ ] Inverse kinematics — Numerical
- [ ] Inverse kinematics — Analytical
- [ ] Trajectory planning
- [ ] Kinematic control
- [ ] Dynamic control
- [ ] Adaptive control
- [ ] Symbolic Recursive Newton-Euler algorithm
- [ ] Cyclic coordinate detection — identify coordinates absent from M and G
- [ ] LQR design for linearised manipulator — Riccati recursion on the linearised (A, B)
- [ ] iLQR / DDP trajectory optimisation — backward/forward pass on the full nonlinear dynamics

#### Performance
- [ ] Propagated Jacobian Method, Screw Theory / Lie Algebra formulation

#### I/O
- [ ] URDF file import/export

---

### General / Infrastructure

- [ ] Expand `Composed_Robot` for hybrid mobile-manipulation systems
- [ ] Expand `Robot` base class with shared interface (control loop, state, etc.)
- [ ] Unit tests for kinematics and dynamics outputs
- [ ] Benchmarking symbolic computation time vs. numerical methods
