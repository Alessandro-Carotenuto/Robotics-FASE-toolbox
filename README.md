# Robotics-FASE-Toolbox

A symbolic robotics toolbox built in Python using [SymPy](https://www.sympy.org/). It provides tools for modelling, analysing, and controlling robotic systems — both serial manipulators and autonomous mobile robots — using exact symbolic mathematics rather than numerical approximations.

---

## Architecture

```
robot.py                        # Base Robot class
├── manipulator.py              # Serial manipulator (kinematics + dynamics)
├── ROBOTS/amr/
│   ├── kinematic_model.py      # KinematicModel + KinematicPreset (constraints / presets)
│   ├── mobile.py               # Mobile robot class
│   └── simulator.py            # KinematicSimulator (Euler / RK4), Dynamic_Simulator
└── composed_robot.py           # Composed / hybrid robot systems

joints.py                       # Joint and link definitions (DH, inertia tensors)
utils.py                        # Helper functions (rotation matrices, skew-symmetric, joint string parser)
test.py                         # Scratch / experimentation scripts
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

#### Simulator (`simulator.py`)

Numerically integrate the kinematic model forward in time:

```python
from ROBOTS.amr.simulator import KinematicSimulator, StepType

sim = KinematicSimulator(model, method=StepType.RK4)
sim.step(robot, u=np.array([1.0, 0.5]), dt=0.01)
```

| Method | Description |
|---|---|
| `StepType.EULER` | First-order Euler integration |
| `StepType.RK4` | Fourth-order Runge-Kutta integration |

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

---

## Installation

```bash
git clone https://github.com/your-repo/Robotics-FASE-toolbox.git
cd Robotics-FASE-toolbox
pip install sympy
```

---

## Quick Example

```python
from manipulator import Manipulator, LinkBodyAssumptions
from sympy import pprint

# 2-DOF planar revolute arm, planar body assumption
robot = Manipulator("RR", assumptions=LinkBodyAssumptions.PLANAR)

# Inspect the inertia matrix
pprint(robot.M)

# Inspect the gravity vector
pprint(robot.G)
```

```python
from ROBOTS.amr.kinematic_model import KinematicModel, KinematicPreset
from ROBOTS.amr.mobile import Mobile
from ROBOTS.amr.simulator import KinematicSimulator, StepType
import numpy as np

model  = KinematicModel(preset=KinematicPreset.UNICYCLE)
robot  = Mobile(kinematic_model=model)
robot.q = np.array([0.0, 0.0, 0.0])

sim = KinematicSimulator(model, method=StepType.RK4)
sim.step(robot, u=np.array([1.0, 0.3]), dt=0.05)
```

---

## Roadmap / To-Do

### Manipulator
- [ ] Validate that the number of `LinkBodyAssumptions` entries matches the joint count
- [ ] Verbose debug output for intermediate calculation steps
- [ ] Dynamic parameter linearisation (`getDynamicCoefficients`): extract base parameters using `expand()` + `as_independent()`
- [ ] Reachable workspace computation
- [ ] Dextrous workspace computation
- [ ] Singularity detection and constraint analysis
- [ ] Inverse kinematics — numerical (iterative) and analytical
- [ ] Trajectory planning for manipulators
- [ ] Kinematic control
- [ ] Dynamic control
- [ ] Adaptive control
- [ ] Formal verification of Coriolis matrix against textbook formulations
- [ ] URDF file import/export
- [ ] Improved readability of symbolic outputs
- [ ] Performance: Propagated Jacobian Method, Screw Theory / Lie Algebra formulation
- [ ] Symbolic Recursive Newton-Euler algorithm

### Autonomous Mobile Robot (`ROBOTS/amr/`)
- [x] Derive kinematic model from constraints
- [x] Kinematic model presets (Unicycle, Bicycle RWD, Bicycle FWD)
- [x] Kinematic simulator — Euler and RK4 integration
- [ ] Feedback control
- [ ] Path planning — Artificial Potential Fields
- [ ] Path planning — RRT and RRT*
- [ ] Trajectory tracking
- [ ] Kalman Filter
- [ ] SLAM

### General / Infrastructure
- [ ] Expand `Composed_Robot` for hybrid mobile-manipulation systems
- [ ] Expand `Robot` base class with shared interface (control loop, state, etc.)
- [ ] Unit tests for kinematics and dynamics outputs
- [ ] Benchmarking symbolic computation time vs. numerical methods

---

## License

To be defined.