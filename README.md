# G1 kinematics app

Move the joints of the **Unitree G1** humanoid and watch the robot (*forward kinematics*), or drag a hand, a foot or the pelvis and read the joint angles (*inverse kinematics*). It runs in MATLAB and shows the real CAD meshes.

* **Forward kinematics** – one slider (or a typed angle) per joint, live 3D view, position and orientation of pelvis, torso, head, hands and feet, centre of mass, collision check.
* **Inverse kinematics** – move the target of a hand, foot, pelvis or torso and read the 29 joint angles, with their limits. Feet on the ground or fixed pelvis, centre of mass over the feet, optional self-collision avoidance.
* Save and load poses, save the 3D view as a PNG, replay a pose in MuJoCo.

## What you need

| | |
|---|---|
| **MATLAB** | R2022a or newer (tested on R2025b). Core MATLAB only: **no toolboxes** needed |
| **Operating system** | Linux, Windows or macOS. Tested on Ubuntu 22.04; Windows and macOS should work but have not been tested |
| **Computer** | any machine that runs MATLAB, no special graphics card; about 100 MB of free disk space (mostly the CAD meshes) |
| **Internet** | only to download the app |
| *Optional* | Python 3 with `pip install mujoco numpy`, only to replay poses in MuJoCo |

## 1. Download

Choose one of the two ways. Both give you the same folder.

### Option A – no git needed (easiest)

1. Open the repository page: <https://github.com/fedene97/g1_kinematics_app>
2. Click the green **Code** button, then **Download ZIP** (direct link: <https://github.com/fedene97/g1_kinematics_app/archive/refs/heads/main.zip>).
3. Unzip it where you want (see *Where to save it* below). You get a folder named `g1_kinematics_app-main`.

### Option B – with git

```bash
git clone https://github.com/fedene97/g1_kinematics_app.git
```

To choose the destination, add it at the end:

```bash
git clone https://github.com/fedene97/g1_kinematics_app.git ~/robots/g1_kinematics_app          # Linux / macOS
git clone https://github.com/fedene97/g1_kinematics_app.git C:\Users\me\robots\g1_kinematics_app  # Windows
```

To get the latest version later, run `git pull` inside the folder.

### Where to save it

Anywhere you like: Documents, a projects folder, a USB drive. The app finds everything **relative to its own folder**, so the only rule is: keep the folder together (do not move single files out of it). If the folder is read-only the apps still work; the files they write go to the temporary folder instead of `exports/`.

## 2. Start it in MATLAB

Open MATLAB, go to the folder where you saved the app, and run:

```matlab
cd('/path/where/you/saved/g1_kinematics_app')   % your own path (Windows: 'C:\Users\me\...')
g1_app test      % optional self-test, a few seconds
g1_app           % opens the launcher
```

Don't want to type the path? Use `cd(uigetdir)` and pick the folder in the window that opens, or browse to it in MATLAB's *Current Folder* panel.

You can also start an app directly: `g1_app fk` (forward kinematics) or `g1_app ik` (inverse kinematics). If you prefer not to change folder, `addpath('/path/where/you/saved/g1_kinematics_app')` once, then `g1_app` works from anywhere.

The first start takes a few seconds (the model is built from the URDF and the CAD meshes are loaded).

**The self-test should end like this:**

```
G1 kinematics app - self-test
G1 model ready: 29 joints, 33 capsules (1.9 s)
  [ok  ] model: 29 joints
  [ok  ] collision capsules built
  [ok  ] g1_fk: feet on the ground
  ...
  [ok  ] inverse kinematics: hand reaches the target
All 10 checks passed.
```

## 3. Using the apps

**Forward kinematics** (`g1_app fk`) – one tab per body part (left leg, right leg, waist, left arm, right arm). Drag a slider, or type the angle in degrees in the box next to it. The table shows where pelvis, torso, head, hands, feet and the centre of mass are. Buttons: *Zero pose*, *Bent knees*, *Save pose*, *Load pose*.

**Inverse kinematics** (`g1_app ik`) – pick a point of the robot in *TCP to move* (hands, feet, pelvis, torso) and move its target with the sliders or by typing: dx, dy, dz in metres, droll, dpitch, dyaw in degrees, relative to where it starts. The robot follows and the table on the right shows the 29 joint angles; a joint that has reached its limit turns red. *Try it:* choose *Left hand* and move *dx* and *dz*.

| Control | Meaning |
|---|---|
| Mode: Feet on ground | the feet stay on the ground, the rest of the body adapts |
| Mode: Fixed pelvis | the pelvis does not move |
| Active (follows target) | this TCP follows its target (moving a slider switches it on) |
| Control orientation | follow the full pose of the TCP, not only its position |
| CoM over feet | keep the centre of mass above the point between the feet |
| Avoid collisions | stop the body parts from going through each other |

**Display tab** (both apps) – show or hide the CAD meshes, skeleton, collision capsules, coordinate frames, and legs / trunk / arms separately; mesh opacity; camera views (Front, Side, Top, 3D); **Save image (PNG)**.

**Pose files** – *Save pose* writes the 29 joint angles to a `.json` or `.csv` file (plain text, in degrees); *Load pose* reads them back. By default they go to the `exports/` folder.

## Troubleshooting

| Problem | What to do |
|---|---|
| `Undefined function or variable 'g1_app'` | MATLAB is not in the app folder. Run `cd(uigetdir)` and select the folder, or use `addpath` as shown above |
| `URDF not found ...` | The folder `urdf/g1_description` is missing: download the app again (the ZIP and the clone both include it) |
| `Undefined function 'uigridlayout'` (or `uieditfield`, `exportgraphics`) | MATLAB is too old: use R2022a or newer |
| A warning says the collision capsules were not built | The meshes in `urdf/g1_description/meshes` are missing or unreadable. The apps still work, without the CAD view and the collision check |
| The window is too big or too small for the screen | It adapts to the screen at start; you can also resize or maximise it |
| MuJoCo viewer does not open on macOS | Run the script with `mjpython` instead of `python3` |

If something else goes wrong, please open an issue and include the full MATLAB error message and your MATLAB version (`ver` or `version`).

## More

* [`docs/SCRIPTING.md`](docs/SCRIPTING.md) – use the model from your own MATLAB scripts, conventions, MuJoCo replay, repository layout.
* [`docs/FUNCTIONS.md`](docs/FUNCTIONS.md) – index of the functions (and an Italian–English glossary: many internal names are Italian).

## Limitations

* Kinematics only: no dynamics, no contact forces. The collision model is a set of capsules fitted to the meshes: an approximation that agrees with the mesh-based MuJoCo contacts on about 90 % of random poses.
* The ankle is modelled by the pitch and roll joints of the URDF. The real G1 drives them with two motors through a linkage, which this app does not model.
* Joint limits are those of the URDF; check them against your robot before using the angles on hardware.

## License

The code is released under the MIT license ([`LICENSE`](LICENSE)). The robot description in `urdf/g1_description/` (URDF, MuJoCo model, meshes) is by Unitree Robotics under the BSD 3-Clause license, see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md).
