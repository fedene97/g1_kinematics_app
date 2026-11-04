# Function index

The apps and the functions starting with `g1_` are documented in English. The kernel keeps the Italian names of the workspace it was extracted from; `help <name>` explains each function (the older help texts are in Italian).

## Glossary of the Italian names

| Italian | English | | Italian | English |
|---|---|---|---|---|
| cinematica diretta | forward kinematics | | compito | (IK) task |
| posa | pose | | priorità | priority |
| giunto / giunti | joint(s) | | postura | posture |
| baricentro | centre of mass | | postura casa | home posture (bent knees and elbows) |
| incremento | increment | | terreno | ground |
| frame | frame | | capsula | capsule |
| creare / leggi | build / read | | distanze | distances |
| punto | point | | mondo | world |

## Entry points and app support (`src/app`)

| Function | What it does |
|---|---|
| `g1_app` | launcher (`g1_app`, `g1_app fk`, `g1_app ik`, `g1_app test`) |
| `g1_gui`, `g1_gui_ik` | the forward and inverse kinematics apps |
| `g1_fk` | joint angles → poses of pelvis, head, hands, feet, centre of mass, collisions (plot / table optional) |
| `g1_model` | builds the kinematic model and the collision capsules from the URDF (cached per session) |
| `g1_setup` | adds `src/` to the path and returns the repository paths |
| `g1_save_pose`, `g1_load_pose` | pose files (`.json`, `.csv`) |
| `g1_tcp_list`, `g1_window_pos` | small helpers of the apps |
| `vista_g1`, `vista_g1_aggiorna`, `vista_g1_visibilita`, `controlli_vista`, `disegna_posa` | 3D view: CAD meshes on one transform per body, visibility, display panel, stick figure |

## Model (`src/model`)

| Function | What it does |
|---|---|
| `leggi_urdf` | reads links and joints of a URDF (plain XML, no toolbox) |
| `crea_modello_cinematico` | builds the kinematic tree: axes, offsets, limits, masses, sole points |
| `stato_cin` | configuration struct: `x.pb` (3x1), `x.Rb` (3x3), `x.qj` (29x1) |
| `postura_casa` | non-singular starting posture |
| `trova_frame` | index of a frame from its name |

## Kinematics (`src/kinematics`)

| Function | What it does |
|---|---|
| `cinematica_diretta` | poses of all bodies in the world frame, O(n) recursion (numeric or symbolic input) |
| `posa_frame` | orientation and origin of a frame |
| `jacobiano_frame`, `jacobiano_punto` | 6 x 35 geometric Jacobian of a frame / of a point of a body (`'mondo'`, `'locale'`, `'spaziale'`) |
| `bias_punto` | `Jdot*nu` term of a point |
| `baricentro` | centre of mass, its Jacobian and `Jdot*nu` |
| `applica_incremento` | configuration update on SE(3) × joints |

## Inverse kinematics (`src/ik`)

`compito` defines a task (frame pose, centre of mass, posture, collision avoidance), `ik_risolvi` solves a stack of prioritised tasks with selective damping, joint limits (active set), step acceptance and random restarts.

## Collisions (`src/collision`)

`crea_capsule` fits capsules to the meshes (several for torso and pelvis); `distanza_segmenti`, `distanze_capsule`, `distanze_terreno` compute distances between capsules and from the ground; `disegna_capsule` draws them.

## Utilities (`src/utils`)

SO(3) helpers (`skew`, `vee`, `so3_exp`, `so3_log`, `rot_asse`, `rpy2rot`, `rpy_da_rot`, `quat_da_rot`), `scrivi_pose_csv` (CSV for the MuJoCo viewer), `completa_opzioni` (default options), `md5_file`, `quantile_semplice`, `pulisci`, `costante_sym` (symbolic support).
