# urdf/

The Unitree G1 description package lives in `urdf/g1_description/`. The app uses:

```
urdf/g1_description/
    g1_29dof.urdf        kinematic model (read by the app)
    g1_29dof.xml         MuJoCo model (only used by python/view_pose.py)
    meshes/              STL meshes (CAD view and collision capsules)
```

The other files of the package (23-DoF and hand variants, images, notebooks) are kept so that the package stays intact. It comes from Unitree's `unitree_ros` repository (`robots/g1_description`) under the BSD 3-Clause license, see [`g1_description/LICENSE`](g1_description/LICENSE) and [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md). If the meshes are missing the apps still start, but without the CAD view and without collision checking.
