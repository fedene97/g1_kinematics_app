# urdf/

The app expects the Unitree G1 description package here:

```
urdf/g1_description/
    g1_29dof.urdf        kinematic model (read by the app)
    g1_29dof.xml         MuJoCo model (only used by python/view_pose.py)
    meshes/              STL meshes (CAD view and collision capsules)
```

The folder comes from Unitree's `unitree_ros` repository (`robots/g1_description`); see [`../THIRD_PARTY_NOTICES.md`](../THIRD_PARTY_NOTICES.md) for the license. If the meshes are missing the apps still start, but without the CAD view and without collision checking.
