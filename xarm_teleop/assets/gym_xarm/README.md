# Asset source

These XML and STL files come from the local installation of `gym-xarm` 0.1.1,
which identifies its source as the user's `gym-xarm` fork. The upstream project
is [huggingface/gym-xarm](https://github.com/huggingface/gym-xarm).
The Apache 2.0 license is retained in `LICENSE`.

Only assets are used. The upstream Python environment is not a dependency.
The local source's camera layout is retained. Unused scene XML and meshes were
omitted. `push.xml` changes cube density from 50,000 to 500 kg/m³, giving the
48 mm cube a mass of about 55 g. This makes the lift task suitable for this test.

The Python wrapper sets the MuJoCo 3 weld relative pose explicitly at load time.
The mocap target is a flange pose; the public action is the `grasp` site's pose.
`env.py` converts between them, including the rotated tool offset.

This model is useful for control and data-pipeline tests. Its mocap weld and
gripper do not reproduce the UFACTORY controller's dynamics or pulse calibration.
