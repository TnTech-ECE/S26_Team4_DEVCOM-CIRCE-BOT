# circe_description/meshes

Chassis, tracks, and wheels are NOT here. They load from `roverrobotics_description/meshes/`
(`flipper_chassis.dae`, `flipper_track.dae`, `pro_tire.dae`) rather than being duplicated —
that package is already in the workspace, so copying 24 MB of `.dae` would buy nothing. See
the `roverrobotics_description` `exec_depend` in `package.xml`.

The sensor mast is modelled with primitives (box/cylinder) because it is a CIRCE part with
no mesh. The D456 uses `realsense2_description/meshes/d455.stl` - see below.

## RPLidar A1

`rplidar_a1.stl` is the manufacturer CAD of the A1M8, used for the VISUAL in
`urdf/accessories/rplidar_a1.urdf.xacro`. Collision there stays a datasheet box on
purpose; the note in that file says why.

- **Millimetres**, `scale="0.001 0.001 0.001"`. **+Y is the spin axis.**
- Turret spin axis at `x=37.101, z=36.518`, fixed to 0.04 mm by fitting the two complete
  circles the model presents (solid top cap, turret floor) - they agree to `dx=0.003`,
  `dz=0.042` mm. The cap measures **70.00 mm** across, the datasheet width exactly.
- The turret's transparent window band runs `y=34..49`, so the optical midline - the scan
  plane - is at **`y=41.5`**. That is the point aligned to `lidar_link`'s origin, because
  `lidar_link` *is* the scan plane. It avoids depending on the A1's ambiguous "base plate"
  datum: this CAD has no flat bottom, the motor hangs below the mounting plate.
- Body is **51.0 mm** tall, not the 60.0 mm the URDF assumed. The scan plane still checks
  out to 0.5 mm; the whole discrepancy is above it. See the warning on `a1_total_height` in
  `urdf/accessories/circe_sensor_mast.urdf.xacro` before "fixing" that.

It was **decimated from 2,436,776 triangles to 34,720** (122 MB -> 1.7 MB) by vertex
clustering on a 1.0 mm grid, so no surface point moved more than about 0.9 mm, and the
bounding box changed by under 1 mm.

The full-resolution original now lives in **`circe_ws/cad_source/`**, deliberately outside
every ROS package. It is 122 MB - five times the entire `roverrobotics_description` mesh set
this file opens by refusing to duplicate - and `CMakeLists.txt` installs this whole
directory, so leaving it here meant copying all of it into `install/` on every non-symlink
build. Do not move it back; re-decimate from `cad_source/` if the model is ever revised.

## RealSense D456

`D456_Solid.stl` is **not used**, and integrating it would be a regression rather than an
improvement. `realsense2_description`'s `d455.stl` already renders this exact body:

| | upstream `d455.stl` | `D456_Solid.stl` |
| --- | --- | --- |
| size | 123.99 x 29.00 x 26.00 mm | 124.0 x 29.1 x 26.0 mm |
| mesh origin | -62.00, -14.50, -26.00 mm | -62.0, -14.6, -26.0 mm |
| triangles | 51,162 | 11,568 |

Same geometry, same origin, same frame - only the units differ (upstream mm, this one
metres). The D456 is the IP65/M12 D455 and shares its case. The camera is built entirely by
upstream's `sensor_d455` macro, which owns `camera_link`, every optical frame and the
`use_nominal_extrinsics` sim/hardware split; substituting the visual means bypassing or
patching that macro. Paying that for an identical silhouette and 40 k fewer triangles is not
a trade worth making.

## Cable spool

`spool_*.stl` are the tether spool, used by `urdf/accessories/circe_cable_spool.urdf.xacro`.

Six parts exported from one SolidWorks assembly. **They share a single assembly coordinate
system** — every part is placed by the same transform, and none of them was re-origined by
hand. Do not "helpfully" re-centre them individually; that is what would turn a short xacro
into six hand-fitted offsets.

| file | role |
| --- | --- |
| `spool_drum.stl` | the 6 in Cat6 drum — the only part that turns |
| `spool_motor_end_block.stl` | driven-end pillow block |
| `spool_motor_end_block_top.stl` | its bearing cap |
| `spool_free_end_block.stl` | idle-end pillow block |
| `spool_free_end_block_top.stl` | its bearing cap |
| `spool_motor.stl` | gearmotor |

Properties derived from the geometry, recorded here so nobody has to re-derive them:

- **Units are millimetres.** Every mesh needs `scale="0.001 0.001 0.001"`.
- **Mesh +X is the rotation axis, +Y is up, and `y=0` is the machined mounting face.**
  The axis is the line `y=139.700, z=132.807`; both the drum's and the motor's centres of
  mass sit on it. The drum's inertia about X is half its transverse inertia, as a solid of
  revolution must be.
- Assembly envelope 278.1 × 254.0 × 228.6 mm (10.95 × 10.00 × 9.00 in); bolt-down footprint
  259.1 × 184.2 mm; drum 228.6 mm (9 in) diameter × 223.5 mm (8.8 in) long.
- Volumes, by exact integration over the closed mesh: drum 1280.9 cm³, the five static parts
  2312.4 cm³ together. The xacro turns these into mass and inertia — see the mass note there
  before trusting either figure.

The files were renamed on import from their SolidWorks names
(`6in Spool Assembly - Motor End-1.STL` and friends). **The original names contain spaces,
which are not valid in a `file://` URI** and fail to load in RViz and gz-sim. Keep new mesh
names free of spaces.

They are also still ASCII STL, which is roughly 10× the size of the equivalent binary STL
(2.0 MB against ~0.2 MB). Converting is lossless at this precision if the size ever matters.
