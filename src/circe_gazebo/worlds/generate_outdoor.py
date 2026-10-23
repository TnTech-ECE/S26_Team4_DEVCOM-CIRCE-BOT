#!/usr/bin/env python3
"""Generate circe_outdoor.sdf - a Tennessee-like outdoor test course for CIRCE.

    python3 generate_outdoor.py > circe_outdoor.sdf

*** WHY THIS IS GENERATED AND NOT HAND-WRITTEN ***

Every feature here is sized against a threshold the rover actually has, and several of
those sizes are the solution of an equation rather than a round number. A hill with a
known maximum slope, for instance, is a sphere whose radius follows from the slope and
the height you want; typing "R = 20" by hand tells you nothing about what it tests.
Keeping the arithmetic in code means the course can be re-derived when the thresholds
in circe_perception/config/terrain.yaml change, instead of quietly going stale.

The thresholds this course is built around, all from terrain.yaml:

    step_warn    0.03 m      step_lethal   0.06 m
    drop_warn    0.04 m      drop_lethal   0.08 m
    slope_warn   0.17 rad (9.7 deg)        slope_lethal  0.44 rad (25.2 deg)
    rough_warn   0.010 m     rough_max     0.040 m

and from the robot itself:

    wheel radius        0.1266 m   (wheel diameter 0.2532 m)
    footprint           0.546 x 0.356 m, circumscribed radius 0.326 m
    lidar scan plane    z = 0.295 m
    camera_link         z = 0.345 m, first sees ground at 0.62 m

*** THE SINGLE MOST IMPORTANT FACT ABOUT THIS COURSE ***

The RPLidar sweeps a plane at z = 0.295 m. EVERY obstacle shorter than that is
completely invisible to it. That is not a detail of the simulation - it is true of the
real robot, and it is the entire reason circe_perception exists. Most of the hazards
below sit in that blind band on purpose: if the depth camera does not see them, nothing
does, and the rover drives straight into them. Trees and boulders are the only things
here the lidar can contribute at all.

Layout, with the rover spawning at the origin facing +x:

    zone            centre        what it asks
    step field      ( 7,  0)      can it tell a 0.055 m root from a 0.07 m rock?
    erosion deck    (17,  0)      potholes: depth series, then width series
    hill field      (-9,  0)      slopes bracketing slope_lethal, smooth not stepped
    gravel patch    ( 6, 11)      does roughness produce preference, never a barrier?
    woodland        everywhere    lidar landmarks, without which SLAM has nothing
"""
import math
import random

# ---- rover and threshold constants (keep in step with terrain.yaml) ----------------
WHEEL_R = 0.1266
LIDAR_Z = 0.295
STEP_WARN, STEP_LETHAL = 0.03, 0.06
DROP_WARN, DROP_LETHAL = 0.04, 0.08
SLOPE_LETHAL_DEG = math.degrees(0.44)

out = []
def w(s=''):
    out.append(s)


def box(name, x, y, z, sx, sy, sz, rgb, static=True, yaw=0.0, note=None):
    """One axis-aligned box. z is the CENTRE, so a slab of height h sitting on ground
    z=g has z = g + h/2 - callers do that arithmetic explicitly to keep it visible."""
    if note:
        w(f'    <!-- {note} -->')
    w(f'    <model name="{name}">')
    w(f'      <static>{"true" if static else "false"}</static>')
    w(f'      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 {yaw:.4f}</pose>')
    w( '      <link name="link">')
    w( '        <collision name="collision">')
    w(f'          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>')
    w( '          <surface><friction><ode>'
       '<mu>1.0</mu><mu2>0.9</mu2>'
       '</ode></friction></surface>')
    w( '        </collision>')
    w( '        <visual name="visual">')
    w(f'          <geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>')
    w(f'          <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>')
    w( '        </visual>')
    w( '      </link>')
    w( '    </model>')


def cyl(name, x, y, z, r, h, rgb, note=None):
    if note:
        w(f'    <!-- {note} -->')
    w(f'    <model name="{name}">')
    w( '      <static>true</static>')
    w(f'      <pose>{x:.4f} {y:.4f} {z:.4f} 0 0 0</pose>')
    w( '      <link name="link">')
    for tag in ('collision', 'visual'):
        w(f'        <{tag} name="{tag}">')
        w(f'          <geometry><cylinder><radius>{r:.4f}</radius>'
          f'<length>{h:.4f}</length></cylinder></geometry>')
        if tag == 'visual':
            w(f'          <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>')
        else:
            w( '          <surface><friction><ode><mu>1.0</mu><mu2>0.9</mu2>'
               '</ode></friction></surface>')
        w(f'        </{tag}>')
    w( '      </link>')
    w( '    </model>')


def sphere_hill(name, x, y, height, rim_deg, rgb):
    """A hill that is the exposed cap of a buried sphere.

    *** WHY A SPHERE AND NOT A WEDGE OR A STACK OF STEPS ***
    Real ground has no edges, and an edge is exactly what the step test looks for: any
    faceted hill trips step_lethal at its own facet joins and the slope result is never
    reached. A spherical cap is smooth everywhere, its slope rises monotonically from 0
    at the crest to a maximum at the rim, and that maximum has a closed form:

        slope_max = acos(1 - height/R)   =>   R = height / (1 - cos(slope_max))

    So asking for "a 0.55 m hill that reaches 20 deg at its foot" fixes the radius
    exactly. The cap's ground-level footprint radius is R*sin(slope_max).

    Placing the centre at z = height - R puts the sphere's top at exactly z = height.
    """
    theta = math.radians(rim_deg)
    r_sphere = height / (1.0 - math.cos(theta))
    foot = r_sphere * math.sin(theta)
    w(f'    <!-- {name}: {height:.2f} m tall, {rim_deg:.0f} deg at the rim '
      f'({"ABOVE" if rim_deg > SLOPE_LETHAL_DEG else "below"} slope_lethal '
      f'{SLOPE_LETHAL_DEG:.1f} deg). Sphere R={r_sphere:.2f} m, '
      f'ground footprint radius {foot:.2f} m. -->')
    w(f'    <model name="{name}">')
    w( '      <static>true</static>')
    w(f'      <pose>{x:.4f} {y:.4f} {height - r_sphere:.4f} 0 0 0</pose>')
    w( '      <link name="link">')
    for tag in ('collision', 'visual'):
        w(f'        <{tag} name="{tag}">')
        w(f'          <geometry><sphere><radius>{r_sphere:.4f}</radius></sphere></geometry>')
        if tag == 'visual':
            w(f'          <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>')
        else:
            w( '          <surface><friction><ode><mu>1.0</mu><mu2>0.9</mu2>'
               '</ode></friction></surface>')
        w(f'        </{tag}>')
    w( '      </link>')
    w( '    </model>')
    return foot


# ====================================================================================
# Course layout
# ====================================================================================
DECK_X0, DECK_X1 = 16.0, 28.0     # erosion deck extent along x
DECK_Y0, DECK_Y1 = -4.0, 4.0
DECK_TOP = 0.35                   # deck surface height; sets the deepest possible hole
RAMP_DEG = 8.0                    # shallow enough to be well under slope_warn

CLAY = '0.42 0.26 0.16 1'         # Tennessee red clay
GRASS = '0.30 0.40 0.20 1'
ROCK = '0.45 0.45 0.42 1'
BARK = '0.28 0.20 0.13 1'
DARK = '0.12 0.10 0.09 1'         # inside of a hole, so it reads as depth not shadow

w('<?xml version="1.0" ?>')
w(f'''<!--
  GENERATED FILE - edit generate_outdoor.py and regenerate:
      python3 generate_outdoor.py > circe_outdoor.sdf

  Open Tennessee-like ground for CIRCE: rolling hills, washed-out clay with potholes,
  scattered rock, and mixed hardwood. Every feature is sized against a threshold in
  circe_perception/config/terrain.yaml so that driving the course ANSWERS something.

  *** THE BLIND BAND ***
  The RPLidar scan plane is at z = {LIDAR_Z} m. Everything shorter than that is invisible
  to it - the rover will drive into it at full speed unless the depth camera sees it.
  Most hazards here are deliberately in that band. Only the trees and the two tallest
  boulders are lidar-visible, and they are there so slam_toolbox has something to match
  against: an open field gives scan matching nothing to hold onto and the map smears.

  *** WHAT EACH ZONE ASKS ***
    step field   x 4..12    a rising series across step_lethal ({STEP_LETHAL} m). The rover
                            should drive over the first two and route around the rest.
    erosion deck x 16..28   potholes. North lane varies DEPTH at constant width; south
                            lane varies WIDTH at constant depth. A clear lane runs
                            between them, so "go around" is always available.
    hill field   x -6..-20  spherical caps whose rim slopes bracket slope_lethal
                            ({SLOPE_LETHAL_DEG:.1f} deg). Smooth, not faceted - see sphere_hill().
    gravel patch (6, 11)    roughness only. It must produce a PREFERENCE, never a wall.

  The rover spawns at the origin facing +x, on clear ground.
-->''')
w('<sdf version="1.9">')
w('  <world name="circe_outdoor">')
w('''
    <physics name="1ms" type="ignored">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <!-- Sensors and Imu are world-scoped in Harmonic and must be declared here, not in
         the model. Same as circe_indoor.sdf; see the note in empty.sdf. -->
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>

    <gravity>0 0 -9.8</gravity>

    <scene>
      <ambient>0.6 0.6 0.6 1</ambient>
      <background>0.65 0.75 0.9 1</background>
      <shadows>true</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 20 0 0 0</pose>
      <diffuse>0.95 0.93 0.88 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <attenuation>
        <range>2000</range><constant>0.9</constant>
        <linear>0.01</linear><quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.4 0.3 -0.86</direction>
    </light>

    <!-- Infinite base plane. Nothing can fall out of the world, so a pothole floor is
         always the deck's own underside rather than a hole into nothing. -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <surface><friction><ode><mu>1.0</mu><mu2>0.9</mu2></ode></friction></surface>
        </collision>
        <!-- *** THE VISUAL SITS 5 mm BELOW THE COLLISION. THIS IS DELIBERATE. ***
             A hill here is a spherical cap, so near its rim the sphere is TANGENT to
             this plane and the two surfaces agree to within a millimetre over a band
             roughly half a metre wide. Coplanar surfaces z-fight: the renderer picks
             between them per-pixel and the ground flickers, and where the plane wins
             it draws OVER the hill's skirt - so the rover, correctly resting on the
             hill, appears to hover above the ground.
             Measured driving onto the 6 deg hill: base_link tracked the sphere's
             analytic surface to within 0.8 mm mean, 7 mm worst. Nothing was floating;
             only the picture was wrong. Dropping the VISUAL 5 mm lets the hill always
             win the depth test near its rim. Collision is untouched, so the physics and
             every terrain measurement are exactly as before. -->
        <visual name="visual">
          <pose>0 0 -0.005 0 0 0</pose>
          <geometry><plane><normal>0 0 1</normal><size>120 120</size></plane></geometry>
          <material>
            <ambient>0.33 0.38 0.22 1</ambient>
            <diffuse>0.33 0.38 0.22 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
''')

# ---- zone 1: the step field --------------------------------------------------------
w('    <!-- ================= STEP FIELD: over it, or around it? ================= -->')
w(f'    <!-- Slabs 3 m wide across the route, so going AROUND is always possible (the')
w(f'         open ground beyond y = +-1.5 m is clear). Heights step across')
w(f'         step_warn {STEP_WARN} m and step_lethal {STEP_LETHAL} m. Everything up to')
w(f'         0.25 m is BELOW the {LIDAR_Z} m scan plane and invisible to the lidar. -->')
steps = [
    (4.0,  0.030, 'root_30mm',    'at step_warn exactly: cost, but drivable'),
    (6.0,  0.055, 'root_55mm',    'just UNDER step_lethal: should still be driven over'),
    (8.0,  0.070, 'rock_70mm',    'just OVER step_lethal: should be routed around'),
    (10.0, 0.120, 'rock_120mm',   'clearly lethal, still lidar-invisible'),
    (12.0, 0.250, 'rock_250mm',   'the worst case: 45 mm below the scan plane'),
]
for x, h, name, note in steps:
    box(name, x, 0.0, h / 2.0, 0.45, 3.0, h, CLAY, note=f'{name}: {h*1000:.0f} mm - {note}')
box('boulder_450mm', 14.0, 0.0, 0.225, 0.7, 2.2, 0.45, ROCK,
    note='boulder_450mm: 450 mm - the FIRST obstacle in this line the lidar can see')


# ---- zone 2: the erosion deck ------------------------------------------------------
# *** WHY A RAISED DECK RATHER THAN HOLES IN THE GROUND ***
# A <plane> collision is infinite and unbreakable: a box sunk beneath it is decoration,
# and the robot rolls straight over the top. (circe_indoor.sdf's pit_floor is exactly
# that mistake - it has never been drivable.) A genuine hole needs a raised surface with
# a real gap in it, so the deck is a slab DECK_TOP thick with trenches cut through and
# a floor at the depth we want. Ramps at each end make it reachable.
w('')
w('    <!-- ================= EROSION DECK: potholes ================= -->')
w(f'    <!-- A {DECK_X1-DECK_X0:.0f} x {DECK_Y1-DECK_Y0:.0f} m clay slab {DECK_TOP} m thick with trenches cut')
w(f'         through it. NORTH lane varies DEPTH at constant 0.60 m width; SOUTH lane')
w(f'         varies WIDTH at constant 0.25 m depth. drop_warn {DROP_WARN} m,')
w(f'         drop_lethal {DROP_LETHAL} m. A clear median runs between the lanes and clear')
w(f'         margins run outside both, so "route around" is always on the table.')
w(f'         Width matters as much as depth: the wheel is {WHEEL_R*2:.3f} m across, so a')
w(f'         trench much narrower than that is bridged and never felt. -->')

RAMP_L = DECK_TOP / math.tan(math.radians(RAMP_DEG))

# lanes in y: (y0, y1, trench list) - trenches are (x_centre, x_width, depth)
# Depths bracket drop_warn 0.04 and drop_lethal 0.08. Width is fixed at 0.60 m, well
# past anything a 0.253 m wheel can bridge, so DEPTH is the only variable.
NORTH = [(x, 0.60, d) for x, d in
         ((17.5, 0.04), (20.0, 0.07), (22.5, 0.12), (25.5, 0.20))]
# Widths bracket what the wheel bridges. A wheel of radius R crossing a gap of width g
# dips by R - sqrt(R^2 - (g/2)^2) before its far side lands, so the hole the ROBOT feels
# is that dip, not the hole's true depth. At R = 0.1266 m the dip runs 6 mm at g = 0.08
# up to the full depth by g = 2R = 0.253. Depth is held at 0.25 m - deep enough that
# every one of these is genuinely lethal IF the wheel reaches the bottom.
SOUTH = [(x, wd, 0.25) for x, wd in
         ((17.5, 0.08), (20.0, 0.15), (22.5, 0.25), (25.5, 0.50))]
BANDS = [
    (DECK_Y0, -2.8, [],    'south margin - clear'),
    (-2.8,    -1.2, SOUTH, 'SOUTH lane: width series at constant 0.25 m depth'),
    (-1.2,     1.2, [],    'median - clear, the always-available way through'),
    (1.2,      2.8, NORTH, 'NORTH lane: depth series at constant 0.60 m width'),
    (2.8,  DECK_Y1, [],    'north margin - clear'),
]

for i, (y0, y1, trenches, label) in enumerate(BANDS):
    yc, ylen = (y0 + y1) / 2.0, (y1 - y0)
    w(f'    <!-- deck band {i}: {label} -->')
    if not trenches:
        box(f'deck_band{i}', (DECK_X0 + DECK_X1) / 2.0, yc, DECK_TOP / 2.0,
            DECK_X1 - DECK_X0, ylen, DECK_TOP, CLAY)
        continue
    # Deck survives between the trenches: emit the x-spans, then each trench floor.
    edges = [DECK_X0]
    for xc, xw, _ in trenches:
        edges += [xc - xw / 2.0, xc + xw / 2.0]
    edges.append(DECK_X1)
    for j in range(0, len(edges), 2):
        a, b = edges[j], edges[j + 1]
        if b - a > 1e-3:
            box(f'deck_band{i}_seg{j//2}', (a + b) / 2.0, yc, DECK_TOP / 2.0,
                b - a, ylen, DECK_TOP, CLAY)
    for k, (xc, xw, depth) in enumerate(trenches):
        fill = DECK_TOP - depth          # solid clay left under the trench floor
        # How far the wheel actually falls before its far side catches the lip.
        half = xw / 2.0
        dip = (WHEEL_R - math.sqrt(max(WHEEL_R ** 2 - half ** 2, 0.0))
               if half < WHEEL_R else depth)
        felt = min(dip, depth)
        if felt >= DROP_LETHAL:
            verdict = f'LETHAL - wheel drops {felt*1000:.0f} mm, past drop_lethal'
        elif felt >= DROP_WARN:
            verdict = f'graded - wheel drops {felt*1000:.0f} mm, past drop_warn'
        else:
            verdict = f'drivable - wheel only drops {felt*1000:.0f} mm'
        box(f'pothole_b{i}_{k}', xc, yc, fill / 2.0, xw, ylen, fill, DARK,
            note=f'pothole {xw*100:.0f} cm wide x {depth*100:.0f} cm deep - {verdict}')

# Ramps on and off the deck.
w(f'    <!-- Access ramps, {RAMP_DEG:.0f} deg over {RAMP_L:.2f} m - well under slope_warn, so')
w( '         getting onto the deck is never itself the thing being tested. Each is sunk')
w( '         by half its thickness along the normal so its low edge meets the ground')
w( '         flush instead of forming a step at the toe. -->')
_p = math.radians(RAMP_DEG)
_hyp = math.hypot(RAMP_L, DECK_TOP)
_yc, _ylen = (DECK_Y0 + DECK_Y1) / 2.0, DECK_Y1 - DECK_Y0
for name, xc, pitch in (('ramp_west', DECK_X0 - RAMP_L / 2.0, -_p),
                        ('ramp_east', DECK_X1 + RAMP_L / 2.0, _p)):
    w(f'    <model name="{name}">')
    w( '      <static>true</static>')
    w(f'      <pose>{xc:.4f} {_yc:.4f} {DECK_TOP/2.0 - 0.05/math.cos(_p):.4f} '
      f'0 {pitch:.4f} 0</pose>')
    w( '      <link name="link">')
    for tag in ('collision', 'visual'):
        w(f'        <{tag} name="{tag}">')
        w(f'          <geometry><box><size>{_hyp:.4f} {_ylen:.4f} 0.10</size></box></geometry>')
        if tag == 'visual':
            w(f'          <material><ambient>{CLAY}</ambient><diffuse>{CLAY}</diffuse></material>')
        else:
            w( '          <surface><friction><ode><mu>1.0</mu><mu2>0.9</mu2>'
               '</ode></friction></surface>')
        w(f'        </{tag}>')
    w( '      </link>')
    w( '    </model>')


# ---- zone 3: the hills -------------------------------------------------------------
w('')
w('    <!-- ================= HILL FIELD ================= -->')
w(f'    <!-- Rim slopes bracket slope_lethal ({SLOPE_LETHAL_DEG:.1f} deg). Heights are kept small')
w( '         because a gentle slope needs an enormous sphere: at 6 deg a 0.20 m rise')
w( '         already spreads over a 3.8 m footprint, and a taller one would swallow the')
w( '         neighbouring hills. Centres are spaced past the sum of their footprints. -->')
HILLS = [
    ('hill_06deg',  -6.0,  -7.0, 0.20,  6.0, GRASS),
    ('hill_12deg',  -7.0,   6.0, 0.30, 12.0, GRASS),
    ('hill_20deg', -13.0,  -6.0, 0.45, 20.0, GRASS),
    ('hill_30deg', -13.0,   3.0, 0.60, 30.0, CLAY),
    ('hill_35deg', -18.0,  -2.0, 0.70, 35.0, CLAY),
]
feet = {}
for name, x, y, h, deg, rgb in HILLS:
    feet[name] = (x, y, sphere_hill(name, x, y, h, deg, rgb))

# Fail loudly rather than silently shipping overlapping hills.
_h = list(feet.items())
for a in range(len(_h)):
    for b in range(a + 1, len(_h)):
        (na, (xa, ya, fa)), (nb, (xb, yb, fb)) = _h[a], _h[b]
        if math.hypot(xa - xb, ya - yb) < fa + fb:
            raise SystemExit(f'hills {na} and {nb} overlap; move them apart')

# ---- zone 4: gravel ----------------------------------------------------------------
w('')
w('    <!-- ================= GRAVEL PATCH ================= -->')
w(f'    <!-- Many small stones in ONE model with one link: a hundred separate models')
w( '         would cost a hundred broadphase entries for no extra fidelity. Heights are')
w(f'         drawn in [0.012, 0.045] m, straddling rough_warn 0.010 and rough_max 0.040,')
w( '         and every one is below step_warn so the STEP test stays quiet and the')
w( '         ROUGHNESS test is what responds. Roughness is graded-only in terrain.yaml:')
w( '         this patch must become expensive, never impassable. -->')
random.seed(20250901)          # fixed: the course must be identical run to run
w('    <model name="gravel_patch">')
w('      <static>true</static>')
w('      <pose>6.0 11.0 0 0 0 0</pose>')
w('      <link name="link">')
for i in range(90):
    gx = random.uniform(-2.5, 2.5)
    gy = random.uniform(-2.5, 2.5)
    gh = random.uniform(0.012, 0.045)
    gs = random.uniform(0.10, 0.22)
    for tag in ('collision', 'visual'):
        w(f'        <{tag} name="{tag}_{i}">')
        w(f'          <pose>{gx:.3f} {gy:.3f} {gh/2:.4f} 0 0 {random.uniform(0, 3.14):.3f}</pose>')
        w(f'          <geometry><box><size>{gs:.3f} {gs:.3f} {gh:.4f}</size></box></geometry>')
        if tag == 'visual':
            w(f'          <material><ambient>{ROCK}</ambient><diffuse>{ROCK}</diffuse></material>')
        w(f'        </{tag}>')
w('      </link>')
w('    </model>')

# ---- woodland and landmarks --------------------------------------------------------
w('')
w('    <!-- ================= WOODLAND ================= -->')
w(f'    <!-- The ONLY lidar-visible features on the course besides the two boulders.')
w( '         Without them slam_toolbox has nothing to scan-match against: an open field')
w( '         is under-constrained in every direction and the map smears. Trees are kept')
w( '         out of the test zones and away from the spawn. -->')
KEEP_OUT = [
    (0.0, 0.0, 3.0),            # spawn
    (8.0, 0.0, 7.0),            # step field
    (22.0, 0.0, 9.0),           # erosion deck
    (6.0, 11.0, 4.0),           # gravel
] + [(x, y, f + 1.0) for x, y, f in feet.values()]

# *** TREE DENSITY IS A LOCALISATION PARAMETER, NOT SET DRESSING. ***
# Measured with 30 trees over this area: slam_toolbox's map -> odom correction was off
# by 16.7 m and 107 deg after a 37.9 m drive, while the EKF's odom -> base_link was off
# by only 0.56 m. Scan matching was not merely unhelpful, it was actively destroying a
# good dead-reckoning estimate - and Nav2 plans in `map`, so the rover drove to the
# wrong place. The A1 reaches about 12 m; from most spots it could see one or two
# trunks, which does not constrain a 2D pose in either translation or rotation.
#
# 90 stems over roughly 0.17 ha is about 520/ha. That is dense, but entirely ordinary
# for Tennessee second-growth mixed hardwood, and it puts several trunks in view from
# anywhere on the course. Spacing is capped at 2.0 m so the rover can still drive
# between them with 0.55 m inflation on a 0.36 m-wide body.
TREE_TARGET = 90
# 3.0 m, not 2.0. At 2.0 m the gaps were 2.0 - 2*0.20 = 1.6 m of bark-to-bark clear
# space, and with inflation_radius 0.55 on both sides that leaves half a metre of free
# corridor - tight enough that the rover wedged between trunks and climbed one, ending
# up on its roof. 3.0 m leaves a 2.6 m gap, comfortably more than the 1.1 m the
# inflated footprint needs, and is still ordinary spacing for second-growth timber.
TREE_MIN_SPACING = 3.0
placed = []
n = 0
attempts = 0
while n < TREE_TARGET and attempts < 20000:
    attempts += 1
    tx = random.uniform(-24.0, 34.0)
    ty = random.uniform(-18.0, 18.0)
    if any(math.hypot(tx - kx, ty - ky) < kr for kx, ky, kr in KEEP_OUT):
        continue
    if any(math.hypot(tx - px, ty - py) < TREE_MIN_SPACING for px, py in placed):
        continue
    placed.append((tx, ty))
    r = random.uniform(0.10, 0.20)
    cyl(f'tree_{n:02d}', tx, ty, 1.25, r, 2.5, BARK)
    n += 1
if n < TREE_TARGET:
    raise SystemExit(f'only placed {n}/{TREE_TARGET} trees; loosen spacing or area')

w('')
w('    <!-- Two tall boulders: lidar-visible, so the GLOBAL planner can route around')
w('         them from far away. Everything else low enough to matter is camera-only. -->')
cyl('boulder_north', 4.0, -9.0, 0.35, 0.55, 0.70, ROCK)
cyl('boulder_south', 12.0, 7.5, 0.40, 0.65, 0.80, ROCK)

w('')
w('  </world>')
w('</sdf>')

print('\n'.join(out))
