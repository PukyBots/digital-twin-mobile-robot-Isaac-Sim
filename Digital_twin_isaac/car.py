
import omni
import omni.usd
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade,
    Sdf, Gf, PhysxSchema
)
import math

# ──────────────────────────────────────────────────────────────
# HELPERS
# ──────────────────────────────────────────────────────────────

def get_stage():
    return omni.usd.get_context().get_stage()

def define_xform(stage, path):
    xform = UsdGeom.Xform.Define(stage, path)
    return xform

def set_translate(prim, x, y, z):
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    t_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
            t_op = op
            break
    if t_op is None:
        t_op = xform.AddTranslateOp()
    t_op.Set(Gf.Vec3d(x, y, z))

def set_rotate_xyz(prim, rx, ry, rz):
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    r_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
            r_op = op
            break
    if r_op is None:
        r_op = xform.AddRotateXYZOp()
    r_op.Set(Gf.Vec3f(rx, ry, rz))

def set_scale(prim, sx, sy, sz):
    xform = UsdGeom.Xformable(prim)
    ops = xform.GetOrderedXformOps()
    s_op = None
    for op in ops:
        if op.GetOpType() == UsdGeom.XformOp.TypeScale:
            s_op = op
            break
    if s_op is None:
        s_op = xform.AddScaleOp()
    s_op.Set(Gf.Vec3f(sx, sy, sz))

def add_rigid_body(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(prim)
    return rigid_api

def add_collision(stage, prim_path):
    prim = stage.GetPrimAtPath(prim_path)
    UsdPhysics.CollisionAPI.Apply(prim)

def add_mass(stage, prim_path, mass_kg):
    prim = stage.GetPrimAtPath(prim_path)
    mass_api = UsdPhysics.MassAPI.Apply(prim)
    mass_api.GetMassAttr().Set(mass_kg)

def set_display_color(geom_prim, r, g, b):
    geom_prim.GetDisplayColorAttr().Set([(r, g, b)])

def make_cube(stage, path, half_extents, color=(0.5, 0.5, 0.5)):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    set_scale(stage.GetPrimAtPath(path), half_extents[0]*2, half_extents[1]*2, half_extents[2]*2)
    set_display_color(cube, *color)
    return cube

def make_cylinder(stage, path, radius, height, color=(0.5, 0.5, 0.5)):
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(height)
    set_display_color(cyl, *color)
    return cyl

def make_sphere(stage, path, radius, color=(0.5, 0.5, 0.5)):
    sph = UsdGeom.Sphere.Define(stage, path)
    sph.GetRadiusAttr().Set(radius)
    set_display_color(sph, *color)
    return sph

def add_fixed_joint(stage, joint_path, body0_path, body1_path,
                    local_pos0=Gf.Vec3f(0,0,0), local_pos1=Gf.Vec3f(0,0,0)):
    joint = UsdPhysics.FixedJoint.Define(stage, joint_path)
    joint.GetBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.GetBody1Rel().SetTargets([Sdf.Path(body1_path)])
    joint.GetLocalPos0Attr().Set(local_pos0)
    joint.GetLocalPos1Attr().Set(local_pos1)
    return joint

def add_revolute_joint(stage, joint_path, body0_path, body1_path,
                       axis="Y",
                       local_pos0=Gf.Vec3f(0,0,0),
                       local_pos1=Gf.Vec3f(0,0,0),
                       local_rot0=Gf.Quatf(1,0,0,0),
                       local_rot1=Gf.Quatf(1,0,0,0)):
    joint = UsdPhysics.RevoluteJoint.Define(stage, joint_path)
    joint.GetAxisAttr().Set(axis)
    joint.GetBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.GetBody1Rel().SetTargets([Sdf.Path(body1_path)])
    joint.GetLocalPos0Attr().Set(local_pos0)
    joint.GetLocalPos1Attr().Set(local_pos1)
    joint.GetLocalRot0Attr().Set(local_rot0)
    joint.GetLocalRot1Attr().Set(local_rot1)
    # Enable angular drive for velocity control
    drive = UsdPhysics.DriveAPI.Apply(stage.GetPrimAtPath(joint_path), "angular")
    drive.GetTypeAttr().Set("velocity")
    drive.GetDampingAttr().Set(1e4)
    drive.GetStiffnessAttr().Set(0.0)
    drive.GetTargetVelocityAttr().Set(0.0)
    return joint

def add_spherical_joint(stage, joint_path, body0_path, body1_path,
                        local_pos0=Gf.Vec3f(0,0,0),
                        local_pos1=Gf.Vec3f(0,0,0)):
    joint = UsdPhysics.SphericalJoint.Define(stage, joint_path)
    joint.GetBody0Rel().SetTargets([Sdf.Path(body0_path)])
    joint.GetBody1Rel().SetTargets([Sdf.Path(body1_path)])
    joint.GetLocalPos0Attr().Set(local_pos0)
    joint.GetLocalPos1Attr().Set(local_pos1)
    joint.GetConeAngle0LimitAttr().Set(180)
    joint.GetConeAngle1LimitAttr().Set(180)
    return joint

# ──────────────────────────────────────────────────────────────
# CLEAR PREVIOUS ROBOT
# ──────────────────────────────────────────────────────────────

stage = get_stage()

robot_root_path = "/World/mobile_robot"
if stage.GetPrimAtPath(robot_root_path):
    stage.RemovePrim(robot_root_path)

# ──────────────────────────────────────────────────────────────
# PHYSICS SCENE
# ──────────────────────────────────────────────────────────────

scene_path = "/World/physicsScene"
if not stage.GetPrimAtPath(scene_path):
    scene = UsdPhysics.Scene.Define(stage, scene_path)
    scene.GetGravityDirectionAttr().Set(Gf.Vec3f(0, 0, -1))
    scene.GetGravityMagnitudeAttr().Set(9.81)

# Enable PhysX on scene
if stage.GetPrimAtPath(scene_path):
    physx_scene = PhysxSchema.PhysxSceneAPI.Apply(stage.GetPrimAtPath(scene_path))
    physx_scene.GetEnableCCDAttr().Set(True)

# ──────────────────────────────────────────────────────────────
# ROOT XFORM
# ──────────────────────────────────────────────────────────────

robot_xform = UsdGeom.Xform.Define(stage, robot_root_path)
robot_prim  = stage.GetPrimAtPath(robot_root_path)
set_translate(robot_prim, 0, 0, 0.12)   # Lift robot off ground

# Articulation root — lets Isaac Sim manage the whole robot as one unit
# and allows viewport selection / dragging when physics is paused
UsdPhysics.ArticulationRootAPI.Apply(robot_prim)
PhysxSchema.PhysxArticulationAPI.Apply(robot_prim)

# ──────────────────────────────────────────────────────────────
# DIMENSIONS  (all in meters, USD default unit = meters)
# ──────────────────────────────────────────────────────────────

# Chassis
bottom_w, bottom_d, bottom_h = 0.28, 0.20, 0.003   # x, y, z half ÷ 2
top_w,    top_d,    top_h    = 0.26, 0.18, 0.003
pillar_radius  = 0.005
pillar_height  = 0.08
chassis_gap    = pillar_height  # vertical space between plates

# Wheels
# Wheel near pillar0 (+0.12, +0.08)  →  front-right corner
# Wheel near pillar2 (+0.12, -0.08)  →  rear-right corner
# Caster between pillar1 and pillar3  →  left edge centre (-0.12, 0)
wheel_radius = 0.06
wheel_width  = 0.025
wheel_edge_x = bottom_w / 2 + wheel_width / 2 + 0.005   # outside right edge ~0.1475
wheel_y_front =  0.08   # near pillar0 / front-right
wheel_y_rear  = -0.08   # near pillar2 / rear-right

# Caster
caster_radius = 0.02
caster_side_x = -(bottom_w / 2 + caster_radius + 0.005)  # outside left edge

# Motor  (long axis along X, shaft exits at +X side toward wheel)
motor_l, motor_w, motor_h = 0.07, 0.03, 0.03
# Motor center placed DEEP inside chassis so its right face (x = motor_center_x + motor_l/2)
# stays well clear of the wheel's nearest X extent (wheel_edge_x - wheel_radius).
#   motor right face = 0.04 + 0.035 = 0.075
#   wheel nearest X  = 0.1475 + 0.0125 + 0.005 - 0.06 ≈ 0.105   --> 0.075 < 0.105 OK
motor_center_x = 0.04

# ──────────────────────────────────────────────────────────────
# GROUND PLANE (so the robot sits on something)
# ──────────────────────────────────────────────────────────────
gnd_path = "/World/GroundPlane"
if not stage.GetPrimAtPath(gnd_path):
    gnd = UsdGeom.Mesh.Define(stage, gnd_path)
    UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(gnd_path))
    # Simple large flat box
    gnd_box = UsdGeom.Cube.Define(stage, gnd_path + "/box")
    gnd_box.GetSizeAttr().Set(1.0)
    gnd_box_prim = stage.GetPrimAtPath(gnd_path + "/box")
    set_scale(gnd_box_prim, 10.0, 10.0, 0.01)
    set_translate(gnd_box_prim, 0, 0, -0.005)
    UsdPhysics.CollisionAPI.Apply(gnd_box_prim)
    gnd_box.GetDisplayColorAttr().Set([(0.3, 0.3, 0.3)])

# ──────────────────────────────────────────────────────────────
# BASE CHASSIS
# ──────────────────────────────────────────────────────────────

base_path = robot_root_path + "/base_chassis"
define_xform(stage, base_path)
base_prim = stage.GetPrimAtPath(base_path)
set_translate(base_prim, 0, 0, 0)

base_box_path = base_path + "/plate"
make_cube(stage, base_box_path,
          (bottom_w/2, bottom_d/2, bottom_h/2),
          color=(0.05, 0.05, 0.05))
base_box_prim = stage.GetPrimAtPath(base_box_path)
set_translate(base_box_prim, 0, 0, 0)

UsdPhysics.RigidBodyAPI.Apply(base_prim)
UsdPhysics.CollisionAPI.Apply(base_box_prim)
mass_api = UsdPhysics.MassAPI.Apply(base_prim)
mass_api.GetMassAttr().Set(0.8)

# ──────────────────────────────────────────────────────────────
# PILLARS (4 corner standoffs)
# ──────────────────────────────────────────────────────────────

pillar_positions = [
    ( bottom_w/2 - 0.02,  bottom_d/2 - 0.02),
    (-bottom_w/2 + 0.02,  bottom_d/2 - 0.02),
    ( bottom_w/2 - 0.02, -bottom_d/2 + 0.02),
    (-bottom_w/2 + 0.02, -bottom_d/2 + 0.02),
]

for i, (px, py) in enumerate(pillar_positions):
    pillar_path = base_path + f"/pillar_{i}"
    cyl_prim = make_cylinder(stage, pillar_path,
                             pillar_radius, pillar_height,
                             color=(0.8, 0.8, 0.8))
    p = stage.GetPrimAtPath(pillar_path)
    set_translate(p, px, py, (bottom_h + pillar_height) / 2 + bottom_h/2)
    UsdPhysics.CollisionAPI.Apply(p)

# ──────────────────────────────────────────────────────────────
# TOP CHASSIS
# ──────────────────────────────────────────────────────────────

top_path = robot_root_path + "/top_chassis"
define_xform(stage, top_path)
top_prim = stage.GetPrimAtPath(top_path)

top_z = bottom_h + pillar_height + top_h / 2
set_translate(top_prim, 0, 0, top_z)

top_box_path = top_path + "/plate"
make_cube(stage, top_box_path,
          (top_w/2, top_d/2, top_h/2),
          color=(0.05, 0.05, 0.05))
top_box_prim = stage.GetPrimAtPath(top_box_path)

UsdPhysics.RigidBodyAPI.Apply(top_prim)
UsdPhysics.CollisionAPI.Apply(top_box_prim)
top_mass = UsdPhysics.MassAPI.Apply(top_prim)
top_mass.GetMassAttr().Set(0.2)

# Fix top chassis to base chassis
add_fixed_joint(stage, robot_root_path + "/joints/top_to_base_joint",
                base_path, top_path,
                local_pos0=Gf.Vec3f(0, 0, top_z),
                local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# LEFT MOTOR
# ──────────────────────────────────────────────────────────────

# LEFT MOTOR — mounts near pillar0 (front-right corner, y=+0.08)
# Floats at wheel axle height (wheel_radius) so centre-line is co-axial with wheel.
lmotor_path = robot_root_path + "/left_motor"
define_xform(stage, lmotor_path)
lm_prim = stage.GetPrimAtPath(lmotor_path)
set_translate(lm_prim, motor_center_x, wheel_y_front, wheel_radius)

lm_body_path = lmotor_path + "/body"
make_cube(stage, lm_body_path,
          (motor_l/2, motor_w/2, motor_h/2),
          color=(0.9, 0.75, 0.0))   # Yellow TT motor
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(lm_body_path))
UsdPhysics.RigidBodyAPI.Apply(lm_prim)
lm_mass = UsdPhysics.MassAPI.Apply(lm_prim)
lm_mass.GetMassAttr().Set(0.3)

add_fixed_joint(stage, robot_root_path + "/joints/left_motor_joint",
                base_path, lmotor_path,
                local_pos0=Gf.Vec3f(motor_center_x, wheel_y_front, wheel_radius),
                local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# RIGHT MOTOR — mounts near pillar2 (rear-right corner, y=-0.08)
# ──────────────────────────────────────────────────────────────

# RIGHT MOTOR — mounts near pillar2 (rear-right corner, y=-0.08)
rmotor_path = robot_root_path + "/right_motor"
define_xform(stage, rmotor_path)
rm_prim = stage.GetPrimAtPath(rmotor_path)
set_translate(rm_prim, motor_center_x, wheel_y_rear, wheel_radius)

rm_body_path = rmotor_path + "/body"
make_cube(stage, rm_body_path,
          (motor_l/2, motor_w/2, motor_h/2),
          color=(0.9, 0.75, 0.0))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(rm_body_path))
UsdPhysics.RigidBodyAPI.Apply(rm_prim)
rm_mass = UsdPhysics.MassAPI.Apply(rm_prim)
rm_mass.GetMassAttr().Set(0.3)

add_fixed_joint(stage, robot_root_path + "/joints/right_motor_joint",
                base_path, rmotor_path,
                local_pos0=Gf.Vec3f(motor_center_x, wheel_y_rear, wheel_radius),
                local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# LEFT WHEEL
# ──────────────────────────────────────────────────────────────

# LEFT WHEEL — near pillar0 (front-right, y=+0.08), sticks out beyond right chassis edge
lwheel_path = robot_root_path + "/left_wheel"
define_xform(stage, lwheel_path)
lw_prim = stage.GetPrimAtPath(lwheel_path)
set_translate(lw_prim, wheel_edge_x, wheel_y_front, wheel_radius)

lw_geom_path = lwheel_path + "/geom"
make_cylinder(stage, lw_geom_path, wheel_radius, wheel_width, color=(0.15, 0.15, 0.15))
lw_geom = stage.GetPrimAtPath(lw_geom_path)
# Cylinder default axis = Z.  Rotate 90° around X → height aligns with Y.
# Wheel disc is in XZ plane (stands upright), axle = Y = revolute joint axis.
set_rotate_xyz(lw_geom, 90, 0, 0)

UsdPhysics.CollisionAPI.Apply(lw_geom)
UsdPhysics.RigidBodyAPI.Apply(lw_prim)
lw_mass = UsdPhysics.MassAPI.Apply(lw_prim)
lw_mass.GetMassAttr().Set(0.2)
PhysxSchema.PhysxRigidBodyAPI.Apply(lw_prim).GetDisableGravityAttr().Set(False)

# Revolute joint: spin axis = Y (robot rolls forward in X)
# local_pos0 = offset from motor centre to wheel centre in motor local frame
lw_joint_path = robot_root_path + "/joints/left_wheel_joint"
add_revolute_joint(stage, lw_joint_path,
                   lmotor_path, lwheel_path,
                   axis="Y",
                   local_pos0=Gf.Vec3f(wheel_edge_x - motor_center_x, 0, 0),
                   local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# RIGHT WHEEL — near pillar2 (rear-right, y=-0.08)
# ──────────────────────────────────────────────────────────────

rwheel_path = robot_root_path + "/right_wheel"
define_xform(stage, rwheel_path)
rw_prim = stage.GetPrimAtPath(rwheel_path)
set_translate(rw_prim, wheel_edge_x, wheel_y_rear, wheel_radius)

rw_geom_path = rwheel_path + "/geom"
make_cylinder(stage, rw_geom_path, wheel_radius, wheel_width, color=(0.15, 0.15, 0.15))
rw_geom = stage.GetPrimAtPath(rw_geom_path)
set_rotate_xyz(rw_geom, 90, 0, 0)

UsdPhysics.CollisionAPI.Apply(rw_geom)
UsdPhysics.RigidBodyAPI.Apply(rw_prim)
rw_mass = UsdPhysics.MassAPI.Apply(rw_prim)
rw_mass.GetMassAttr().Set(0.2)

rw_joint_path = robot_root_path + "/joints/right_wheel_joint"
add_revolute_joint(stage, rw_joint_path,
                   rmotor_path, rwheel_path,
                   axis="Y",
                   local_pos0=Gf.Vec3f(wheel_edge_x - motor_center_x, 0, 0),
                   local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# FRONT CASTER WHEEL
# ──────────────────────────────────────────────────────────────

# CASTER — between pillar1 (-0.12, +0.08) and pillar3 (-0.12, -0.08)
# Centre = (-0.12, 0) → left chassis edge
caster_path = robot_root_path + "/front_caster"
define_xform(stage, caster_path)
cast_prim = stage.GetPrimAtPath(caster_path)
set_translate(cast_prim, caster_side_x, 0.0, caster_radius)

cast_geom_path = caster_path + "/ball"
make_sphere(stage, cast_geom_path, caster_radius, color=(0.6, 0.6, 0.6))
cast_geom = stage.GetPrimAtPath(cast_geom_path)
UsdPhysics.CollisionAPI.Apply(cast_geom)
UsdPhysics.RigidBodyAPI.Apply(cast_prim)
cast_mass = UsdPhysics.MassAPI.Apply(cast_prim)
cast_mass.GetMassAttr().Set(0.05)

cast_joint_path = robot_root_path + "/joints/caster_joint"
add_spherical_joint(stage, cast_joint_path,
                    base_path, caster_path,
                    local_pos0=Gf.Vec3f(caster_side_x, 0.0, caster_radius),
                    local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# RASPBERRY PI BOARD
# ──────────────────────────────────────────────────────────────

rpi_path = robot_root_path + "/raspberry_pi"
define_xform(stage, rpi_path)
rpi_prim = stage.GetPrimAtPath(rpi_path)
set_translate(rpi_prim, -0.02, 0.01, top_z + top_h/2 + 0.0075 + 0.002)

rpi_board_path = rpi_path + "/board"
make_cube(stage, rpi_board_path, (0.085/2, 0.056/2, 0.0015/2), color=(0.0, 0.38, 0.0))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(rpi_board_path))

# USB ports
for i, ux in enumerate([-0.025, 0.005]):
    usb_path = rpi_path + f"/usb_{i}"
    make_cube(stage, usb_path, (0.015/2, 0.012/2, 0.006/2), color=(0.7, 0.7, 0.5))
    usb_prim = stage.GetPrimAtPath(usb_path)
    set_translate(usb_prim, ux, -0.056/2 - 0.006, 0.0015 + 0.003)
    UsdPhysics.CollisionAPI.Apply(usb_prim)

# Ethernet port
eth_path = rpi_path + "/eth"
make_cube(stage, eth_path, (0.016/2, 0.013/2, 0.013/2), color=(0.6, 0.6, 0.3))
eth_prim = stage.GetPrimAtPath(eth_path)
set_translate(eth_prim, 0.03, -0.056/2 - 0.007, 0.0015 + 0.007)
UsdPhysics.CollisionAPI.Apply(eth_prim)

UsdPhysics.RigidBodyAPI.Apply(rpi_prim)
rpi_mass = UsdPhysics.MassAPI.Apply(rpi_prim)
rpi_mass.GetMassAttr().Set(0.1)

add_fixed_joint(stage, robot_root_path + "/joints/rpi_joint",
                top_path, rpi_path,
                local_pos0=Gf.Vec3f(-0.02, 0.01, top_h/2 + 0.0075 + 0.002),
                local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# MOTOR DRIVER BOARD
# ──────────────────────────────────────────────────────────────

md_path = robot_root_path + "/motor_driver"
define_xform(stage, md_path)
md_prim = stage.GetPrimAtPath(md_path)
set_translate(md_prim, 0.07, 0.01, top_z + top_h/2 + 0.005 + 0.002)

md_board_path = md_path + "/board"
make_cube(stage, md_board_path, (0.06/2, 0.05/2, 0.01/2), color=(0.0, 0.0, 0.5))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(md_board_path))

UsdPhysics.RigidBodyAPI.Apply(md_prim)
md_mass = UsdPhysics.MassAPI.Apply(md_prim)
md_mass.GetMassAttr().Set(0.05)

add_fixed_joint(stage, robot_root_path + "/joints/md_joint",
                top_path, md_path,
                local_pos0=Gf.Vec3f(0.07, 0.01, top_h/2 + 0.005 + 0.002),
                local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# VOLTAGE REGULATOR MODULE
# ──────────────────────────────────────────────────────────────

vreg_path = robot_root_path + "/voltage_module"
define_xform(stage, vreg_path)
vreg_prim = stage.GetPrimAtPath(vreg_path)
set_translate(vreg_prim, -0.08, -0.04, top_z + top_h/2 + 0.01 + 0.002)

vreg_board_path = vreg_path + "/board"
make_cube(stage, vreg_board_path, (0.05/2, 0.03/2, 0.02/2), color=(0.1, 0.0, 0.5))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(vreg_board_path))

# Knob
knob_path = vreg_path + "/knob"
make_cylinder(stage, knob_path, 0.004, 0.008, color=(0.7, 0.7, 0.7))
knob_prim = stage.GetPrimAtPath(knob_path)
set_translate(knob_prim, 0, 0, 0.02/2 + 0.004)
UsdPhysics.CollisionAPI.Apply(knob_prim)

UsdPhysics.RigidBodyAPI.Apply(vreg_prim)
vreg_mass = UsdPhysics.MassAPI.Apply(vreg_prim)
vreg_mass.GetMassAttr().Set(0.03)

add_fixed_joint(stage, robot_root_path + "/joints/vreg_joint",
                top_path, vreg_path,
                local_pos0=Gf.Vec3f(-0.08, -0.04, top_h/2 + 0.01 + 0.002),
                local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# BATTERY CONNECTOR (XT60 style)
# ──────────────────────────────────────────────────────────────

batt_path = robot_root_path + "/battery"
define_xform(stage, batt_path)
batt_prim = stage.GetPrimAtPath(batt_path)
set_translate(batt_prim, 0.0, -0.06, bottom_h/2 + 0.015 + 0.001)

batt_box_path = batt_path + "/block"
make_cube(stage, batt_box_path, (0.04/2, 0.025/2, 0.03/2), color=(0.9, 0.5, 0.0))
UsdPhysics.CollisionAPI.Apply(stage.GetPrimAtPath(batt_box_path))

UsdPhysics.RigidBodyAPI.Apply(batt_prim)
batt_mass = UsdPhysics.MassAPI.Apply(batt_prim)
batt_mass.GetMassAttr().Set(0.15)

add_fixed_joint(stage, robot_root_path + "/joints/batt_joint",
                base_path, batt_path,
                local_pos0=Gf.Vec3f(0, -0.06, bottom_h/2 + 0.015 + 0.001),
                local_pos1=Gf.Vec3f(0, 0, 0))

# ──────────────────────────────────────────────────────────────
# WIRES  (thin cylinders connecting components)
# ──────────────────────────────────────────────────────────────

wires_path = robot_root_path + "/wires"
define_xform(stage, wires_path)

def add_wire(stage, parent_path, name, start, end,
             radius=0.0015, color=(0.05, 0.05, 0.05)):
    """Add a thin cylinder wire between two 3-D points."""
    sp = Gf.Vec3d(*start)
    ep = Gf.Vec3d(*end)
    mid   = (sp + ep) / 2.0
    delta = ep - sp
    length = delta.GetLength()
    if length < 1e-6:
        return

    wire_path = parent_path + f"/{name}"
    cyl = UsdGeom.Cylinder.Define(stage, wire_path)
    cyl.GetRadiusAttr().Set(radius)
    cyl.GetHeightAttr().Set(length)
    cyl.GetDisplayColorAttr().Set([color])

    # Align cylinder (default axis Z) toward delta direction
    cyl_prim = stage.GetPrimAtPath(wire_path)
    xf = UsdGeom.Xformable(cyl_prim)
    xf.ClearXformOpOrder()
    xf.AddTranslateOp().Set(mid)
    # Compute rotation from Z to delta
    z_axis = Gf.Vec3d(0, 0, 1)
    d_norm = delta.GetNormalized()
    cross  = Gf.Cross(z_axis, d_norm)
    dot    = Gf.Dot(z_axis, d_norm)
    angle  = math.degrees(math.acos(max(-1.0, min(1.0, dot))))
    if cross.GetLength() > 1e-6:
        cross = cross.GetNormalized()
        xf.AddRotateAxisAngleOp().Set(Gf.Vec3f(*cross), angle)

wire_z_base = bottom_h + 0.002
wire_z_top  = top_z + 0.01

# battery → motor driver
add_wire(stage, wires_path, "batt_to_md",
         (0.0,  -0.06, wire_z_base),
         (0.07,  0.01, wire_z_top), color=(1.0, 0.0, 0.0))

# motor driver → left motor (near pillar0, front-right)
add_wire(stage, wires_path, "md_to_lmotor",
         (0.07,  0.01, wire_z_top),
         (motor_edge_x, wheel_y_front, motor_h / 2),
         color=(0.0, 0.0, 0.9))

# motor driver → right motor (near pillar2, rear-right)
add_wire(stage, wires_path, "md_to_rmotor",
         (0.07,  0.01, wire_z_top),
         (motor_edge_x, wheel_y_rear, motor_h / 2),
         color=(0.0, 0.0, 0.9))

# raspberry pi → motor driver
add_wire(stage, wires_path, "rpi_to_md",
         (-0.02,  0.01, wire_z_top),
         ( 0.07,  0.01, wire_z_top),
         color=(0.8, 0.8, 0.0))

# ──────────────────────────────────────────────────────────────
# JOINT XFORM PRIM (ensure joints folder exists)
# ──────────────────────────────────────────────────────────────
joints_path = robot_root_path + "/joints"
if not stage.GetPrimAtPath(joints_path):
    UsdGeom.Xform.Define(stage, joints_path)

# ──────────────────────────────────────────────────────────────
# DIFFERENTIAL DRIVE VELOCITY CONTROL HELPER
# ──────────────────────────────────────────────────────────────

def set_wheel_velocities(left_vel_deg_s: float, right_vel_deg_s: float):
    """
    Control the two drive wheels via angular velocity drives.

    Parameters
    ----------
    left_vel_deg_s  : angular velocity for the left  wheel (deg/s)
    right_vel_deg_s : angular velocity for the right wheel (deg/s)

    Usage example (paste in Script Editor after building the robot):
        set_wheel_velocities(300, 300)   # forward
        set_wheel_velocities(-300, 300)  # turn left
        set_wheel_velocities(0, 0)       # stop
    """
    _stage = omni.usd.get_context().get_stage()
    for joint_path, vel in [
        (robot_root_path + "/joints/left_wheel_joint",  left_vel_deg_s),
        (robot_root_path + "/joints/right_wheel_joint", right_vel_deg_s),
    ]:
        j_prim = _stage.GetPrimAtPath(joint_path)
        if not j_prim.IsValid():
            print(f"[WARN] Joint not found: {joint_path}")
            continue
        drive = UsdPhysics.DriveAPI.Apply(j_prim, "angular")
        drive.GetTargetVelocityAttr().Set(vel)

# ──────────────────────────────────────────────────────────────
# FINALIZE
# ──────────────────────────────────────────────────────────────

print("=" * 60)
print("  Mobile Robot built successfully under /World/mobile_robot")
print("=" * 60)
print()
print("Hierarchy:")
print("  /World/mobile_robot")
print("    /base_chassis      — 28x20cm black acrylic bottom plate + 4 pillars")
print("    /top_chassis       — 26x18cm black acrylic top plate")
print("    /left_motor        — Yellow TT gear motor (left)")
print("    /right_motor       — Yellow TT gear motor (right)")
print("    /left_wheel        — Ø6cm rear drive wheel (revolute joint)")
print("    /right_wheel       — Ø6cm rear drive wheel (revolute joint)")
print("    /front_caster      — Ø2cm ball caster (spherical joint)")
print("    /raspberry_pi      — RPi board with USB + Ethernet")
print("    /motor_driver      — Motor driver board")
print("    /voltage_module    — Voltage regulator + knob")
print("    /battery           — XT60 battery connector")
print("    /wires             — Wires: battery→MD, MD→motors, RPi→MD")
print()
print("Differential drive velocity control helper:")
print("  set_wheel_velocities(left_deg_s, right_deg_s)")
print("  e.g.  set_wheel_velocities(300, 300)    # forward")
print("        set_wheel_velocities(-300, 300)   # spin left")
print("        set_wheel_velocities(0, 0)        # stop")
