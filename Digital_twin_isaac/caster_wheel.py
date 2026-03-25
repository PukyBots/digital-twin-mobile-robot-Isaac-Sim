# =============================================================================
# Mini Ball Caster Wheel — 3D Model Generator
# NVIDIA Omniverse Isaac Sim  |  Window -> Script Editor -> Run
#
# Dimensions : Total height 20 mm  |  Housing width 15 mm  |  Ball Ø 12 mm
# Mass       : 7 g  (0.007 kg)
#
# Coordinate convention (parent-local):
#   X / Y  = horizontal plane
#   Z      = up
#
# Assembly layout (Z axis, bottom to top):
#   Z = 0            → ground contact point (bottom of ball)
#   Z = BALL_R       → ball centre           (6 mm)
#   Z = BALL_R * 1.5 → housing bottom face   (ball half-submerged)
#   Z = BALL_R * 1.5 + HOUSING_H → housing top face
#   Z = housing top  → flange bottom face
#   Z = housing top + FLANGE_T → assembly top
#
# The caster_link origin is placed so the ball bottom touches Z = 0.
# =============================================================================

import omni.usd
import omni.kit.commands
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade,
    Sdf, Gf
)

# =============================================================================
# DIMENSIONS  (metres)
# =============================================================================

BALL_D          = 0.012    # 12 mm  ball diameter
BALL_R          = BALL_D / 2.0   # 6 mm  ball radius

HOUSING_D       = 0.015    # 15 mm  housing outer diameter
HOUSING_R       = HOUSING_D / 2.0
TOTAL_H         = 0.020    # 20 mm  total assembly height

# Ball centre sits at Z = BALL_R (ball bottom at Z = 0)
BALL_CZ         = BALL_R   # 0.006 m

# Housing bottom is at the ball equator (ball half-submerged)
HOUSING_BOT_Z   = BALL_CZ + BALL_R * 0.5   # 0.009 m

# Remaining height above housing bottom
FLANGE_T        = 0.002    # 2 mm  mounting flange thickness
HOUSING_H       = TOTAL_H - HOUSING_BOT_Z - FLANGE_T   # ~0.009 m
HOUSING_CZ      = HOUSING_BOT_Z + HOUSING_H / 2.0

# Flange (rectangular plate on top of housing)
FLANGE_L        = 0.022    # 22 mm  length (slightly wider than housing)
FLANGE_W        = 0.018    # 18 mm  width
FLANGE_BOT_Z    = HOUSING_BOT_Z + HOUSING_H
FLANGE_CZ       = FLANGE_BOT_Z + FLANGE_T / 2.0

# Mounting holes in the flange
MHOLE_R         = 0.0010   # 1 mm radius  (Ø 2 mm holes)
MHOLE_H         = FLANGE_T + 0.0004  # slightly taller than flange
MHOLE_OFFSET_X  = 0.007    # ± 7 mm from centre
MHOLE_CZ        = FLANGE_CZ

# =============================================================================
# HELPERS
# =============================================================================

def make_mat(stage, path: str, color: Gf.Vec3f,
             metallic: float, roughness: float) -> UsdShade.Material:
    """
    Create a UsdPreviewSurface material.
    Works on all Isaac Sim versions — no MDL file dependency.
    """
    mat    = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",
                       Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("metallic",
                       Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness",
                       Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("useSpecularWorkflow",
                       Sdf.ValueTypeNames.Int).Set(0)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat: UsdShade.Material):
    """Bind a UsdShade.Material to a USD prim."""
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def add_cylinder(stage, path: str,
                 radius: float, height: float,
                 cz: float, axis: str = "Z",
                 mat: UsdShade.Material = None) -> UsdGeom.Cylinder:
    """Define a vertical UsdGeom.Cylinder centred at (0, 0, cz)."""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    xf  = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, cz))
    if mat:
        bind(cyl.GetPrim(), mat)
    return cyl


def add_cube(stage, path: str,
             sx: float, sy: float, sz: float,
             cx: float, cy: float, cz: float,
             mat: UsdShade.Material = None) -> UsdGeom.Cube:
    """
    Define a UsdGeom.Cube with full extents sx × sy × sz,
    centred at (cx, cy, cz).
    UsdGeom.Cube default edge = 2, so scale by half-extents.
    """
    cube = UsdGeom.Cube.Define(stage, path)
    xf   = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3f(cx, cy, cz))
    xf.AddScaleOp().Set(Gf.Vec3f(sx / 2.0, sy / 2.0, sz / 2.0))
    if mat:
        bind(cube.GetPrim(), mat)
    return cube


# =============================================================================
# MAIN BUILD FUNCTION
# =============================================================================

def build_ball_caster():
    """
    Construct the Mini Ball Caster Wheel on the active Isaac Sim stage.

    USD hierarchy:
      /World/
        caster_link/                    ← RigidBody Xform (mass = 0.007 kg)
          Materials/
            Chrome_Ball                 ← polished chrome sphere material
            Housing_White               ← matte white housing material
            Dark_Hole                   ← near-black mounting hole material
          Geometry/
            Ball_Sphere                 ← UsdGeom.Sphere  (chrome, frictionless)
            Housing_Cylinder            ← UsdGeom.Cylinder (white)
            Flange_Cube                 ← UsdGeom.Cube    (white mounting plate)
            MountingHoles/
              Hole_Left                 ← UsdGeom.Cylinder (dark visual only)
              Hole_Right
    """

    # ── Stage ─────────────────────────────────────────────────────────────────
    ctx   = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None:
        ctx.new_stage()
        stage = ctx.get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # ── /World ────────────────────────────────────────────────────────────────
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")

    # ── caster_link root ──────────────────────────────────────────────────────
    ROOT      = "/World/caster_link"
    root_xf   = UsdGeom.Xform.Define(stage, ROOT)
    root_prim = root_xf.GetPrim()

    # Ball bottom at Z=0 → translate root up by BALL_R so ball grazes the ground
    # (caster_link origin = ball bottom contact point = ground level)
    xf = UsdGeom.Xformable(root_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

    # ── Materials ─────────────────────────────────────────────────────────────
    M = f"{ROOT}/Materials"
    UsdGeom.Scope.Define(stage, M)

    # Polished chrome ball — metallic=1, roughness=0 for mirror finish
    mat_chrome = make_mat(stage, f"{M}/Chrome_Ball",
                          Gf.Vec3f(0.92, 0.92, 0.95),
                          metallic=1.0, roughness=0.0)

    # Matte white housing — low metallic, high roughness
    mat_white  = make_mat(stage, f"{M}/Housing_White",
                          Gf.Vec3f(0.88, 0.88, 0.90),
                          metallic=0.1, roughness=0.75)

    # Near-black for mounting holes (visual only)
    mat_dark   = make_mat(stage, f"{M}/Dark_Hole",
                          Gf.Vec3f(0.03, 0.03, 0.03),
                          metallic=0.0, roughness=1.0)

    # ── Geometry scope ────────────────────────────────────────────────────────
    GEO = f"{ROOT}/Geometry"
    UsdGeom.Scope.Define(stage, GEO)

    # =========================================================================
    # 1. Caster Ball — UsdGeom.Sphere
    #    Centre at Z = BALL_R so bottom of ball sits at Z = 0
    # =========================================================================
    ball = UsdGeom.Sphere.Define(stage, f"{GEO}/Ball_Sphere")
    ball.CreateRadiusAttr(BALL_R)
    ball_xf = UsdGeom.Xformable(ball)
    ball_xf.AddTranslateOp().Set(Gf.Vec3f(0.0, 0.0, BALL_CZ))
    bind(ball.GetPrim(), mat_chrome)

    # ── Physics friction material on the ball ─────────────────────────────────
    # A frictionless physics material lets the ball roll smoothly in simulation
    # without generating lateral forces that cause jitter.
    ball_prim = ball.GetPrim()

    # Create a dedicated physics material prim for the ball
    PHYS_MAT_PATH = f"{ROOT}/BallPhysicsMaterial"
    phys_mat_prim = stage.DefinePrim(PHYS_MAT_PATH, "Material")

    # Apply UsdPhysics.MaterialAPI and set zero friction
    phys_mat_api = UsdPhysics.MaterialAPI.Apply(phys_mat_prim)
    phys_mat_api.CreateStaticFrictionAttr(0.0)
    phys_mat_api.CreateDynamicFrictionAttr(0.0)
    phys_mat_api.CreateRestitutionAttr(0.1)   # slight bounce is realistic

    # Bind the physics material to the ball prim
    # UsdShade.MaterialBindingAPI with the "physics" purpose
    binding_api = UsdShade.MaterialBindingAPI(ball_prim)
    phys_mat_shade = UsdShade.Material(phys_mat_prim)
    binding_api.Bind(phys_mat_shade,
                     UsdShade.Tokens.weakerThanDescendants,
                     "physics")

    # Collision on the ball (sphere — analytically perfect for rolling)
    UsdPhysics.CollisionAPI.Apply(ball_prim)

    # =========================================================================
    # 2. Housing Cylinder
    #    Runs from HOUSING_BOT_Z to HOUSING_BOT_Z + HOUSING_H
    #    with a slight taper-look by using a standard cylinder
    # =========================================================================
    add_cylinder(stage, f"{GEO}/Housing_Cylinder",
                 radius=HOUSING_R, height=HOUSING_H,
                 cz=HOUSING_CZ, axis="Z",
                 mat=mat_white)

    housing_prim = stage.GetPrimAtPath(f"{GEO}/Housing_Cylinder")
    UsdPhysics.CollisionAPI.Apply(housing_prim)

    # =========================================================================
    # 3. Mounting Flange — flat rectangular plate on top of housing
    # =========================================================================
    add_cube(stage, f"{GEO}/Flange_Cube",
             FLANGE_L, FLANGE_W, FLANGE_T,
             0.0, 0.0, FLANGE_CZ,
             mat_white)

    flange_prim = stage.GetPrimAtPath(f"{GEO}/Flange_Cube")
    UsdPhysics.CollisionAPI.Apply(flange_prim)

    # =========================================================================
    # 4. Mounting Holes — two dark cylinders recessed into the flange
    #    (visual only — no physics; analytically the flange collision is solid)
    # =========================================================================
    HOLES = f"{GEO}/MountingHoles"
    UsdGeom.Scope.Define(stage, HOLES)

    for name, ox in [("Hole_Left", -MHOLE_OFFSET_X),
                     ("Hole_Right", +MHOLE_OFFSET_X)]:
        hole = UsdGeom.Cylinder.Define(stage, f"{HOLES}/{name}")
        hole.CreateRadiusAttr(MHOLE_R)
        hole.CreateHeightAttr(MHOLE_H)
        hole.CreateAxisAttr("Z")
        hxf = UsdGeom.Xformable(hole)
        hxf.AddTranslateOp().Set(Gf.Vec3f(ox, 0.0, MHOLE_CZ))
        bind(hole.GetPrim(), mat_dark)

    # =========================================================================
    # Physics on caster_link root
    # =========================================================================

    # RigidBodyAPI — makes the whole assembly a single rigid body
    rb = UsdPhysics.RigidBodyAPI.Apply(root_prim)
    rb.CreateRigidBodyEnabledAttr(True)

    # MassAPI — exactly 7 g = 0.007 kg
    ma = UsdPhysics.MassAPI.Apply(root_prim)
    ma.CreateMassAttr(0.007)

    # =========================================================================
    # Viewport focus
    # =========================================================================
    try:
        import omni.kit.viewport.utility as vpu
        vp = vpu.get_active_viewport()
        if vp:
            omni.kit.commands.execute(
                "FramePrimsCommand",
                prim_to_frame=ROOT,
                viewport_api=vp
            )
    except Exception:
        pass

    # =========================================================================
    # Save & report
    # =========================================================================
    try:
        stage.Save()
    except Exception:
        pass

    print("=" * 58)
    print("  Mini Ball Caster Wheel — created successfully!")
    print(f"  Root prim       : {ROOT}")
    print(f"  Ball Ø          : {BALL_D*1e3:.0f} mm  (R={BALL_R*1e3:.0f} mm)")
    print(f"  Ball centre Z   : {BALL_CZ*1e3:.1f} mm  (bottom at Z=0)")
    print(f"  Housing Ø       : {HOUSING_D*1e3:.0f} mm")
    print(f"  Housing height  : {HOUSING_H*1e3:.2f} mm")
    print(f"  Flange          : {FLANGE_L*1e3:.0f} x {FLANGE_W*1e3:.0f} x {FLANGE_T*1e3:.0f} mm")
    print(f"  Total height    : {TOTAL_H*1e3:.0f} mm")
    print(f"  Mass            : 0.007 kg  (7 g)")
    print(f"  Ball friction   : static=0.0  dynamic=0.0  (frictionless)")
    print("=" * 58)


# =============================================================================
# Entry point
# =============================================================================
build_ball_caster()