# =============================================================================
# TT Motor Robot Wheel (65 mm Yellow) — 3D Model Generator
# NVIDIA Omniverse Isaac Sim  |  Window -> Script Editor -> Run
#
# Dimensions : OD 65 mm  |  Thickness 26 mm  |  Hub protrusion 12 mm
# Mass       : 18 g  (0.018 kg)
#
# Coordinate convention (parent-local, wheel lies flat):
#   Wheel rotational axis  →  Y  (axle runs along Y)
#   Wheel face faces       →  +Z  (hub protrudes toward +Y)
#   Centre of wheel body   →  origin (0, 0, 0)
#
# Assembly layers along Y (thickness axis):
#   −Y face (back)  ←──  rim cylinder  ──→  +Y face (front, hub side)
#   Tire ring wraps the outer rim on both sides.
#   Hub protrudes from the +Y face.
# =============================================================================

import omni.usd
import omni.kit.commands
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade,
    Sdf, Gf
)
import math

# =============================================================================
# CONSTANTS  (metres)
# =============================================================================

# ── Outer envelope ────────────────────────────────────────────────────────────
WHEEL_OD        = 0.065          # 65 mm outer diameter
WHEEL_R         = WHEEL_OD / 2.0 # 32.5 mm outer radius
WHEEL_THICKNESS = 0.026          # 26 mm total thickness  (Y axis)
HUB_PROTRUDE    = 0.012          # 12 mm hub protrusion beyond +Y face

# ── Tire (black rubber outer ring) ───────────────────────────────────────────
TIRE_WALL       = 0.008          # radial thickness of rubber band  (8 mm)
TIRE_R_OUT      = WHEEL_R        # matches outer diameter exactly
TIRE_R_IN       = WHEEL_R - TIRE_WALL   # inner edge of tire / outer edge of rim
TIRE_Y          = 0.0            # centred on Y
TIRE_H          = WHEEL_THICKNESS

# Tread blocks: small cubes proud of the tire surface, arranged radially
TREAD_COUNT     = 16             # number of tread blocks around circumference
TREAD_L         = 0.0055         # arc length  (tangential)
TREAD_W         = WHEEL_THICKNESS * 0.70   # runs most of the tire width
TREAD_H         = 0.0015         # radial height of block above tire surface
TREAD_R_CENTRE  = TIRE_R_OUT + TREAD_H / 2.0   # block centre radius

# ── Rim (yellow plastic inner cylinder) ──────────────────────────────────────
RIM_R           = TIRE_R_IN - 0.0005     # tiny gap between tire and rim
RIM_H           = WHEEL_THICKNESS
RIM_Y           = 0.0

# Spokes carved out of the rim face:
# We represent "weight-relief" pockets as dark inset cylinders recessed
# into the rim, evenly spaced radially.
SPOKE_POCKET_COUNT  = 5
SPOKE_POCKET_R      = 0.0065    # radius of each pocket
SPOKE_POCKET_RAD    = RIM_R * 0.55   # how far from centre the pocket sits
SPOKE_POCKET_DEPTH  = WHEEL_THICKNESS * 0.60  # depth of recess
SPOKE_POCKET_Y_FRONT =  RIM_H / 2.0 - SPOKE_POCKET_DEPTH / 2.0  # front face pockets
SPOKE_POCKET_Y_BACK  = -RIM_H / 2.0 + SPOKE_POCKET_DEPTH / 2.0  # back face pockets

# ── Axle bore (central through-hole for axle) ────────────────────────────────
AXLE_R          = 0.004          # 4 mm radius bore

# ── Centre hub (yellow protrusion from +Y face) ──────────────────────────────
HUB_R           = 0.0095         # 9.5 mm radius hub cylinder
HUB_H           = HUB_PROTRUDE   # 12 mm protrusion
HUB_Y           = RIM_H / 2.0 + HUB_H / 2.0   # centred on +Y side

# ── D-shaft hole in hub (the D-shaped motor mount) ───────────────────────────
# Modelled as a cylindrical bore + a thin slab that clips one side to make D
DSHAFT_R        = 0.003          # 3 mm radius bore
DSHAFT_H        = HUB_H + 0.001  # slightly deeper than hub for clean cut visual
DSHAFT_Y        = HUB_Y
# D-flat slab: yellow cube that blocks half the bore, implying the flat face
DFLAT_SIZE_X    = DSHAFT_R * 2.0 + 0.002
DFLAT_SIZE_Y    = HUB_H
DFLAT_SIZE_Z    = DSHAFT_R - 0.001   # thin slice covers only the flat portion
DFLAT_OFFSET_Z  = DSHAFT_R - DFLAT_SIZE_Z / 2.0  # shift to top of bore

# ── Collision proxy ───────────────────────────────────────────────────────────
# A single cylinder matching the full outer envelope — no per-tread collision
COLL_R          = WHEEL_R + TREAD_H   # include tread height in contact radius
COLL_H          = WHEEL_THICKNESS

# =============================================================================
# UTILITIES
# =============================================================================

def make_mat(stage, path: str, color: Gf.Vec3f,
             metallic: float = 0.0, roughness: float = 0.5) -> UsdShade.Material:
    """Create a UsdPreviewSurface material (MDL-free, works on all Isaac Sim versions)."""
    mat    = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",        Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("metallic",            Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness",           Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(0)
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat: UsdShade.Material):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def add_cylinder(stage, path: str,
                 radius: float, height: float,
                 pos: Gf.Vec3f, axis: str = "Y",
                 rot_xyz: Gf.Vec3f = None,
                 mat: UsdShade.Material = None) -> UsdGeom.Cylinder:
    """Create a UsdGeom.Cylinder with axis, position, optional rotation."""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    xf  = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(pos)
    if rot_xyz is not None:
        xf.AddRotateXYZOp().Set(rot_xyz)
    if mat:
        bind(cyl.GetPrim(), mat)
    return cyl


def add_cube(stage, path: str,
             size: Gf.Vec3f, pos: Gf.Vec3f,
             rot_xyz: Gf.Vec3f = None,
             mat: UsdShade.Material = None) -> UsdGeom.Cube:
    """Create a UsdGeom.Cube scaled to full extents (size = L×W×H)."""
    c  = UsdGeom.Cube.Define(stage, path)
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(pos)
    if rot_xyz is not None:
        xf.AddRotateXYZOp().Set(rot_xyz)
    xf.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))
    if mat:
        bind(c.GetPrim(), mat)
    return c


# =============================================================================
# MAIN BUILD FUNCTION
# =============================================================================

def build_tt_wheel():
    """
    Build the TT Motor Robot Wheel on the active Isaac Sim stage.

    Full USD hierarchy:
      /World/TT_RobotWheel                    ← RigidBody Xform (root)
        /Materials/...
        /Geometry/
          /Tire_Body                          ← black outer cylinder
          /Treads/Tread_NN                    ← 16 black tread blocks (cubes)
          /Rim_Body                           ← yellow inner cylinder
          /SpokePockets_Front/Pocket_NN       ← dark inset cylinders (5, front)
          /SpokePockets_Back/Pocket_NN        ← dark inset cylinders (5, back)
          /Axle_Bore                          ← dark through-hole cylinder
          /Hub/
            /Hub_Body                         ← yellow hub protrusion cylinder
            /DShaft_Bore                      ← dark D-shaft hole cylinder
            /DShaft_Flat                      ← yellow slab making the D flat
        /CollisionProxy                       ← invisible cylinder collider
    """

    # ── 1. Stage setup ────────────────────────────────────────────────────────
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # ── 2. Root prims ─────────────────────────────────────────────────────────
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")

    ROOT      = "/World/TT_RobotWheel"
    root_xf   = UsdGeom.Xform.Define(stage, ROOT)
    root_prim = root_xf.GetPrim()

    # Place wheel so its back face rests on Z = 0 plane,
    # wheel axle along Y, wheel rolling on Z-X plane.
    # Lift by outer radius so the tire bottom touches ground.
    xf = UsdGeom.Xformable(root_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, WHEEL_R + TREAD_H))

    # ── 3. Materials ──────────────────────────────────────────────────────────
    M = f"{ROOT}/Materials"
    UsdGeom.Scope.Define(stage, M)

    mat_rubber = make_mat(stage, f"{M}/Rubber_Black",
                          Gf.Vec3f(0.06, 0.06, 0.06),
                          metallic=0.0, roughness=0.95)   # matte black rubber

    mat_yellow = make_mat(stage, f"{M}/Plastic_Yellow",
                          Gf.Vec3f(0.95, 0.80, 0.04),
                          metallic=0.0, roughness=0.35)   # glossy bright yellow

    mat_dark   = make_mat(stage, f"{M}/Dark_Recess",
                          Gf.Vec3f(0.03, 0.03, 0.03),
                          metallic=0.0, roughness=1.0)    # near-black recesses

    # ── 4. Geometry scope ─────────────────────────────────────────────────────
    GEO = f"{ROOT}/Geometry"
    UsdGeom.Scope.Define(stage, GEO)

    # =========================================================================
    # 4a. Tire body — thick black outer cylinder (axis = Y)
    # =========================================================================
    add_cylinder(stage, f"{GEO}/Tire_Body",
                 radius=TIRE_R_OUT, height=TIRE_H,
                 pos=Gf.Vec3f(0.0, TIRE_Y, 0.0),
                 axis="Y", mat=mat_rubber)

    # =========================================================================
    # 4b. Tread blocks — 16 black cubes arranged radially on the tire surface
    #     Each cube is rotated so its long axis is tangential.
    # =========================================================================
    TREAD_SCOPE = f"{GEO}/Treads"
    UsdGeom.Scope.Define(stage, TREAD_SCOPE)

    for ti in range(TREAD_COUNT):
        angle_deg = ti * (360.0 / TREAD_COUNT)
        angle_rad = math.radians(angle_deg)

        # Centre of tread block in XZ plane (wheel axis = Y)
        tx = TREAD_R_CENTRE * math.sin(angle_rad)
        tz = TREAD_R_CENTRE * math.cos(angle_rad)

        # Rotate cube so its length aligns tangentially around the tire
        # The block's local X (length) must be tangent = perpendicular to radius
        rot_y = -angle_deg   # rotate around Y axis so block face is radial

        add_cube(stage, f"{TREAD_SCOPE}/Tread_{ti:02d}",
                 Gf.Vec3f(TREAD_L, TREAD_W, TREAD_H),
                 Gf.Vec3f(tx, 0.0, tz),
                 rot_xyz=Gf.Vec3f(0.0, rot_y, 0.0),
                 mat=mat_rubber)

    # =========================================================================
    # 4c. Rim body — yellow inner cylinder filling the inside of the tire
    # =========================================================================
    add_cylinder(stage, f"{GEO}/Rim_Body",
                 radius=RIM_R, height=RIM_H,
                 pos=Gf.Vec3f(0.0, RIM_Y, 0.0),
                 axis="Y", mat=mat_yellow)

    # =========================================================================
    # 4d. Spoke weight-relief pockets — dark cylinders recessed into both faces
    #     Evenly spaced radially, recessed from front AND back rim faces.
    # =========================================================================
    def add_pockets(scope_path: str, pocket_y: float):
        UsdGeom.Scope.Define(stage, scope_path)
        for pi in range(SPOKE_POCKET_COUNT):
            ang = math.radians(pi * 360.0 / SPOKE_POCKET_COUNT)
            px  = SPOKE_POCKET_RAD * math.sin(ang)
            pz  = SPOKE_POCKET_RAD * math.cos(ang)
            add_cylinder(stage, f"{scope_path}/Pocket_{pi:02d}",
                         radius=SPOKE_POCKET_R, height=SPOKE_POCKET_DEPTH,
                         pos=Gf.Vec3f(px, pocket_y, pz),
                         axis="Y", mat=mat_dark)

    add_pockets(f"{GEO}/SpokePockets_Front", SPOKE_POCKET_Y_FRONT)
    add_pockets(f"{GEO}/SpokePockets_Back",  SPOKE_POCKET_Y_BACK)

    # =========================================================================
    # 4e. Axle bore — dark cylinder running full Y through wheel centre
    # =========================================================================
    add_cylinder(stage, f"{GEO}/Axle_Bore",
                 radius=AXLE_R, height=RIM_H + 0.001,
                 pos=Gf.Vec3f(0.0, 0.0, 0.0),
                 axis="Y", mat=mat_dark)

    # =========================================================================
    # 4f. Hub assembly — yellow protrusion from +Y face
    # =========================================================================
    HUB_SCOPE = f"{GEO}/Hub"
    UsdGeom.Scope.Define(stage, HUB_SCOPE)

    # Hub cylinder body
    add_cylinder(stage, f"{HUB_SCOPE}/Hub_Body",
                 radius=HUB_R, height=HUB_H,
                 pos=Gf.Vec3f(0.0, HUB_Y, 0.0),
                 axis="Y", mat=mat_yellow)

    # D-shaft bore — dark cylinder through hub centre
    add_cylinder(stage, f"{HUB_SCOPE}/DShaft_Bore",
                 radius=DSHAFT_R, height=DSHAFT_H,
                 pos=Gf.Vec3f(0.0, DSHAFT_Y, 0.0),
                 axis="Y", mat=mat_dark)

    # D-shaft flat — thin yellow slab that masks one arc of the bore,
    # creating the visual D-profile (flat side at +Z of bore)
    add_cube(stage, f"{HUB_SCOPE}/DShaft_Flat",
             Gf.Vec3f(DFLAT_SIZE_X, DFLAT_SIZE_Y, DFLAT_SIZE_Z),
             Gf.Vec3f(0.0, DSHAFT_Y, DFLAT_OFFSET_Z),
             mat=mat_yellow)

    # =========================================================================
    # 5. Physics
    # =========================================================================

    # RigidBodyAPI on root prim
    rb = UsdPhysics.RigidBodyAPI.Apply(root_prim)
    rb.CreateRigidBodyEnabledAttr(True)

    # MassAPI — exactly 18 g = 0.018 kg
    ma = UsdPhysics.MassAPI.Apply(root_prim)
    ma.CreateMassAttr(0.018)

    # ── Collision proxy: single cylinder (NO per-tread mesh collision) ────────
    # Using a cylinder that exactly encloses the treaded tire outer surface.
    # This guarantees smooth rolling without jitter from tread geometry.
    PROXY = f"{ROOT}/CollisionProxy"
    proxy = add_cylinder(stage, PROXY,
                         radius=COLL_R, height=COLL_H,
                         pos=Gf.Vec3f(0.0, 0.0, 0.0),
                         axis="Y")
    pp = proxy.GetPrim()
    UsdGeom.Imageable(pp).MakeInvisible()

    # Apply CollisionAPI directly — cylinder is already convex,
    # so convexHull approximation is ideal for rolling contact.
    UsdPhysics.CollisionAPI.Apply(pp)
    mc = UsdPhysics.MeshCollisionAPI.Apply(pp)
    mc.CreateApproximationAttr("convexHull")

    # =========================================================================
    # 6. Viewport focus
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
    # 7. Save & report
    # =========================================================================
    stage.Save()

    print("=" * 64)
    print("  TT Motor Robot Wheel — model created successfully!")
    print(f"  Root prim       : {ROOT}")
    print(f"  Outer diameter  : {WHEEL_OD*1e3:.0f} mm")
    print(f"  Thickness       : {WHEEL_THICKNESS*1e3:.0f} mm")
    print(f"  Hub protrusion  : {HUB_PROTRUDE*1e3:.0f} mm  (along +Y)")
    print(f"  Mass            : 0.018 kg  (18 g)")
    print(f"  Tread blocks    : {TREAD_COUNT}")
    print(f"  Spoke pockets   : {SPOKE_POCKET_COUNT} × 2 faces")
    print(f"  Collision proxy : cylinder r={COLL_R*1e3:.1f} mm  (convexHull)")
    print(f"  Wheel axis      : Y  — attach RevoluteJoint along Y")
    print("=" * 64)


# =============================================================================
# Entry point
# =============================================================================
build_tt_wheel()