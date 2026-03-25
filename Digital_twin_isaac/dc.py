# =============================================================================
# TT Yellow DC Gear Motor — 3D Model Generator for NVIDIA Omniverse Isaac Sim
# Usage  : Paste into Window -> Script Editor and click "Run"
# Dims   : 70 mm (L) × 22 mm (W) × 18 mm (H)   |   Mass: 28 g
#
# Coordinate convention (parent-local):
#   X  = length axis  (shaft points in ±X)
#   Y  = width axis
#   Z  = height axis  (up)
#
# Assembly layout along X:
#   −X end  ←  DC motor cylinder  |  Gearbox housing  →  +X end (shaft exit)
# =============================================================================

import omni.usd
import omni.kit.commands
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade,
    Sdf, Gf
)

# =============================================================================
# CONSTANTS  (all values in metres)
# =============================================================================

# --- Overall envelope ---
TOTAL_L = 0.070   # 70 mm — full assembly length along X
TOTAL_W = 0.022   # 22 mm — width along Y
TOTAL_H = 0.018   # 18 mm — height along Z

# --- Gearbox housing (yellow block) ---
GBOX_L  = 0.040   # 40 mm — occupies the +X portion
GBOX_W  = TOTAL_W
GBOX_H  = TOTAL_H
# Gearbox occupies X = [MOTOR_L, TOTAL_L] along the assembly
# We centre the assembly at X=0, so gearbox centre X:
GBOX_CX =  (TOTAL_L / 2.0) - (GBOX_L / 2.0)   # +0.015
GBOX_CY =  0.0
GBOX_CZ =  0.0                                   # centred in Z

# Recessed circular indentations on the top face of the gearbox (decorative)
INDENT_R  = 0.0025
INDENT_D  = 0.0005   # depth of recess
INDENT_Z  = GBOX_H / 2.0 - INDENT_D / 2.0
INDENT_POSITIONS = [   # (X, Y) in parent space
    (GBOX_CX - 0.008,  0.0),
    (GBOX_CX + 0.002,  0.0),
]

# Two through-holes (mounting) along Y axis in the yellow housing
MHOLE_R  = 0.0015
MHOLE_H  = GBOX_W + 0.0004   # slightly longer than housing width
MHOLE_POSITIONS_X = [   # X positions of the two mounting holes
    GBOX_CX - 0.010,
    GBOX_CX + 0.010,
]

# --- DC Motor cylinder (silver) ---
# Sits flush to −X end of gearbox; its right face touches gearbox left face
MOTOR_L  = TOTAL_L - GBOX_L          # 30 mm
MOTOR_R  = TOTAL_H / 2.0 * 0.85     # slightly smaller radius than gearbox half-height
MOTOR_CX = -(TOTAL_L / 2.0) + (MOTOR_L / 2.0)   # −0.020
MOTOR_CY = 0.0
MOTOR_CZ = 0.0

# --- Motor terminals (copper tabs protruding from rear face of motor) ---
TERM_L   = 0.0030   # depth along X
TERM_W   = 0.0040   # width along Y (two tabs side by side)
TERM_H   = 0.0015   # height along Z
TERM_SPACING_Y = 0.0060
TERM_CX  = MOTOR_CX - MOTOR_L / 2.0 - TERM_L / 2.0   # just behind motor rear
TERM_CZ  = 0.0

# --- Output D-shafts (white, dual, protrude from both sides in Y) ---
# D-shaft: cylinder with one flat face — approximated as cylinder + thin
# cube slice to represent the flat. Shaft axis along Y.
SHAFT_R       = 0.003     # 3 mm radius shaft
SHAFT_R_FLAT  = 0.002     # distance from centre to flat face (D cut)
SHAFT_PROTRUDE= 0.014     # how far each shaft protrudes beyond gearbox side (Y)
SHAFT_TOTAL_L = GBOX_W + 2.0 * SHAFT_PROTRUDE   # full shaft length in Y
SHAFT_CX      = GBOX_CX + GBOX_L / 2.0 - 0.006  # near front of gearbox
SHAFT_CY      = 0.0
SHAFT_CZ      = 0.0
# D-flat slab: sits on top of the shaft cylinder to "cut" one face flat
DFLAT_L       = SHAFT_TOTAL_L
DFLAT_W       = (SHAFT_R - SHAFT_R_FLAT) * 2.0   # thin slab width
DFLAT_H       = SHAFT_R * 2.0
DFLAT_OFFSET_Z = SHAFT_R - (SHAFT_R - SHAFT_R_FLAT)  # shift slab so it clips top

# --- Retaining band (thin silver strip wrapping motor + gearbox junction) ---
BAND_W     = 0.002    # 2 mm wide strip in X
BAND_R_OUT = MOTOR_R + 0.0008   # just outside motor radius
BAND_H     = BAND_R_OUT * 2.0
# Placed at the junction between motor and gearbox
BAND_CX    = GBOX_CX - GBOX_L / 2.0 + BAND_W / 2.0
BAND_CY    = 0.0
BAND_CZ    = 0.0

# =============================================================================
# UTILITY FUNCTIONS
# =============================================================================

def make_material(stage, path: str, color: Gf.Vec3f,
                  metallic: float = 0.0, roughness: float = 0.5) -> UsdShade.Material:
    """
    Create a UsdPreviewSurface material.
    Works across all Isaac Sim versions — no MDL file required.
    """
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
    """Bind a UsdShade.Material to a USD prim."""
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def cube(stage, path: str,
         size: Gf.Vec3f, pos: Gf.Vec3f,
         mat: UsdShade.Material = None,
         rot_xyz: Gf.Vec3f = None) -> UsdGeom.Cube:
    """
    Define a UsdGeom.Cube scaled to full extents (size = L × W × H).
    UsdGeom.Cube unit cube has edge = 2, so we scale by half-extents.
    """
    c  = UsdGeom.Cube.Define(stage, path)
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(pos)
    if rot_xyz is not None:
        xf.AddRotateXYZOp().Set(rot_xyz)
    xf.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))
    if mat:
        bind(c.GetPrim(), mat)
    return c


def cylinder(stage, path: str,
             radius: float, height: float,
             pos: Gf.Vec3f, axis: str = "Z",
             rot_xyz: Gf.Vec3f = None,
             mat: UsdShade.Material = None) -> UsdGeom.Cylinder:
    """
    Define a UsdGeom.Cylinder.
    axis    : "X", "Y", or "Z"  (default Z = vertical)
    rot_xyz : optional Euler XYZ rotation in degrees
    """
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


# =============================================================================
# MAIN BUILD FUNCTION
# =============================================================================

def build_tt_motor():
    """
    Construct the TT Yellow DC Gear Motor on the current Isaac Sim stage.
    Hierarchy:
      /World/TT_DCGearMotor               ← RigidBody root (Xform)
        /Materials/...
        /Geometry/
          /GearboxAssembly/               ← Xform grouping gearbox + shafts
            /Gearbox_Housing
            /Indentations/...
            /MountingHoles/...
            /OutputShaft_Left             ← Cylinder (−Y side) — joint-ready
            /OutputShaft_Right            ← Cylinder (+Y side) — joint-ready
            /DFlat_Left                   ← slab cutting the D-flat on left shaft
            /DFlat_Right
          /DCMotor_Body                   ← silver motor cylinder
          /Terminals/...
          /RetainingBand
        /CollisionProxy                   ← invisible bounding-box collider
    """

    # -------------------------------------------------------------------------
    # 1.  Stage
    # -------------------------------------------------------------------------
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # -------------------------------------------------------------------------
    # 2.  Root prims
    # -------------------------------------------------------------------------
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")

    ROOT = "/World/TT_DCGearMotor"
    root_xf   = UsdGeom.Xform.Define(stage, ROOT)
    root_prim = root_xf.GetPrim()

    # Lift assembly so its bottom face rests on the ground plane
    xf = UsdGeom.Xformable(root_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TOTAL_H / 2.0))

    # -------------------------------------------------------------------------
    # 3.  Materials
    # -------------------------------------------------------------------------
    M = f"{ROOT}/Materials"
    UsdGeom.Scope.Define(stage, M)

    mat_yellow = make_material(stage, f"{M}/Yellow_Gearbox",
                               Gf.Vec3f(0.95, 0.80, 0.05),
                               metallic=0.0, roughness=0.65)   # matte bright yellow

    mat_silver = make_material(stage, f"{M}/Silver_Metal",
                               Gf.Vec3f(0.78, 0.80, 0.84),
                               metallic=1.0, roughness=0.14)   # shiny silver

    mat_white  = make_material(stage, f"{M}/White_Shaft",
                               Gf.Vec3f(0.92, 0.92, 0.92),
                               metallic=0.0, roughness=0.55)   # matte white

    mat_copper = make_material(stage, f"{M}/Copper_Terminal",
                               Gf.Vec3f(0.72, 0.45, 0.20),
                               metallic=0.9, roughness=0.25)   # brass/copper

    mat_dark   = make_material(stage, f"{M}/Dark_Recess",
                               Gf.Vec3f(0.03, 0.03, 0.03),
                               metallic=0.0, roughness=1.0)    # near-black holes

    # -------------------------------------------------------------------------
    # 4.  Geometry scope
    # -------------------------------------------------------------------------
    GEO = f"{ROOT}/Geometry"
    UsdGeom.Scope.Define(stage, GEO)

    # =========================================================================
    # 4a.  GearboxAssembly  — yellow housing + D-shafts
    #      Kept as a named Xform so the shaft prims are easy to locate
    #      when setting up a Revolute Joint later.
    # =========================================================================
    GASM = f"{GEO}/GearboxAssembly"
    UsdGeom.Xform.Define(stage, GASM)   # identity transform — inherits root

    # Yellow housing block
    cube(stage, f"{GASM}/Gearbox_Housing",
         Gf.Vec3f(GBOX_L, GBOX_W, GBOX_H),
         Gf.Vec3f(GBOX_CX, GBOX_CY, GBOX_CZ),
         mat_yellow)

    # Recessed indentations on top face (dark shallow cylinders)
    IND = f"{GASM}/Indentations"
    UsdGeom.Scope.Define(stage, IND)
    for ii, (ix, iy) in enumerate(INDENT_POSITIONS):
        cylinder(stage, f"{IND}/Indent_{ii:02d}",
                 radius=INDENT_R, height=INDENT_D,
                 pos=Gf.Vec3f(ix, iy, INDENT_Z),
                 axis="Z", mat=mat_dark)

    # Mounting through-holes (dark cylinders running along Y)
    MH = f"{GASM}/MountingHoles"
    UsdGeom.Scope.Define(stage, MH)
    for mi, mx in enumerate(MHOLE_POSITIONS_X):
        cylinder(stage, f"{MH}/Hole_{mi:02d}",
                 radius=MHOLE_R, height=MHOLE_H,
                 pos=Gf.Vec3f(mx, 0.0, 0.0),
                 axis="Y", mat=mat_dark)

    # -----------------------------------------------------------------
    # Output D-shafts — each shaft is ONE clearly-named Cylinder prim
    # whose rotation axis aligns with Y.  A Revolute Joint can be
    # attached to /GearboxAssembly/OutputShaft_Left or _Right directly.
    # -----------------------------------------------------------------

    # Left shaft  (−Y protrusion) — full length spans the housing + protrusion
    cylinder(stage, f"{GASM}/OutputShaft_Left",
             radius=SHAFT_R, height=SHAFT_TOTAL_L,
             pos=Gf.Vec3f(SHAFT_CX, SHAFT_CY, SHAFT_CZ),
             axis="Y", mat=mat_white)

    # Right shaft prim — identical geometry, separate prim for joint mapping
    cylinder(stage, f"{GASM}/OutputShaft_Right",
             radius=SHAFT_R, height=SHAFT_TOTAL_L,
             pos=Gf.Vec3f(SHAFT_CX, SHAFT_CY, SHAFT_CZ),
             axis="Y", mat=mat_white)

    # D-flat slabs — thin cubes that "clip" the top of each shaft cylinder,
    # giving the visual impression of a D-profile.
    # Left flat (overrides top of left shaft with yellow housing colour
    #            to imply the cut — white shaft body still visible below)
    cube(stage, f"{GASM}/DFlat_Left",
         Gf.Vec3f(DFLAT_L, DFLAT_W, DFLAT_H),
         Gf.Vec3f(SHAFT_CX,
                  SHAFT_CY,
                  SHAFT_CZ + SHAFT_R - DFLAT_W / 2.0),
         mat_yellow)   # same yellow masks the top slice

    cube(stage, f"{GASM}/DFlat_Right",
         Gf.Vec3f(DFLAT_L, DFLAT_W, DFLAT_H),
         Gf.Vec3f(SHAFT_CX,
                  SHAFT_CY,
                  SHAFT_CZ + SHAFT_R - DFLAT_W / 2.0),
         mat_yellow)

    # =========================================================================
    # 4b.  DC Motor body — silver cylinder, axis along X
    # =========================================================================
    cylinder(stage, f"{GEO}/DCMotor_Body",
             radius=MOTOR_R, height=MOTOR_L,
             pos=Gf.Vec3f(MOTOR_CX, MOTOR_CY, MOTOR_CZ),
             axis="X", mat=mat_silver)

    # =========================================================================
    # 4c.  Motor terminals — two copper tabs at the very back (−X)
    # =========================================================================
    TERM = f"{GEO}/Terminals"
    UsdGeom.Scope.Define(stage, TERM)
    for ti, ty_offset in enumerate([-TERM_SPACING_Y / 2.0,
                                     TERM_SPACING_Y / 2.0]):
        cube(stage, f"{TERM}/Tab_{ti:02d}",
             Gf.Vec3f(TERM_L, TERM_W, TERM_H),
             Gf.Vec3f(TERM_CX, ty_offset, TERM_CZ),
             mat_copper)

    # =========================================================================
    # 4d.  Retaining band — thin cylinder (ring) at motor/gearbox junction
    #      Modelled as a short flat cylinder (disc) around the motor body.
    # =========================================================================
    cylinder(stage, f"{GEO}/RetainingBand",
             radius=BAND_R_OUT, height=BAND_W,
             pos=Gf.Vec3f(BAND_CX, BAND_CY, BAND_CZ),
             axis="X", mat=mat_silver)

    # =========================================================================
    # 5.  Physics
    # =========================================================================

    # RigidBodyAPI on root prim
    rb = UsdPhysics.RigidBodyAPI.Apply(root_prim)
    rb.CreateRigidBodyEnabledAttr(True)

    # MassAPI — exactly 28 g = 0.028 kg
    ma = UsdPhysics.MassAPI.Apply(root_prim)
    ma.CreateMassAttr(0.028)

    # Invisible bounding-box collision proxy
    PROXY = f"{ROOT}/CollisionProxy"
    proxy = cube(stage, PROXY,
                 Gf.Vec3f(TOTAL_L, TOTAL_W, TOTAL_H),
                 Gf.Vec3f(0.0, 0.0, 0.0))
    pp = proxy.GetPrim()
    UsdGeom.Imageable(pp).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(pp)
    mc = UsdPhysics.MeshCollisionAPI.Apply(pp)
    mc.CreateApproximationAttr("boundingCube")

    # =========================================================================
    # 6.  Viewport focus
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
    # 7.  Save & report
    # =========================================================================
    stage.Save()

    print("=" * 64)
    print("  TT Yellow DC Gear Motor — model created successfully!")
    print(f"  Root prim       : {ROOT}")
    print(f"  Assembly size   : {TOTAL_L*1e3:.0f} mm × "
          f"{TOTAL_W*1e3:.0f} mm × {TOTAL_H*1e3:.0f} mm")
    print(f"  Mass            : 0.028 kg  (28 g)")
    print(f"  Shaft prims     : {GASM}/OutputShaft_Left")
    print(f"                    {GASM}/OutputShaft_Right")
    print(f"  Shaft axis      : Y  (attach Revolute Joint along Y)")
    print("=" * 64)


# =============================================================================
# Entry point
# =============================================================================
build_tt_motor()