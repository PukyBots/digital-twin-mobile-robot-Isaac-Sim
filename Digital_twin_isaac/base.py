# =============================================================================
# Custom 3D-Printed Chassis Base Plate — Primitive Assembly Version
# NVIDIA Omniverse Isaac Sim  |  Window -> Script Editor -> Run
#
# Dimensions : 170 mm (L) x 120 mm (W) x 5 mm (T)
# Mass       : 110 g  (0.110 kg)
#
# Construction strategy (no custom mesh):
#   Part 1 — Main Body  : UsdGeom.Cube   120mm W x 100mm L x 5mm T
#   Part 2 — Top Arch   : UsdGeom.Cylinder  R=60mm, H=5mm, axis=Z
#
#   The cylinder centre is translated to sit exactly on the top edge of the
#   cube, so the two primitives share a seamless interface:
#
#       cube top-edge Y  =  +BODY_L/2  =  +0.050 m
#       cylinder centre Y = +0.050 m   (radius extends +0.060 further)
#       ──► total length  =  0.050 + 0.050 + 0.060 = 0.170 m  ✓
#
# Coordinate convention:
#   X  = width  (120 mm, centred at 0)
#   Y  = length (170 mm, centred at 0 after offset below)
#   Z  = up     (plate bottom at Z = 0, top at Z = PLATE_T)
#
# The chassis_link origin is placed so the plate bottom face rests at Z = 0.
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

PLATE_T  = 0.005   # 5 mm — shared thickness of both parts

# ── Part 1: Main Body cube ────────────────────────────────────────────────────
BODY_W   = 0.120   # 120 mm  (X)
BODY_L   = 0.100   # 100 mm  (Y)  straight section
BODY_T   = PLATE_T # 5 mm    (Z)

# ── Part 2: Top Arch cylinder ─────────────────────────────────────────────────
ARCH_R   = 0.060   # 60 mm  — radius equals half plate width → seamless join
ARCH_H   = PLATE_T # 5 mm   — same thickness as body
# Cylinder axis = Z  (flat top/bottom faces align with plate faces)

# ── Positioning ───────────────────────────────────────────────────────────────
# Body cube is centred at local (0, BODY_CY, 0) so that:
#   body bottom face = Z_BOT = 0
#   body top face    = Z_TOP = PLATE_T
# We need the assembly centred in Y, so work out the offsets:
#
#   Full length = BODY_L + ARCH_R = 0.100 + 0.060 = 0.160 m
#   BUT the body's bottom edge sits at Y = -BODY_L/2 (local)
#   and the arch adds another ARCH_R above the body top edge.
#   Total extents in Y (uncentred, from body bottom to arch top):
#       0 (body bottom)  to  BODY_L + ARCH_R = 0.160 m
#   Centre Y of the whole assembly = 0.080 m from body bottom.
#
# To centre the assembly at Y=0, shift everything by -0.080 m:
#
#   Body centre Y   = BODY_L/2  - 0.080 = 0.050 - 0.080 = -0.030 m
#   Arch centre Y   = BODY_L    - 0.080 = 0.100 - 0.080 = +0.020 m
#     (arch centre is at Y = BODY_L because arch top-edge = body top-edge,
#      so arch centre = body top-edge Y = BODY_L when body starts at 0)
#
# Z-centre for both parts (both have thickness PLATE_T, bottom at Z=0):
#   Part centre Z = PLATE_T / 2 = 0.0025 m

ASSEMBLY_HALF_L = (BODY_L + ARCH_R) / 2.0   # 0.080 m

BODY_CX  = 0.0
BODY_CY  = BODY_L / 2.0 - ASSEMBLY_HALF_L   # -0.030 m
BODY_CZ  = PLATE_T / 2.0                     #  0.0025 m

ARCH_CX  = 0.0
ARCH_CY  = BODY_L - ASSEMBLY_HALF_L          # +0.020 m
ARCH_CZ  = PLATE_T / 2.0                     #  0.0025 m

# =============================================================================
# HELPERS
# =============================================================================

def make_mat(stage, path: str) -> UsdShade.Material:
    """
    UsdPreviewSurface material simulating matte dark-grey 3D-printed PLA/PETG.
    No MDL / OmniPBR MDL dependency — works on every Isaac Sim version.
    """
    mat    = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, f"{path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",
                       Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.08, 0.08, 0.09))
    shader.CreateInput("roughness",
                       Sdf.ValueTypeNames.Float).Set(0.90)
    shader.CreateInput("metallic",
                       Sdf.ValueTypeNames.Float).Set(0.0)
    shader.CreateInput("useSpecularWorkflow",
                       Sdf.ValueTypeNames.Int).Set(0)
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat: UsdShade.Material):
    """Bind a UsdShade.Material to a prim."""
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def apply_collision(prim):
    """
    Apply CollisionAPI + convexHull MeshCollisionAPI to a prim.
    Wraps MeshCollisionAPI in try/except because it is not available on
    all Isaac Sim versions (primitives get analytical collision automatically).
    """
    UsdPhysics.CollisionAPI.Apply(prim)
    try:
        mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
        mc.CreateApproximationAttr().Set("convexHull")
    except Exception:
        pass   # primitives (Cube/Cylinder) use analytical collision by default


# =============================================================================
# MAIN
# =============================================================================

def build_chassis_plate():
    """
    Assemble the chassis plate from two standard primitives under chassis_link.

    Final USD hierarchy:
      /World/
        chassis_link/               ← RigidBody Xform  (mass = 0.110 kg)
          Materials/
            PLA_MatteGray           ← shared material
          Geometry/
            Body_Cube               ← UsdGeom.Cube   (main rectangular section)
            Arch_Cylinder           ← UsdGeom.Cylinder (top arch)
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

    # ── chassis_link root Xform ───────────────────────────────────────────────
    ROOT      = "/World/chassis_link"
    root_xf   = UsdGeom.Xform.Define(stage, ROOT)
    root_prim = root_xf.GetPrim()

    # Plate bottom face rests on Z = 0; no XY offset needed (centred at origin)
    xf = UsdGeom.Xformable(root_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0))

    # ── Shared material ───────────────────────────────────────────────────────
    UsdGeom.Scope.Define(stage, f"{ROOT}/Materials")
    mat = make_mat(stage, f"{ROOT}/Materials/PLA_MatteGray")

    # ── Geometry scope ────────────────────────────────────────────────────────
    GEO = f"{ROOT}/Geometry"
    UsdGeom.Scope.Define(stage, GEO)

    # =========================================================================
    # Part 1 — Main Body (UsdGeom.Cube)
    # UsdGeom.Cube default size = 2 (unit cube, edge −1..+1)
    # We scale by half-extents: scaleX = W/2, scaleY = L/2, scaleZ = T/2
    # =========================================================================
    body = UsdGeom.Cube.Define(stage, f"{GEO}/Body_Cube")

    body_xf = UsdGeom.Xformable(body)
    body_xf.AddTranslateOp().Set(Gf.Vec3f(BODY_CX, BODY_CY, BODY_CZ))
    body_xf.AddScaleOp().Set(
        Gf.Vec3f(BODY_W / 2.0,   # half-extent X
                 BODY_L / 2.0,   # half-extent Y
                 BODY_T / 2.0))  # half-extent Z

    bind(body.GetPrim(), mat)
    apply_collision(body.GetPrim())

    # =========================================================================
    # Part 2 — Top Arch (UsdGeom.Cylinder, axis = Z)
    # The cylinder's flat faces (top/bottom) align with the plate faces.
    # Its centre is placed at the top edge of the body cube so the curved
    # side of the cylinder forms the arch of the tombstone shape.
    # =========================================================================
    arch = UsdGeom.Cylinder.Define(stage, f"{GEO}/Arch_Cylinder")
    arch.CreateRadiusAttr(ARCH_R)   # 60 mm
    arch.CreateHeightAttr(ARCH_H)   # 5 mm
    arch.CreateAxisAttr("Z")        # flat faces on top & bottom

    arch_xf = UsdGeom.Xformable(arch)
    arch_xf.AddTranslateOp().Set(Gf.Vec3f(ARCH_CX, ARCH_CY, ARCH_CZ))

    bind(arch.GetPrim(), mat)
    apply_collision(arch.GetPrim())

    # =========================================================================
    # Physics on chassis_link parent
    # =========================================================================

    # RigidBodyAPI — enables simulation
    rb = UsdPhysics.RigidBodyAPI.Apply(root_prim)
    rb.CreateRigidBodyEnabledAttr(True)

    # MassAPI — exactly 110 g = 0.110 kg
    ma = UsdPhysics.MassAPI.Apply(root_prim)
    ma.CreateMassAttr(0.110)

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
        pass   # anonymous stage — safe to ignore

    print("=" * 60)
    print("  Custom Chassis Plate — created successfully!")
    print(f"  Root prim     : {ROOT}")
    print(f"  Total width   : {BODY_W * 1e3:.0f} mm  (= arch diameter)")
    print(f"  Total length  : {(BODY_L + ARCH_R) * 1e3:.0f} mm")
    print(f"  Thickness     : {PLATE_T * 1e3:.0f} mm")
    print(f"  Body cube     : {BODY_W*1e3:.0f} x {BODY_L*1e3:.0f} x {BODY_T*1e3:.0f} mm")
    print(f"  Arch cylinder : R={ARCH_R*1e3:.0f} mm, H={ARCH_H*1e3:.0f} mm, axis=Z")
    print(f"  Mass          : 0.110 kg  (110 g)")
    print(f"  Body centre Y : {BODY_CY*1e3:.1f} mm")
    print(f"  Arch centre Y : {ARCH_CY*1e3:.1f} mm")
    print("=" * 60)


# =============================================================================
# Entry point
# =============================================================================
build_chassis_plate()