# =============================================================================
# Li-Po 3S Battery Pack — Hyper-Realistic Digital Twin Generator
# NVIDIA Omniverse Isaac Sim  |  Window -> Script Editor -> Run
#
# Dimensions : 90 mm (L) × 45 mm (W) × 16 mm (H)
# Mass       : 180 g  (0.180 kg)
#
# Coordinate convention (parent-local):
#   X  = length axis   (90 mm, wires exit from +X end)
#   Y  = width axis    (45 mm)
#   Z  = height / up   (16 mm, label faces +Z)
#
# Assembly layout:
#   battery_link (root Xform, RigidBody)
#     ├─ Materials/         ← all PBR material prims
#     ├─ Body/
#     │    ├─ Battery_Body  ← main matte-black block
#     │    └─ Label_Surface ← thin slab on top face with label UVs
#     ├─ Wires/
#     │    ├─ MainHarness/
#     │    │    ├─ PowerCap       ← red/black tape cap block
#     │    │    ├─ Wire_Red_NN    ← red silicone cylinders (segmented arch)
#     │    │    ├─ Wire_Black_NN  ← black silicone cylinders
#     │    │    └─ XT60_Connector ← yellow XT60 proxy block
#     │    └─ BalanceLead/
#     │         ├─ BalanceCap     ← bundled wire cap
#     │         ├─ BalWire_[col]  ← 4 coloured thin wires
#     │         └─ JST_Connector  ← white JST-XH proxy block
#     └─ CollisionProxy    ← invisible bounding-box collider (body only)
# =============================================================================

import math
import omni.usd
import omni.kit.commands
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade,
    Sdf, Gf, Vt
)

# =============================================================================
# DIMENSIONS  (metres)
# =============================================================================

# ── Battery body ──────────────────────────────────────────────────────────────
BATT_L   = 0.090   # 90 mm  X
BATT_W   = 0.045   # 45 mm  Y
BATT_H   = 0.016   # 16 mm  Z

# Body centre sits at origin; bottom face at Z = -BATT_H/2
# Label top face at Z = +BATT_H/2

# ── Label slab (thin plane on top of battery) ─────────────────────────────────
LABEL_T  = 0.0002   # 0.2 mm — just proud of the top face
LABEL_L  = BATT_L * 0.82   # slightly inset from ends
LABEL_W  = BATT_W * 0.86   # slightly inset from sides
LABEL_CZ = BATT_H / 2.0 + LABEL_T / 2.0

# ── Wire harness exit point (from +X end face, vertically centred) ────────────
WIRE_EXIT_X  = BATT_L / 2.0
WIRE_EXIT_Z  = 0.0

# Main power leads
WIRE_R_THICK = 0.0025   # 2.5 mm radius (thick power wire)
WIRE_SEGS    = 8        # cylinder segments per wire (arch approximation)
WIRE_LEN     = 0.100    # 100 mm total wire length from exit to connector
WIRE_ARCH_H  = 0.012    # arch peak above exit Z

# XT60 connector dimensions
XT60_L   = 0.018
XT60_W   = 0.012
XT60_H   = 0.014
# XT60 sits at X = WIRE_EXIT_X + WIRE_LEN, slightly above battery centre
XT60_X   = WIRE_EXIT_X + WIRE_LEN
XT60_Z   = WIRE_EXIT_Z + WIRE_ARCH_H * 0.5

# Balance lead (thinner, 4 wires)
BAL_R    = 0.0008    # 0.8 mm radius thin balance wire
BAL_LEN  = 0.060     # 60 mm
BAL_OFFSET_Y = -BATT_W * 0.25   # offset toward −Y to separate from main harness
BAL_SEGS = 5

# JST-XH connector
JST_L    = 0.010
JST_W    = 0.008
JST_H    = 0.006
JST_X    = WIRE_EXIT_X + BAL_LEN
JST_Z    = WIRE_EXIT_Z - 0.003

# Tape/cap block at wire exit
CAP_L    = 0.006
CAP_W    = BATT_W * 0.35
CAP_H    = 0.010
CAP_CX   = WIRE_EXIT_X + CAP_L / 2.0

# Balance cap (smaller)
BCAP_L   = 0.004
BCAP_W   = 0.010
BCAP_H   = 0.006
BCAP_CX  = WIRE_EXIT_X + BCAP_L / 2.0
BCAP_CY  = BAL_OFFSET_Y

# =============================================================================
# COLOUR PALETTE
# =============================================================================

C_BLACK       = Gf.Vec3f(0.04, 0.04, 0.05)
C_RED         = Gf.Vec3f(0.85, 0.04, 0.04)
C_YELLOW      = Gf.Vec3f(0.95, 0.80, 0.02)
C_WHITE       = Gf.Vec3f(0.92, 0.92, 0.93)
C_DARK_GREY   = Gf.Vec3f(0.12, 0.12, 0.14)
C_LABEL_BG    = Gf.Vec3f(0.10, 0.10, 0.35)   # dark blue-black label background
C_WIRE_BAL_Y  = Gf.Vec3f(0.90, 0.80, 0.00)   # balance wire yellow
C_WIRE_BAL_W  = Gf.Vec3f(0.90, 0.90, 0.90)   # balance wire white
C_TAPE_RED    = Gf.Vec3f(0.70, 0.05, 0.05)

# =============================================================================
# MATERIAL FACTORY
# =============================================================================

def make_mat(stage, path: str,
             color: Gf.Vec3f,
             metallic: float   = 0.0,
             roughness: float  = 0.7,
             opacity: float    = 1.0,
             clearcoat: float  = 0.0,
             clearcoat_rough: float = 0.5,
             ior: float        = 1.5) -> UsdShade.Material:
    """
    Create a UsdPreviewSurface PBR material.
    All parameters exposed so each sub-component can be tuned precisely.
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
    shader.CreateInput("opacity",
                       Sdf.ValueTypeNames.Float).Set(opacity)
    shader.CreateInput("clearcoat",
                       Sdf.ValueTypeNames.Float).Set(clearcoat)
    shader.CreateInput("clearcoatRoughness",
                       Sdf.ValueTypeNames.Float).Set(clearcoat_rough)
    shader.CreateInput("ior",
                       Sdf.ValueTypeNames.Float).Set(ior)
    shader.CreateInput("useSpecularWorkflow",
                       Sdf.ValueTypeNames.Int).Set(0)

    mat.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface")
    return mat


def bind(prim, mat: UsdShade.Material):
    UsdShade.MaterialBindingAPI(prim).Bind(mat)

# =============================================================================
# PRIMITIVE HELPERS
# =============================================================================

def add_cube(stage, path: str,
             lx: float, ly: float, lz: float,
             cx: float = 0.0, cy: float = 0.0, cz: float = 0.0,
             mat: UsdShade.Material = None) -> UsdGeom.Cube:
    """Cube with full extents lx × ly × lz centred at (cx, cy, cz)."""
    c  = UsdGeom.Cube.Define(stage, path)
    xf = UsdGeom.Xformable(c)
    xf.AddTranslateOp().Set(Gf.Vec3f(cx, cy, cz))
    xf.AddScaleOp().Set(Gf.Vec3f(lx / 2.0, ly / 2.0, lz / 2.0))
    if mat:
        bind(c.GetPrim(), mat)
    return c


def add_cylinder(stage, path: str,
                 radius: float, height: float,
                 cx: float = 0.0, cy: float = 0.0, cz: float = 0.0,
                 axis: str = "Z",
                 rx: float = 0.0, ry: float = 0.0, rz: float = 0.0,
                 mat: UsdShade.Material = None) -> UsdGeom.Cylinder:
    """Cylinder with optional Euler XYZ rotation."""
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    xf  = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(Gf.Vec3f(cx, cy, cz))
    if rx != 0.0 or ry != 0.0 or rz != 0.0:
        xf.AddRotateXYZOp().Set(Gf.Vec3f(rx, ry, rz))
    if mat:
        bind(cyl.GetPrim(), mat)
    return cyl

# =============================================================================
# ARCH WIRE BUILDER
# Approximates a curved wire as N cylinder segments following a parabolic arch.
# =============================================================================

def build_arch_wire(stage, scope_path: str,
                    start: Gf.Vec3f, end: Gf.Vec3f,
                    arch_z: float,
                    radius: float, segs: int,
                    mat: UsdShade.Material):
    """
    Place `segs` cylinder segments between start and end points,
    arching upward by arch_z in the Z direction.

    start, end  : Gf.Vec3f world positions (in parent space)
    arch_z      : peak height of the arch above the start/end Z
    radius      : wire radius
    segs        : number of cylinder segments
    """
    UsdGeom.Scope.Define(stage, scope_path)

    # Generate arch sample points along a quadratic bezier in XZ
    def sample(t):
        # Quadratic Bezier: P0=start, P1=mid, P2=end
        # mid = midpoint XY, raised by arch_z
        mx = (start[0] + end[0]) / 2.0
        my = (start[1] + end[1]) / 2.0
        mz = max(start[2], end[2]) + arch_z
        P0 = Gf.Vec3f(start[0], start[1], start[2])
        P1 = Gf.Vec3f(mx, my, mz)
        P2 = Gf.Vec3f(end[0], end[1], end[2])
        one_m_t = 1.0 - t
        x = one_m_t**2 * P0[0] + 2*one_m_t*t * P1[0] + t**2 * P2[0]
        y = one_m_t**2 * P0[1] + 2*one_m_t*t * P1[1] + t**2 * P2[1]
        z = one_m_t**2 * P0[2] + 2*one_m_t*t * P1[2] + t**2 * P2[2]
        return Gf.Vec3f(x, y, z)

    pts = [sample(i / segs) for i in range(segs + 1)]

    for i in range(segs):
        p0 = pts[i]
        p1 = pts[i + 1]

        # Segment centre
        cx = (p0[0] + p1[0]) / 2.0
        cy = (p0[1] + p1[1]) / 2.0
        cz = (p0[2] + p1[2]) / 2.0

        # Segment length
        dx = p1[0] - p0[0]
        dy = p1[1] - p0[1]
        dz = p1[2] - p0[2]
        length = math.sqrt(dx*dx + dy*dy + dz*dz)
        if length < 1e-9:
            continue

        # Direction vector (normalised)
        nx, ny, nz = dx/length, dy/length, dz/length

        # Euler angles to rotate Z-axis cylinder to align with segment direction
        # We need the cylinder's Z axis to point along (nx, ny, nz)
        # Use: tilt around Y then rotate around Z
        rot_y = math.degrees(math.atan2(nx, nz))            # yaw in XZ plane
        rot_x = math.degrees(-math.asin(max(-1.0, min(1.0, ny))))  # pitch

        cyl_path = f"{scope_path}/Seg_{i:03d}"
        seg = UsdGeom.Cylinder.Define(stage, cyl_path)
        seg.CreateRadiusAttr(radius)
        seg.CreateHeightAttr(length)
        seg.CreateAxisAttr("Z")
        xf = UsdGeom.Xformable(seg)
        xf.AddTranslateOp().Set(Gf.Vec3f(cx, cy, cz))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(rot_x, rot_y, 0.0))
        if mat:
            bind(seg.GetPrim(), mat)

# =============================================================================
# LABEL SURFACE — programmatic text via embedded primvar metadata
# =============================================================================

def build_label_surface(stage, path: str, mat: UsdShade.Material):
    """
    Create the label slab and embed spec-text as USD custom metadata
    so it can be picked up by downstream tools or a custom shader.
    The label material is visually distinct (dark blue / printed look).
    """
    slab = add_cube(stage, path,
                    LABEL_L, LABEL_W, LABEL_T,
                    0.0, 0.0, LABEL_CZ, mat)

    prim = slab.GetPrim()

    # Store label text lines as USD custom string metadata
    # (readable by any USD tool, and usable as shader primvar in a graph)
    prim.SetCustomDataByKey("label:line0", "OV-POWER")
    prim.SetCustomDataByKey("label:line1", "LI-PO 3S  11.1V  2200mAh")
    prim.SetCustomDataByKey("label:line2", "30C CONTINUOUS DISCHARGE")
    prim.SetCustomDataByKey("label:line3", "⚠ DANGER: FIRE HAZARD  ♻ DO NOT DISPOSE IN BIN")
    prim.SetCustomDataByKey("label:line4", "CHARGE ONLY WITH BALANCE CHARGER")

    # Assign UV primvar coordinates (0-1 range over the full slab face)
    # so an image-based texture can be hot-swapped in later.
    uvs = Vt.Vec2fArray([
        Gf.Vec2f(0.0, 0.0),
        Gf.Vec2f(1.0, 0.0),
        Gf.Vec2f(1.0, 1.0),
        Gf.Vec2f(0.0, 1.0),
    ])
    mesh_api = UsdGeom.PrimvarsAPI(prim)
    uv_pv    = mesh_api.CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray,
        UsdGeom.Tokens.varying)
    uv_pv.Set(uvs)

    return slab

# =============================================================================
# MAIN BUILD FUNCTION
# =============================================================================

def build_lipo_battery():
    """
    Construct the Li-Po 3S Battery Pack digital twin on the active stage.
    """

    # ── Stage ─────────────────────────────────────────────────────────────────
    ctx   = omni.usd.get_context()
    stage = ctx.get_stage()
    if stage is None:
        ctx.new_stage()
        stage = ctx.get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")

    # ── battery_link root ─────────────────────────────────────────────────────
    ROOT      = "/World/battery_link"
    root_xf   = UsdGeom.Xform.Define(stage, ROOT)
    root_prim = root_xf.GetPrim()

    # Battery bottom face at Z = 0; body centred → translate up by BATT_H/2
    xf = UsdGeom.Xformable(root_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, BATT_H / 2.0))

    # ── Materials ─────────────────────────────────────────────────────────────
    M = f"{ROOT}/Materials"
    UsdGeom.Scope.Define(stage, M)

    # Main battery body — matte black heat-shrink silicone
    mat_body = make_mat(stage, f"{M}/Body_MatteBlack",
                        C_BLACK, metallic=0.0, roughness=0.88,
                        clearcoat=0.05, clearcoat_rough=0.90)

    # Label surface — dark blue-black printed sticker
    mat_label = make_mat(stage, f"{M}/Label_Surface",
                         C_LABEL_BG, metallic=0.0, roughness=0.55,
                         clearcoat=0.15, clearcoat_rough=0.40)

    # Red silicone wire insulation
    mat_wire_red = make_mat(stage, f"{M}/Wire_Red",
                            C_RED, metallic=0.0, roughness=0.72)

    # Black silicone wire insulation
    mat_wire_blk = make_mat(stage, f"{M}/Wire_Black",
                            C_DARK_GREY, metallic=0.0, roughness=0.72)

    # XT60 connector — bright hard yellow plastic
    mat_xt60 = make_mat(stage, f"{M}/XT60_Yellow",
                        C_YELLOW, metallic=0.0, roughness=0.30,
                        clearcoat=0.20, clearcoat_rough=0.25)

    # JST-XH connector — white hard plastic
    mat_jst = make_mat(stage, f"{M}/JST_White",
                       C_WHITE, metallic=0.0, roughness=0.35,
                       clearcoat=0.10, clearcoat_rough=0.30)

    # Red electrical tape / cap
    mat_tape_red = make_mat(stage, f"{M}/Tape_Red",
                            C_TAPE_RED, metallic=0.0, roughness=0.80)

    # Black electrical tape / cap
    mat_tape_blk = make_mat(stage, f"{M}/Tape_Black",
                            C_BLACK, metallic=0.0, roughness=0.80)

    # Balance wire colours
    mat_bal_red = mat_wire_red
    mat_bal_blk = mat_wire_blk
    mat_bal_yel = make_mat(stage, f"{M}/BalWire_Yellow",
                           C_WIRE_BAL_Y, metallic=0.0, roughness=0.70)
    mat_bal_wht = make_mat(stage, f"{M}/BalWire_White",
                           C_WIRE_BAL_W, metallic=0.0, roughness=0.70)

    # ── Body scope ────────────────────────────────────────────────────────────
    BODY = f"{ROOT}/Body"
    UsdGeom.Scope.Define(stage, BODY)

    # =========================================================================
    # Battery Main Body — slightly "soft" by using a standard cube
    # (a proper subdivision/chamfer requires a custom mesh; here we use the
    #  cube prim and rely on the matte material + slight scale reduction to
    #  give the heat-shrink "soft wrap" impression)
    # =========================================================================
    body_cube = add_cube(stage, f"{BODY}/Battery_Body",
                         BATT_L, BATT_W, BATT_H,
                         0.0, 0.0, 0.0,
                         mat_body)

    # =========================================================================
    # Label Surface slab — proud of top face (+Z), with UV primvars
    # =========================================================================
    build_label_surface(stage, f"{BODY}/Label_Surface", mat_label)

    # ── Wires scope ───────────────────────────────────────────────────────────
    WIRES = f"{ROOT}/Wires"
    UsdGeom.Scope.Define(stage, WIRES)

    # =========================================================================
    # Main Harness (Power Leads + XT60)
    # =========================================================================
    MAIN = f"{WIRES}/MainHarness"
    UsdGeom.Scope.Define(stage, MAIN)

    # Wire cap / tape block (red + black split)
    add_cube(stage, f"{MAIN}/Cap_Red",
             CAP_L, CAP_W * 0.5, CAP_H,
             CAP_CX, CAP_W * 0.25, 0.0,
             mat_tape_red)
    add_cube(stage, f"{MAIN}/Cap_Black",
             CAP_L, CAP_W * 0.5, CAP_H,
             CAP_CX, -CAP_W * 0.25, 0.0,
             mat_tape_blk)

    # Red power wire — arched from exit to XT60 (+Y offset)
    wire_red_start = Gf.Vec3f(WIRE_EXIT_X + CAP_L, +WIRE_R_THICK * 1.2, WIRE_EXIT_Z)
    wire_red_end   = Gf.Vec3f(XT60_X - XT60_L / 2.0,
                               +WIRE_R_THICK * 1.2, XT60_Z)
    build_arch_wire(stage, f"{MAIN}/Wire_Red",
                    wire_red_start, wire_red_end,
                    WIRE_ARCH_H, WIRE_R_THICK, WIRE_SEGS,
                    mat_wire_red)

    # Black power wire — arched from exit to XT60 (−Y offset)
    wire_blk_start = Gf.Vec3f(WIRE_EXIT_X + CAP_L, -WIRE_R_THICK * 1.2, WIRE_EXIT_Z)
    wire_blk_end   = Gf.Vec3f(XT60_X - XT60_L / 2.0,
                               -WIRE_R_THICK * 1.2, XT60_Z)
    build_arch_wire(stage, f"{MAIN}/Wire_Black",
                    wire_blk_start, wire_blk_end,
                    WIRE_ARCH_H, WIRE_R_THICK, WIRE_SEGS,
                    mat_wire_blk)

    # XT60 connector body — two interlocking blocks (body + hood)
    add_cube(stage, f"{MAIN}/XT60_Body",
             XT60_L, XT60_W, XT60_H,
             XT60_X, 0.0, XT60_Z,
             mat_xt60)
    # XT60 key ridges (thin raised cubes on long sides)
    for side_y in [XT60_W / 2.0 + 0.0005, -(XT60_W / 2.0 + 0.0005)]:
        add_cube(stage, f"{MAIN}/XT60_Ridge_{'P' if side_y > 0 else 'N'}",
                 XT60_L * 0.7, 0.001, XT60_H * 0.6,
                 XT60_X, side_y, XT60_Z,
                 mat_xt60)
    # Gold pin holes (dark inset cylinders on mating face)
    for pin_y in [+XT60_W * 0.22, -XT60_W * 0.22]:
        add_cylinder(stage, f"{MAIN}/XT60_Pin_{'P' if pin_y>0 else 'N'}",
                     radius=0.002, height=0.004,
                     cx=XT60_X + XT60_L / 2.0, cy=pin_y, cz=XT60_Z,
                     axis="X",
                     mat=make_mat(stage,
                                  f"{M}/Gold_Pin_{'P' if pin_y>0 else 'N'}",
                                  Gf.Vec3f(0.83, 0.65, 0.15),
                                  metallic=0.95, roughness=0.15))

    # =========================================================================
    # Balance Lead (4 thin wires + JST-XH)
    # =========================================================================
    BAL = f"{WIRES}/BalanceLead"
    UsdGeom.Scope.Define(stage, BAL)

    # Balance wire cap
    add_cube(stage, f"{BAL}/BalanceCap",
             BCAP_L, BCAP_W, BCAP_H,
             BCAP_CX, BCAP_CY, -0.002,
             mat_tape_blk)

    # Four balance wires — Red, Black, Yellow, White, side by side in Y
    bal_mats   = [mat_bal_red, mat_bal_blk, mat_bal_yel, mat_bal_wht]
    bal_names  = ["Red", "Black", "Yellow", "White"]
    bal_y_offs = [-0.0015, -0.0005, +0.0005, +0.0015]

    for bname, bmat, by in zip(bal_names, bal_mats, bal_y_offs):
        b_start = Gf.Vec3f(WIRE_EXIT_X + BCAP_L,
                           BCAP_CY + by, -0.003)
        b_end   = Gf.Vec3f(JST_X - JST_L / 2.0,
                           BCAP_CY + by, JST_Z)
        build_arch_wire(stage, f"{BAL}/BalWire_{bname}",
                        b_start, b_end,
                        0.005, BAL_R, BAL_SEGS, bmat)

    # JST-XH connector — white plastic housing
    add_cube(stage, f"{BAL}/JST_Body",
             JST_L, JST_W, JST_H,
             JST_X, BCAP_CY, JST_Z,
             mat_jst)
    # JST pin row (4 tiny gold pins recessed into face)
    for pi, py_off in enumerate(bal_y_offs):
        add_cylinder(stage, f"{BAL}/JST_Pin_{pi:02d}",
                     radius=0.0004, height=0.003,
                     cx=JST_X + JST_L / 2.0,
                     cy=BCAP_CY + py_off, cz=JST_Z,
                     axis="X",
                     mat=make_mat(stage, f"{M}/JST_Pin_{pi:02d}",
                                  Gf.Vec3f(0.80, 0.65, 0.10),
                                  metallic=0.95, roughness=0.12))

    # =========================================================================
    # Physics
    # =========================================================================

    # RigidBodyAPI on root
    rb = UsdPhysics.RigidBodyAPI.Apply(root_prim)
    rb.CreateRigidBodyEnabledAttr(True)

    # MassAPI — exactly 180 g = 0.180 kg
    ma = UsdPhysics.MassAPI.Apply(root_prim)
    ma.CreateMassAttr(0.180)

    # Collision proxy — covers only the battery body (wires excluded)
    PROXY = f"{ROOT}/CollisionProxy"
    proxy = add_cube(stage, PROXY,
                     BATT_L, BATT_W, BATT_H,
                     0.0, 0.0, 0.0)
    pp = proxy.GetPrim()
    UsdGeom.Imageable(pp).MakeInvisible()
    UsdPhysics.CollisionAPI.Apply(pp)
    try:
        mc = UsdPhysics.MeshCollisionAPI.Apply(pp)
        mc.CreateApproximationAttr().Set("boundingCube")
    except Exception:
        pass   # cube gets analytical collision automatically

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

    print("=" * 64)
    print("  Li-Po 3S Battery Pack — digital twin created!")
    print(f"  Root prim      : {ROOT}")
    print(f"  Body           : {BATT_L*1e3:.0f} mm × "
          f"{BATT_W*1e3:.0f} mm × {BATT_H*1e3:.0f} mm")
    print(f"  Label slab     : UV primvars set  |  metadata: 5 text lines")
    print(f"  Main wires     : Red + Black, "
          f"R={WIRE_R_THICK*1e3:.1f} mm, {WIRE_SEGS}-seg arch")
    print(f"  Balance wires  : 4 colours, R={BAL_R*1e3:.1f} mm, "
          f"{BAL_SEGS}-seg arch")
    print(f"  XT60 connector : {XT60_L*1e3:.0f}×{XT60_W*1e3:.0f}×"
          f"{XT60_H*1e3:.0f} mm  (yellow)")
    print(f"  JST-XH         : {JST_L*1e3:.0f}×{JST_W*1e3:.0f}×"
          f"{JST_H*1e3:.0f} mm  (white)")
    print(f"  Mass           : 0.180 kg  (180 g)")
    print(f"  Collision proxy: body-only bounding cube")
    print("=" * 64)


# =============================================================================
# Entry point
# =============================================================================
build_lipo_battery()