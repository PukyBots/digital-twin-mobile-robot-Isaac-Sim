# =============================================================================
# L298N Motor Driver Module 3D Model Generator for NVIDIA Omniverse Isaac Sim
# Usage: Paste into Window -> Script Editor and click "Run"
# Dimensions: 43mm x 43mm x 27mm | Mass: 26g
# =============================================================================

import omni.usd
import omni.kit.commands
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade,
    Sdf, Gf, Vt
)

# =============================================================================
# CONSTANTS — All values in meters (Isaac Sim default unit: meters)
# =============================================================================

# --- PCB Board ---
BOARD_L   = 0.043    # 43 mm (X axis)
BOARD_W   = 0.043    # 43 mm (Y axis)
BOARD_H   = 0.0016   # 1.6 mm (Z axis) — standard PCB thickness

# PCB surface Z (top face of board, in parent-local space)
PCB_TOP_Z = BOARD_H / 2.0

# --- Mounting Holes (four corners) ---
HOLE_R      = 0.0015   # 1.5 mm radius (approx 3 mm diameter)
HOLE_H      = BOARD_H + 0.0002   # slightly taller than PCB for visual clarity
HOLE_INSET  = 0.003    # 3 mm inset from each edge
HOLE_POSITIONS = [
    ( BOARD_L/2 - HOLE_INSET,  BOARD_W/2 - HOLE_INSET),
    (-BOARD_L/2 + HOLE_INSET,  BOARD_W/2 - HOLE_INSET),
    ( BOARD_L/2 - HOLE_INSET, -BOARD_W/2 + HOLE_INSET),
    (-BOARD_L/2 + HOLE_INSET, -BOARD_W/2 + HOLE_INSET),
]

# --- Heatsink ---
# Multi-fin block: modelled as a central spine + individual fin slabs
HSINK_SPINE_L  = 0.012   # spine depth (X)
HSINK_SPINE_W  = 0.022   # spine width (Y)
HSINK_SPINE_H  = 0.022   # heatsink body height (Z)
HSINK_X        = 0.0     # centred on X
HSINK_Y        = 0.003   # slight offset toward +Y (board centre)
HSINK_Z        = PCB_TOP_Z + HSINK_SPINE_H / 2.0

# Fins: thin slabs protruding in X from the spine
FIN_COUNT      = 7
FIN_THICKNESS  = 0.0008   # 0.8 mm each fin
FIN_DEPTH      = 0.018    # fin protrudes 18 mm in X beyond spine edge
FIN_H          = HSINK_SPINE_H
FIN_PITCH      = HSINK_SPINE_W / (FIN_COUNT + 1)   # spacing in Y
FIN_PROTRUDE_X = HSINK_SPINE_L / 2.0 + FIN_DEPTH / 2.0

# --- L298N Chip (flat IC on front face of heatsink) ---
CHIP_L  = 0.004   # 4 mm thick (X, pressed against heatsink front face)
CHIP_W  = 0.020   # 20 mm wide (Y)
CHIP_H  = 0.010   # 10 mm tall (Z)
CHIP_X  = -(HSINK_SPINE_L / 2.0) - CHIP_L / 2.0   # front face of heatsink
CHIP_Y  = HSINK_Y
CHIP_Z  = PCB_TOP_Z + CHIP_H / 2.0

# --- Screw Terminals ---
# Blue rectangular blocks; screws are small silver cylinders on top
TERM_H          = 0.012   # terminal block height
TERM_PIN_W      = 0.0075  # width per pin slot
TERM_DEPTH      = 0.009   # depth (protrudes off edge)
SCREW_R         = 0.0025  # screw head radius
SCREW_H         = 0.001   # screw head height

# Left motor terminal (2-pin) — left edge (−X side)
TERM_L_PINS     = 2
TERM_L_W        = TERM_PIN_W * TERM_L_PINS   # 15 mm
TERM_L_X        = -(BOARD_L / 2.0) + TERM_DEPTH / 2.0
TERM_L_Y        =  0.005   # offset toward +Y
TERM_L_Z        =  PCB_TOP_Z + TERM_H / 2.0

# Right motor terminal (2-pin) — right edge (+X side)
TERM_R_PINS     = 2
TERM_R_W        = TERM_PIN_W * TERM_R_PINS
TERM_R_X        = (BOARD_L / 2.0) - TERM_DEPTH / 2.0
TERM_R_Y        =  0.005
TERM_R_Z        =  PCB_TOP_Z + TERM_H / 2.0

# Front power terminal (3-pin) — front edge (−Y side)
TERM_F_PINS     = 3
TERM_F_L        = TERM_PIN_W * TERM_F_PINS   # 22.5 mm
TERM_F_W        = TERM_DEPTH
TERM_F_X        = -0.002   # slightly left of centre
TERM_F_Y        = -(BOARD_W / 2.0) + TERM_DEPTH / 2.0
TERM_F_Z        =  PCB_TOP_Z + TERM_H / 2.0

# --- Logic Header Pins (6-pin single row) ---
LPIN_R          = 0.00025   # 0.25 mm radius
LPIN_H          = 0.007     # 7 mm tall above PCB
LPIN_PITCH      = 0.00254   # 2.54 mm standard
LPIN_COUNT      = 6
LPIN_BASE_L     = LPIN_PITCH * LPIN_COUNT + 0.002
LPIN_BASE_W     = 0.004
LPIN_BASE_H     = 0.002
LPIN_START_X    = -(LPIN_PITCH * (LPIN_COUNT - 1)) / 2.0
LPIN_Y          = -(BOARD_W / 2.0) + 0.011   # just behind front terminal
LPIN_BASE_Z     =  PCB_TOP_Z + LPIN_BASE_H / 2.0
LPIN_CYL_Z      =  PCB_TOP_Z + LPIN_BASE_H + LPIN_H / 2.0

# --- Electrolytic Capacitors (2×) ---
CAP_R           = 0.0025   # 2.5 mm radius (5 mm diameter cap)
CAP_H           = 0.011    # 11 mm tall
CAP_TOP_R       = CAP_R - 0.0002   # slightly smaller silver top disc
CAP_TOP_H       = 0.0008
# Placed behind the heatsink, symmetric about Y axis
CAP_Z           = PCB_TOP_Z + CAP_H / 2.0
CAP_POSITIONS   = [
    (-0.008,  0.014),   # left cap  (X, Y)
    ( 0.008,  0.014),   # right cap
]

# --- Bounding Box Collider (invisible physics proxy) ---
TOTAL_H         = 0.027    # 27 mm as specified

# =============================================================================
# HELPERS
# =============================================================================

def create_material(stage, path: str, color: Gf.Vec3f,
                    metallic: float = 0.0, roughness: float = 0.5) -> UsdShade.Material:
    """
    Define a UsdPreviewSurface material and return the Material prim.
    Works on all Isaac Sim versions without requiring MDL / OmniPBR MDL files.
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


def bind_mat(prim, mat: UsdShade.Material):
    """Bind a material to a prim via UsdShade.MaterialBindingAPI."""
    UsdShade.MaterialBindingAPI(prim).Bind(mat)


def add_cube(stage, path: str,
             size: Gf.Vec3f, translate: Gf.Vec3f,
             mat: UsdShade.Material = None) -> UsdGeom.Cube:
    """
    Create a Cube prim scaled to full extents (size = full L×W×H).
    UsdGeom.Cube default edge length = 2 (unit cube), so scale = half-extents.
    """
    cube = UsdGeom.Cube.Define(stage, path)
    xf   = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(translate)
    xf.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))
    if mat:
        bind_mat(cube.GetPrim(), mat)
    return cube


def add_cylinder(stage, path: str,
                 radius: float, height: float,
                 translate: Gf.Vec3f,
                 rotate_xyz: Gf.Vec3f = None,
                 axis: str = "Z",
                 mat: UsdShade.Material = None) -> UsdGeom.Cylinder:
    """
    Create a Cylinder prim. Default axis = Z (vertical).
    Optional rotate_xyz applies Euler XYZ rotation in degrees.
    """
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    xf  = UsdGeom.Xformable(cyl)
    xf.AddTranslateOp().Set(translate)
    if rotate_xyz is not None:
        xf.AddRotateXYZOp().Set(rotate_xyz)
    if mat:
        bind_mat(cyl.GetPrim(), mat)
    return cyl


# =============================================================================
# MAIN BUILD FUNCTION
# =============================================================================

def build_l298n():
    """
    Construct the full L298N Motor Driver Module on the current Isaac Sim stage.
    All geometry is placed under /World/L298N_MotorDriver.
    """

    # -------------------------------------------------------------------------
    # 1. Stage setup
    # -------------------------------------------------------------------------
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # -------------------------------------------------------------------------
    # 2. Root Xforms
    # -------------------------------------------------------------------------
    world_path = "/World"
    if not stage.GetPrimAtPath(world_path):
        UsdGeom.Xform.Define(stage, world_path)

    root_path  = "/World/L298N_MotorDriver"
    root_xform = UsdGeom.Xform.Define(stage, root_path)
    root_prim  = root_xform.GetPrim()

    # Lift the assembly so the board bottom rests on the ground plane (Z = 0)
    xf = UsdGeom.Xformable(root_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TOTAL_H / 2.0))

    # -------------------------------------------------------------------------
    # 3. Materials
    # -------------------------------------------------------------------------
    mats_path = f"{root_path}/Materials"
    UsdGeom.Scope.Define(stage, mats_path)

    mat_pcb    = create_material(stage, f"{mats_path}/PCB_Red",
                                 Gf.Vec3f(0.72, 0.05, 0.05),
                                 metallic=0.0, roughness=0.35)  # glossy red

    mat_silver = create_material(stage, f"{mats_path}/Metal_Silver",
                                 Gf.Vec3f(0.80, 0.82, 0.85),
                                 metallic=1.0, roughness=0.12)  # shiny silver

    mat_black  = create_material(stage, f"{mats_path}/Matte_Black",
                                 Gf.Vec3f(0.05, 0.05, 0.05),
                                 metallic=0.0, roughness=0.85)  # matte black

    mat_blue   = create_material(stage, f"{mats_path}/Matte_Blue",
                                 Gf.Vec3f(0.08, 0.18, 0.65),
                                 metallic=0.0, roughness=0.80)  # matte blue

    mat_hole   = create_material(stage, f"{mats_path}/Hole_Dark",
                                 Gf.Vec3f(0.02, 0.02, 0.02),
                                 metallic=0.0, roughness=1.0)   # near-black hole

    mat_cap_top = create_material(stage, f"{mats_path}/Cap_Silver_Top",
                                  Gf.Vec3f(0.75, 0.75, 0.75),
                                  metallic=0.6, roughness=0.3)  # aluminium cap top

    # -------------------------------------------------------------------------
    # 4. Geometry scope
    # -------------------------------------------------------------------------
    geo_path = f"{root_path}/Geometry"
    UsdGeom.Scope.Define(stage, geo_path)

    # =========================================================================
    # 4a. PCB Board
    # =========================================================================
    add_cube(
        stage, f"{geo_path}/PCB_Board",
        Gf.Vec3f(BOARD_L, BOARD_W, BOARD_H),
        Gf.Vec3f(0.0, 0.0, 0.0),
        mat_pcb
    )

    # =========================================================================
    # 4b. Mounting Holes (dark cylinders on PCB corners)
    # =========================================================================
    holes_scope = f"{geo_path}/MountingHoles"
    UsdGeom.Scope.Define(stage, holes_scope)

    for idx, (hx, hy) in enumerate(HOLE_POSITIONS):
        add_cylinder(
            stage, f"{holes_scope}/Hole_{idx:02d}",
            radius=HOLE_R, height=HOLE_H,
            translate=Gf.Vec3f(hx, hy, 0.0),
            mat=mat_hole
        )

    # =========================================================================
    # 4c. Heatsink — spine + fins
    # =========================================================================
    hsink_scope = f"{geo_path}/Heatsink"
    UsdGeom.Scope.Define(stage, hsink_scope)

    # Central spine block
    add_cube(
        stage, f"{hsink_scope}/Spine",
        Gf.Vec3f(HSINK_SPINE_L, HSINK_SPINE_W, HSINK_SPINE_H),
        Gf.Vec3f(HSINK_X, HSINK_Y, HSINK_Z),
        mat_silver
    )

    # Fins — evenly spaced along Y, protruding in +X from spine
    fins_scope = f"{hsink_scope}/Fins"
    UsdGeom.Scope.Define(stage, fins_scope)

    fin_start_y = HSINK_Y - HSINK_SPINE_W / 2.0 + FIN_PITCH
    for fi in range(FIN_COUNT):
        fy = fin_start_y + fi * FIN_PITCH
        add_cube(
            stage, f"{fins_scope}/Fin_{fi:02d}",
            Gf.Vec3f(FIN_DEPTH, FIN_THICKNESS, FIN_H),
            Gf.Vec3f(HSINK_X + FIN_PROTRUDE_X, fy, HSINK_Z),
            mat_silver
        )

    # =========================================================================
    # 4d. L298N Chip (flat black IC against heatsink front face)
    # =========================================================================
    add_cube(
        stage, f"{geo_path}/L298N_Chip",
        Gf.Vec3f(CHIP_L, CHIP_W, CHIP_H),
        Gf.Vec3f(CHIP_X, CHIP_Y, CHIP_Z),
        mat_black
    )

    # =========================================================================
    # 4e. Screw Terminals — blue blocks + silver screw heads
    # =========================================================================
    terms_scope = f"{geo_path}/ScrewTerminals"
    UsdGeom.Scope.Define(stage, terms_scope)

    def add_terminal_with_screws(base_path: str, num_pins: int,
                                 block_l: float, block_w: float,
                                 tx: float, ty: float, tz: float,
                                 screw_axis: str = "Y"):
        """
        Create a blue terminal block and place silver screw-head cylinders on top.
        screw_axis: layout axis for pin offsets ('X' or 'Y').
        """
        # Blue body
        add_cube(
            stage, f"{base_path}/Body",
            Gf.Vec3f(block_l, block_w, TERM_H),
            Gf.Vec3f(tx, ty, tz),
            mat_blue
        )
        # Screw head cylinders on top of block
        screws_scope = f"{base_path}/Screws"
        UsdGeom.Scope.Define(stage, screws_scope)
        screw_top_z = tz + TERM_H / 2.0 + SCREW_H / 2.0
        for si in range(num_pins):
            offset = (si - (num_pins - 1) / 2.0) * TERM_PIN_W
            sx = tx + (offset if screw_axis == "X" else 0.0)
            sy = ty + (offset if screw_axis == "Y" else 0.0)
            add_cylinder(
                stage, f"{screws_scope}/Screw_{si:02d}",
                radius=SCREW_R, height=SCREW_H,
                translate=Gf.Vec3f(sx, sy, screw_top_z),
                mat=mat_silver
            )

    # Left 2-pin terminal (block depth runs along X, screws spaced in Y)
    add_terminal_with_screws(
        f"{terms_scope}/Terminal_Left", TERM_L_PINS,
        TERM_DEPTH, TERM_L_W,
        TERM_L_X, TERM_L_Y, TERM_L_Z,
        screw_axis="Y"
    )

    # Right 2-pin terminal
    add_terminal_with_screws(
        f"{terms_scope}/Terminal_Right", TERM_R_PINS,
        TERM_DEPTH, TERM_R_W,
        TERM_R_X, TERM_R_Y, TERM_R_Z,
        screw_axis="Y"
    )

    # Front 3-pin power terminal (block depth runs along Y, screws spaced in X)
    add_terminal_with_screws(
        f"{terms_scope}/Terminal_Front", TERM_F_PINS,
        TERM_F_L, TERM_F_W,
        TERM_F_X, TERM_F_Y, TERM_F_Z,
        screw_axis="X"
    )

    # =========================================================================
    # 4f. Logic Header Pins (6-pin row)
    # =========================================================================
    logic_scope = f"{geo_path}/LogicPins"
    UsdGeom.Scope.Define(stage, logic_scope)

    # Black plastic base for the header
    add_cube(
        stage, f"{logic_scope}/Base",
        Gf.Vec3f(LPIN_BASE_L, LPIN_BASE_W, LPIN_BASE_H),
        Gf.Vec3f(0.0, LPIN_Y, LPIN_BASE_Z),
        mat_black
    )

    # Individual metal pins
    pins_scope = f"{logic_scope}/Pins"
    UsdGeom.Scope.Define(stage, pins_scope)

    for pi in range(LPIN_COUNT):
        px = LPIN_START_X + pi * LPIN_PITCH
        add_cylinder(
            stage, f"{pins_scope}/Pin_{pi:02d}",
            radius=LPIN_R, height=LPIN_H,
            translate=Gf.Vec3f(px, LPIN_Y, LPIN_CYL_Z),
            mat=mat_silver
        )

    # =========================================================================
    # 4g. Electrolytic Capacitors (2×)
    # =========================================================================
    caps_scope = f"{geo_path}/Capacitors"
    UsdGeom.Scope.Define(stage, caps_scope)

    for ci, (cx, cy) in enumerate(CAP_POSITIONS):
        cap_base = f"{caps_scope}/Cap_{ci:02d}"
        UsdGeom.Scope.Define(stage, cap_base)

        # Black cylindrical body
        add_cylinder(
            stage, f"{cap_base}/Body",
            radius=CAP_R, height=CAP_H,
            translate=Gf.Vec3f(cx, cy, CAP_Z),
            mat=mat_black
        )
        # Silver aluminium top disc
        add_cylinder(
            stage, f"{cap_base}/Top",
            radius=CAP_TOP_R, height=CAP_TOP_H,
            translate=Gf.Vec3f(cx, cy, PCB_TOP_Z + CAP_H + CAP_TOP_H / 2.0),
            mat=mat_cap_top
        )

    # =========================================================================
    # 5. Physics
    # =========================================================================

    # RigidBodyAPI on root prim
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(root_prim)
    rigid_api.CreateRigidBodyEnabledAttr(True)

    # MassAPI — exactly 26 g = 0.026 kg
    mass_api = UsdPhysics.MassAPI.Apply(root_prim)
    mass_api.CreateMassAttr(0.026)

    # Invisible bounding-box collision proxy (avoids iterating fine geometry)
    proxy_path = f"{root_path}/CollisionProxy"
    proxy_cube = add_cube(
        stage, proxy_path,
        Gf.Vec3f(BOARD_L, BOARD_W, TOTAL_H),
        Gf.Vec3f(0.0, 0.0, 0.0)
    )
    proxy_prim = proxy_cube.GetPrim()
    UsdGeom.Imageable(proxy_prim).MakeInvisible()

    UsdPhysics.CollisionAPI.Apply(proxy_prim)
    mesh_col = UsdPhysics.MeshCollisionAPI.Apply(proxy_prim)
    mesh_col.CreateApproximationAttr("boundingCube")

    # =========================================================================
    # 6. Focus viewport on the new model
    # =========================================================================
    try:
        import omni.kit.viewport.utility as vp_util
        vp = vp_util.get_active_viewport()
        if vp:
            omni.kit.commands.execute(
                "FramePrimsCommand",
                prim_to_frame=root_path,
                viewport_api=vp
            )
    except Exception:
        pass   # Viewport utility API may vary across Isaac Sim versions

    # =========================================================================
    # 7. Save and report
    # =========================================================================
    stage.Save()

    print("=" * 62)
    print("  L298N Motor Driver Module — model created successfully!")
    print(f"  Root prim  : {root_path}")
    print(f"  Board size : {BOARD_L*1000:.0f} mm × {BOARD_W*1000:.0f} mm × {TOTAL_H*1000:.0f} mm")
    print(f"  Mass       : 0.026 kg  (26 g)")
    print(f"  Logic pins : {LPIN_COUNT}")
    print(f"  Caps       : {len(CAP_POSITIONS)}")
    print(f"  Heatsink fins : {FIN_COUNT}")
    print("=" * 62)


# =============================================================================
# Entry point — executed when script is run in the Isaac Sim Script Editor
# =============================================================================
build_l298n()