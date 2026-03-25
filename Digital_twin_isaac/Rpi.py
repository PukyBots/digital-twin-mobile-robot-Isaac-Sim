# =============================================================================
# Raspberry Pi 3D Model Generator for NVIDIA Omniverse Isaac Sim
# Usage: Paste into Window -> Script Editor and click "Run"
# Dimensions: 85mm x 56mm x 17mm | Mass: 46g
# =============================================================================

import omni.usd
import omni.kit.commands
from pxr import (
    Usd, UsdGeom, UsdPhysics, UsdShade,
    Sdf, Gf, Vt
)

# =============================================================================
# CONSTANTS — All values in meters (Isaac Sim default unit)
# =============================================================================

# Board overall dimensions
BOARD_L  = 0.085   # 85 mm  (X axis)
BOARD_W  = 0.056   # 56 mm  (Y axis)
BOARD_H  = 0.0015  # 1.5 mm (Z axis) — PCB thickness only

# Corner radius approximation (we chamfer via small cylinders at corners)
CORNER_R = 0.003   # 3 mm rounded corner radius

# Mounting hole dimensions
HOLE_R   = 0.00275 / 2.0   # 2.75 mm diameter -> radius
HOLE_H   = BOARD_H + 0.0002  # slightly taller than board for clean cut appearance
# Hole offsets from board edges (standard Raspberry Pi mounting pattern)
HOLE_INSET_X = 0.0035
HOLE_INSET_Y = 0.0035
# Four corner positions (in board-local XY, Z at board top surface)
HOLE_POSITIONS = [
    ( BOARD_L/2 - HOLE_INSET_X,  BOARD_W/2 - HOLE_INSET_Y),
    (-BOARD_L/2 + HOLE_INSET_X,  BOARD_W/2 - HOLE_INSET_Y),
    ( BOARD_L/2 - HOLE_INSET_X, -BOARD_W/2 + HOLE_INSET_Y),
    (-BOARD_L/2 + HOLE_INSET_X, -BOARD_W/2 + HOLE_INSET_Y),
]

# GPIO header — 40-pin (2 rows × 20 cols)
GPIO_PIN_R      = 0.00025   # 0.25 mm pin radius
GPIO_PIN_H      = 0.0058    # 5.8 mm total pin height above PCB
GPIO_BASE_L     = 0.051     # plastic base length
GPIO_BASE_W     = 0.005     # plastic base width
GPIO_BASE_H     = 0.0025    # plastic base height
GPIO_PIN_PITCH  = 0.00254   # 2.54 mm standard pitch
GPIO_ROWS       = 2
GPIO_COLS       = 20
# Header sits along the top edge (+Y), offset from center
GPIO_CENTER_X   = -BOARD_L/2 + 0.007 + GPIO_BASE_L / 2   # ~7mm from left edge
GPIO_CENTER_Y   =  BOARD_W/2 - 0.003                       # near top edge
GPIO_BASE_Z     =  BOARD_H / 2 + GPIO_BASE_H / 2

# SoC (System on Chip) — BCM2711 style square chip
SOC_L   = 0.014    # 14 mm
SOC_W   = 0.014    # 14 mm
SOC_H   = 0.0015   # 1.5 mm height
SOC_X   = 0.005    # slightly right of center
SOC_Y   = 0.002    # slightly above center
SOC_Z   = BOARD_H / 2 + SOC_H / 2

# Right-edge ports (positive X side) — Ethernet + 2× USB-A
PORT_Z_BASE = BOARD_H / 2   # sits on top of PCB surface

ETH_L   = 0.0155   # 15.5 mm deep
ETH_W   = 0.016    # 16 mm wide
ETH_H   = 0.014    # 14 mm tall
ETH_X   = BOARD_L / 2 - ETH_L / 2
ETH_Y   = BOARD_W / 2 - 0.024   # positioned near top-right
ETH_Z   = PORT_Z_BASE + ETH_H / 2

# Double-stacked USB-A connector block
USB_L   = 0.016    # depth
USB_W   = 0.014    # single port width
USB_H   = 0.016    # height of one port
USB_X   = BOARD_L / 2 - USB_L / 2
USB1_Y  = ETH_Y - ETH_W / 2 - USB_W / 2 - 0.001
USB2_Y  = USB1_Y - USB_W - 0.001
USB_Z   = PORT_Z_BASE + USB_H / 2

# Bottom-edge ports (negative Y side)
USBC_L   = 0.0078   # USB-C/power
USBC_W   = 0.009
USBC_H   = 0.003
USBC_X   = -BOARD_L/2 + 0.015
USBC_Y   = -BOARD_W/2 + USBC_W / 2 - 0.0015
USBC_Z   = PORT_Z_BASE + USBC_H / 2

HDMI_L   = 0.0075
HDMI_W   = 0.010
HDMI_H   = 0.003
HDMI1_X  = USBC_X + USBC_L / 2 + 0.003 + HDMI_L / 2
HDMI2_X  = HDMI1_X + HDMI_L + 0.003
HDMI_Y   = -BOARD_W/2 + HDMI_W / 2 - 0.0015
HDMI_Z   = PORT_Z_BASE + HDMI_H / 2

AUDIO_R  = 0.003    # 3.5 mm audio jack radius
AUDIO_H  = 0.012
AUDIO_X  = HDMI2_X + HDMI_L / 2 + 0.006
AUDIO_Y  = -BOARD_W/2 + AUDIO_R + 0.001
AUDIO_Z  = PORT_Z_BASE + AUDIO_H / 2

# Total assembly height (used for bounding-box collider)
TOTAL_H  = 0.017   # 17 mm as specified

# =============================================================================
# HELPER: safe USD path string
# =============================================================================

def safe_path(base: str, name: str) -> str:
    """Return a valid SdfPath string by joining base prim path and child name."""
    return f"{base}/{name}"


# =============================================================================
# MATERIAL CREATION
# =============================================================================

def create_material(stage, mat_path: str, color: Gf.Vec3f,
                    metallic: float = 0.0, roughness: float = 0.5) -> UsdShade.Material:
    """
    Create a UsdPreviewSurface material at mat_path and return the Material prim.
    Falls back gracefully if OmniPBR MDL is unavailable (pure USD Preview Surface).
    """
    material = UsdShade.Material.Define(stage, mat_path)
    shader   = UsdShade.Shader.Define(stage, f"{mat_path}/Shader")

    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor",  Sdf.ValueTypeNames.Color3f).Set(color)
    shader.CreateInput("metallic",      Sdf.ValueTypeNames.Float).Set(metallic)
    shader.CreateInput("roughness",     Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput("useSpecularWorkflow", Sdf.ValueTypeNames.Int).Set(0)

    material.CreateSurfaceOutput().ConnectToSource(
        shader.ConnectableAPI(), "surface"
    )
    return material


def bind_material(prim, material: UsdShade.Material):
    """Bind a UsdShade.Material to a prim."""
    UsdShade.MaterialBindingAPI(prim).Bind(material)


# =============================================================================
# GEOMETRY HELPERS
# =============================================================================

def add_cube(stage, path: str, size: Gf.Vec3f, translate: Gf.Vec3f,
             material: UsdShade.Material = None) -> UsdGeom.Cube:
    """
    Define a Cube prim scaled to the given size (half-extents handled via xformOp:scale).
    size   = (full_length_x, full_width_y, full_height_z)
    translate = world offset within parent scope
    """
    cube = UsdGeom.Cube.Define(stage, path)
    xform = UsdGeom.Xformable(cube)
    xform.AddTranslateOp().Set(translate)
    # UsdGeom.Cube has default size 2 (unit cube -1..1), so scale by half-extents
    xform.AddScaleOp().Set(Gf.Vec3f(size[0] / 2.0, size[1] / 2.0, size[2] / 2.0))
    if material:
        bind_material(cube.GetPrim(), material)
    return cube


def add_cylinder(stage, path: str, radius: float, height: float,
                 translate: Gf.Vec3f, rotate_xyz: Gf.Vec3f = Gf.Vec3f(0, 0, 0),
                 axis: str = "Z",
                 material: UsdShade.Material = None) -> UsdGeom.Cylinder:
    """
    Define a Cylinder prim. Height runs along 'axis'.
    rotate_xyz = Euler XYZ rotation in degrees for orientation.
    """
    cyl = UsdGeom.Cylinder.Define(stage, path)
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(axis)
    xform = UsdGeom.Xformable(cyl)
    xform.AddTranslateOp().Set(translate)
    if rotate_xyz != Gf.Vec3f(0, 0, 0):
        xform.AddRotateXYZOp().Set(rotate_xyz)
    if material:
        bind_material(cyl.GetPrim(), material)
    return cyl


# =============================================================================
# MAIN SCRIPT
# =============================================================================

def build_raspberry_pi():
    """
    Construct the full Raspberry Pi model on the current stage.
    All geometry is placed under /World/RaspberryPi as a hierarchy of Xforms.
    """

    # -------------------------------------------------------------------------
    # 1. Ensure a valid stage exists
    # -------------------------------------------------------------------------
    stage = omni.usd.get_context().get_stage()
    if stage is None:
        omni.usd.get_context().new_stage()
        stage = omni.usd.get_context().get_stage()

    # Set stage up-axis to Z (Isaac Sim default)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    # Set stage units to meters
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    # -------------------------------------------------------------------------
    # 2. Create root World Xform (if missing) and RaspberryPi parent Xform
    # -------------------------------------------------------------------------
    world_path = "/World"
    if not stage.GetPrimAtPath(world_path):
        UsdGeom.Xform.Define(stage, world_path)

    rpi_path = "/World/RaspberryPi"
    rpi_xform = UsdGeom.Xform.Define(stage, rpi_path)
    rpi_prim  = rpi_xform.GetPrim()

    # Place assembly at origin, offset Z so board bottom sits on ground plane
    xf = UsdGeom.Xformable(rpi_prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, TOTAL_H / 2.0))

    # -------------------------------------------------------------------------
    # 3. Materials scope
    # -------------------------------------------------------------------------
    mats_path = f"{rpi_path}/Materials"
    UsdGeom.Scope.Define(stage, mats_path)

    mat_pcb      = create_material(stage, f"{mats_path}/PCB",
                                   Gf.Vec3f(0.05, 0.25, 0.08),
                                   metallic=0.0, roughness=0.7)   # dark green

    mat_metal    = create_material(stage, f"{mats_path}/Metal",
                                   Gf.Vec3f(0.8, 0.82, 0.85),
                                   metallic=1.0, roughness=0.15)  # shiny silver

    mat_gold_pin = create_material(stage, f"{mats_path}/GoldPin",
                                   Gf.Vec3f(0.83, 0.68, 0.21),
                                   metallic=1.0, roughness=0.2)   # gold

    mat_black    = create_material(stage, f"{mats_path}/MatteBlack",
                                   Gf.Vec3f(0.05, 0.05, 0.05),
                                   metallic=0.0, roughness=0.9)   # matte black

    mat_hole     = create_material(stage, f"{mats_path}/HoleDark",
                                   Gf.Vec3f(0.02, 0.02, 0.02),
                                   metallic=0.0, roughness=1.0)   # near black

    # -------------------------------------------------------------------------
    # 4. PCB Base Board
    # -------------------------------------------------------------------------
    geom_path = f"{rpi_path}/Geometry"
    UsdGeom.Scope.Define(stage, geom_path)

    board_path = f"{geom_path}/PCB_Board"
    add_cube(
        stage, board_path,
        Gf.Vec3f(BOARD_L, BOARD_W, BOARD_H),
        Gf.Vec3f(0.0, 0.0, 0.0),
        mat_pcb
    )

    # -------------------------------------------------------------------------
    # 5. Mounting Holes (visual dark cylinders punched into PCB)
    # -------------------------------------------------------------------------
    holes_scope = f"{geom_path}/MountingHoles"
    UsdGeom.Scope.Define(stage, holes_scope)

    for idx, (hx, hy) in enumerate(HOLE_POSITIONS):
        add_cylinder(
            stage, f"{holes_scope}/Hole_{idx:02d}",
            radius=HOLE_R, height=HOLE_H,
            translate=Gf.Vec3f(hx, hy, 0.0),
            material=mat_hole
        )

    # -------------------------------------------------------------------------
    # 6. GPIO Header
    # -------------------------------------------------------------------------
    gpio_scope = f"{geom_path}/GPIO_Header"
    UsdGeom.Scope.Define(stage, gpio_scope)

    # Plastic base block
    add_cube(
        stage, f"{gpio_scope}/GPIO_Base",
        Gf.Vec3f(GPIO_BASE_L, GPIO_BASE_W, GPIO_BASE_H),
        Gf.Vec3f(GPIO_CENTER_X, GPIO_CENTER_Y, GPIO_BASE_Z),
        mat_black
    )

    # 40 individual pins (2 rows × 20 cols)
    pins_scope = f"{gpio_scope}/Pins"
    UsdGeom.Scope.Define(stage, pins_scope)

    pin_count = 0
    col_start_x = GPIO_CENTER_X - (GPIO_COLS - 1) * GPIO_PIN_PITCH / 2.0
    row_offsets  = [-GPIO_PIN_PITCH / 2.0, GPIO_PIN_PITCH / 2.0]  # two rows in Y

    for row in range(GPIO_ROWS):
        for col in range(GPIO_COLS):
            px = col_start_x + col * GPIO_PIN_PITCH
            py = GPIO_CENTER_Y + row_offsets[row]
            pz = BOARD_H / 2.0 + GPIO_PIN_H / 2.0
            add_cylinder(
                stage, f"{pins_scope}/Pin_{pin_count:03d}",
                radius=GPIO_PIN_R, height=GPIO_PIN_H,
                translate=Gf.Vec3f(px, py, pz),
                material=mat_gold_pin
            )
            pin_count += 1

    # -------------------------------------------------------------------------
    # 7. SoC Chip (BCM2711)
    # -------------------------------------------------------------------------
    add_cube(
        stage, f"{geom_path}/SoC_BCM2711",
        Gf.Vec3f(SOC_L, SOC_W, SOC_H),
        Gf.Vec3f(SOC_X, SOC_Y, SOC_Z),
        mat_black
    )

    # -------------------------------------------------------------------------
    # 8. Right-Edge Ports — Ethernet
    # -------------------------------------------------------------------------
    ports_right = f"{geom_path}/Ports_RightEdge"
    UsdGeom.Scope.Define(stage, ports_right)

    add_cube(
        stage, f"{ports_right}/Ethernet",
        Gf.Vec3f(ETH_L, ETH_W, ETH_H),
        Gf.Vec3f(ETH_X, ETH_Y, ETH_Z),
        mat_metal
    )

    # Right-Edge Ports — USB-A ×2 (stacked vertically)
    for usb_idx, uy in enumerate([USB1_Y, USB2_Y]):
        add_cube(
            stage, f"{ports_right}/USB_A_{usb_idx}",
            Gf.Vec3f(USB_L, USB_W, USB_H),
            Gf.Vec3f(USB_X, uy, USB_Z),
            mat_metal
        )

    # -------------------------------------------------------------------------
    # 9. Bottom-Edge Ports — USB-C power, micro-HDMI ×2, Audio Jack
    # -------------------------------------------------------------------------
    ports_bottom = f"{geom_path}/Ports_BottomEdge"
    UsdGeom.Scope.Define(stage, ports_bottom)

    # USB-C / Micro-USB power port
    add_cube(
        stage, f"{ports_bottom}/Power_USBC",
        Gf.Vec3f(USBC_L, USBC_W, USBC_H),
        Gf.Vec3f(USBC_X, USBC_Y, USBC_Z),
        mat_metal
    )

    # micro-HDMI port 1
    add_cube(
        stage, f"{ports_bottom}/MicroHDMI_0",
        Gf.Vec3f(HDMI_L, HDMI_W, HDMI_H),
        Gf.Vec3f(HDMI1_X, HDMI_Y, HDMI_Z),
        mat_metal
    )

    # micro-HDMI port 2
    add_cube(
        stage, f"{ports_bottom}/MicroHDMI_1",
        Gf.Vec3f(HDMI_L, HDMI_W, HDMI_H),
        Gf.Vec3f(HDMI2_X, HDMI_Y, HDMI_Z),
        mat_metal
    )

    # 3.5 mm Audio Jack — cylindrical, protruding from bottom edge
    # Rotate 90° around X so the cylinder axis aligns with Y (pointing outward)
    add_cylinder(
        stage, f"{ports_bottom}/AudioJack",
        radius=AUDIO_R, height=AUDIO_H,
        translate=Gf.Vec3f(AUDIO_X, AUDIO_Y, AUDIO_Z),
        rotate_xyz=Gf.Vec3f(90.0, 0.0, 0.0),  # align along Y axis
        material=mat_metal
    )

    # -------------------------------------------------------------------------
    # 10. Physics — RigidBody + CollisionAPI on parent Xform
    # -------------------------------------------------------------------------

    # Apply RigidBodyAPI to the root RaspberryPi prim
    rigid_api = UsdPhysics.RigidBodyAPI.Apply(rpi_prim)
    # Enable rigid body simulation
    rigid_api.CreateRigidBodyEnabledAttr(True)

    # Apply MassAPI and set exact mass = 46 g = 0.046 kg
    mass_api = UsdPhysics.MassAPI.Apply(rpi_prim)
    mass_api.CreateMassAttr(0.046)   # kg

    # For collision we use a simplified bounding-box proxy cube
    # so physics doesn't iterate over 40+ individual pin cylinders.
    collision_proxy_path = f"{rpi_path}/CollisionProxy"
    proxy_cube = add_cube(
        stage, collision_proxy_path,
        Gf.Vec3f(BOARD_L, BOARD_W, TOTAL_H),
        Gf.Vec3f(0.0, 0.0, 0.0)
    )
    proxy_prim = proxy_cube.GetPrim()

    # Make the proxy invisible (it is purely for physics)
    UsdGeom.Imageable(proxy_prim).MakeInvisible()

    # Apply CollisionAPI to the proxy
    UsdPhysics.CollisionAPI.Apply(proxy_prim)

    # Mark proxy as a MeshCollisionAPI with convex hull approximation
    mesh_col = UsdPhysics.MeshCollisionAPI.Apply(proxy_prim)
    mesh_col.CreateApproximationAttr("boundingCube")

    # -------------------------------------------------------------------------
    # 11. Set default camera so the model is immediately visible
    # -------------------------------------------------------------------------
    # Focus the viewport on the newly created prim
    try:
        import omni.kit.viewport.utility as vp_util
        viewport = vp_util.get_active_viewport()
        if viewport:
            omni.kit.commands.execute(
                "FramePrimsCommand",
                prim_to_frame=rpi_path,
                viewport_api=viewport
            )
    except Exception:
        # Viewport utility may differ across Isaac Sim versions — skip gracefully
        pass

    # -------------------------------------------------------------------------
    # 12. Save stage state
    # -------------------------------------------------------------------------
    stage.Save()

    print("=" * 60)
    print("Raspberry Pi model successfully created!")
    print(f"  Root prim  : {rpi_path}")
    print(f"  Board size : {BOARD_L*1000:.1f} mm × {BOARD_W*1000:.1f} mm × {TOTAL_H*1000:.1f} mm")
    print(f"  Mass       : 0.046 kg (46 g)")
    print(f"  GPIO pins  : {pin_count}")
    print("=" * 60)


# =============================================================================
# Entry point — called when script is executed in the Script Editor
# =============================================================================
build_raspberry_pi()