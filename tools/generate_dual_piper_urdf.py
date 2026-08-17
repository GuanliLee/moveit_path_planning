#!/usr/bin/env python3
"""Generate the dual-Piper URDF used by path_planning_server.

This is an offline build tool. It only reads the single-arm Piper URDF,
duplicates it with ``left_`` and ``right_`` prefixes, and writes the combined
model. It does not start ROS nodes, RViz, or publish control commands.
"""

import argparse
import copy
import xml.etree.ElementTree as ET
from pathlib import Path


REPOSITORY_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_MODEL = (
    REPOSITORY_ROOT
    / "src"
    / "piper_ros"
    / "src"
    / "piper_description"
    / "urdf"
    / "piper_description.urdf"
)
DEFAULT_OUTPUT = (
    REPOSITORY_ROOT
    / "src"
    / "path_planning_server"
    / "generated"
    / "dual_piper.urdf"
)
WORLD_LINK = "rviz_piper_world"
WRIST_CAMERA_STAND_MESH = (
    "package://path_planning_server/"
    "meshes/wrist_camera/d435_camera_stand.dae"
)
WRIST_D435I_MESH = (
    "package://path_planning_server/meshes/wrist_camera/d435.dae"
)

# The Aloha D435 model uses a different link6 coordinate convention. Mesh
# registration between its gripper body and piper_description/gripper_base.STL
# gives:
#
#   p_aloha = Rz(-pi/2) * p_piper + [0, 0, 0.004]
#
# Combining that frame conversion with Aloha's link6 -> camera-stand transform
# [0, 0.013, 0.03] yields this pose in the original Piper link6 frame.
WRIST_CAMERA_STAND_XYZ = (-0.013, 0.0, 0.026)
WRIST_CAMERA_STAND_RPY = (0.0, 0.0, 1.5707963267948966)

# Aloha's assembled URDF places the D435 mesh using an older mesh-local frame.
# Keep that proven final body pose on the stand, but express the mounting joint
# in the official D435i frame used below. This is the frame conversion:
#
#   T_stand_mount = T_aloha_body * inverse(T_d435i_mount_body)
#
# It prevents the 17.5 mm / 120 degree displacement that results from combining
# Aloha's original mounting joint with the current D435i mesh convention.
WRIST_D435I_MOUNT_XYZ = (
    -0.0301,
    0.0475316082292132,
    -0.00359836534200768,
)
WRIST_D435I_MOUNT_RPY = (
    0.0,
    -1.0507963267948965,
    -1.5707963267948966,
)


def parse_xyz(value: str) -> tuple[float, float, float]:
    """Parse an XYZ argument containing exactly three numbers."""
    try:
        coordinates = tuple(float(item) for item in value.split())
    except ValueError as exc:
        raise argparse.ArgumentTypeError(
            f"invalid XYZ value {value!r}: all coordinates must be numbers"
        ) from exc
    if len(coordinates) != 3:
        raise argparse.ArgumentTypeError(
            f"invalid XYZ value {value!r}: expected exactly three numbers"
        )
    return coordinates


def local_name(tag: str) -> str:
    """Return an XML tag name without an optional namespace."""
    return tag.rsplit("}", 1)[-1]


def find_root_link(robot: ET.Element) -> str:
    """Find the link that is not the child of any joint."""
    link_names = {
        element.attrib["name"]
        for element in robot
        if local_name(element.tag) == "link" and "name" in element.attrib
    }
    child_links = set()
    for joint in robot:
        if local_name(joint.tag) != "joint":
            continue
        for element in joint:
            if local_name(element.tag) == "child" and "link" in element.attrib:
                child_links.add(element.attrib["link"])

    roots = sorted(link_names - child_links)
    if roots:
        return roots[0]
    if link_names:
        return sorted(link_names)[0]
    raise ValueError("input URDF contains no links")


def prefixed_copy(element: ET.Element, prefix: str) -> ET.Element:
    """Copy a link or joint and prefix all link/joint references."""
    result = copy.deepcopy(element)

    # The upstream gripper links have a negative Iyy. Dynamics are not used by
    # this MoveIt visualization model, so retain the behavior of the model that
    # originally generated dual_piper.urdf and omit those two inertial blocks.
    if (
        local_name(result.tag) == "link"
        and result.attrib.get("name") in ("link7", "link8")
    ):
        for child in list(result):
            if local_name(child.tag) == "inertial":
                result.remove(child)

    for child in result.iter():
        tag = local_name(child.tag)
        if tag in ("link", "joint") and "name" in child.attrib:
            child.attrib["name"] = f"{prefix}_{child.attrib['name']}"
        elif tag in ("parent", "child") and "link" in child.attrib:
            child.attrib["link"] = f"{prefix}_{child.attrib['link']}"
        elif tag == "mimic" and "joint" in child.attrib:
            child.attrib["joint"] = f"{prefix}_{child.attrib['joint']}"
    return result


def add_base_joint(
    robot: ET.Element,
    side: str,
    child_link: str,
    origin: tuple[float, float, float],
) -> None:
    """Attach one arm base to the shared world link."""
    joint = ET.SubElement(
        robot,
        "joint",
        {"name": f"rviz_piper_world_to_{side}", "type": "fixed"},
    )
    ET.SubElement(joint, "parent", {"link": WORLD_LINK})
    ET.SubElement(joint, "child", {"link": child_link})
    ET.SubElement(
        joint,
        "origin",
        {"xyz": f"{origin[0]} {origin[1]} {origin[2]}", "rpy": "0 0 0"},
    )


def vector_text(values: tuple[float, float, float]) -> str:
    """Format an XYZ or RPY vector for a URDF attribute."""
    return " ".join(f"{value:.15g}" for value in values)


def add_fixed_joint(
    robot: ET.Element,
    name: str,
    parent: str,
    child: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """Append a fixed joint with an explicit origin."""
    joint = ET.SubElement(robot, "joint", {"name": name, "type": "fixed"})
    ET.SubElement(joint, "origin", {
        "xyz": vector_text(xyz),
        "rpy": vector_text(rpy),
    })
    ET.SubElement(joint, "parent", {"link": parent})
    ET.SubElement(joint, "child", {"link": child})


def add_mesh_geometry(
    link: ET.Element,
    element_name: str,
    mesh_uri: str,
    xyz: tuple[float, float, float],
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> None:
    """Append visual or collision geometry backed by a mesh."""
    element = ET.SubElement(link, element_name)
    ET.SubElement(element, "origin", {
        "xyz": vector_text(xyz),
        "rpy": vector_text(rpy),
    })
    geometry = ET.SubElement(element, "geometry")
    ET.SubElement(geometry, "mesh", {"filename": mesh_uri})


def add_wrist_camera(robot: ET.Element, side: str) -> None:
    """Attach the Aloha wrist D435i assembly without changing the Piper arm."""
    stand_link_name = f"{side}_camera_stand"
    camera_mount_link_name = f"{side}_d435i_camera_link"
    bottom_screw_link_name = f"{side}_camera_bottom_screw_frame"
    camera_link_name = f"{side}_camera_link"

    stand_link = ET.SubElement(robot, "link", {"name": stand_link_name})
    add_mesh_geometry(
        stand_link,
        "visual",
        WRIST_CAMERA_STAND_MESH,
        (-0.0025, 0.0, 0.0),
    )
    add_mesh_geometry(
        stand_link,
        "collision",
        WRIST_CAMERA_STAND_MESH,
        (-0.0025, 0.0, 0.0),
    )
    add_fixed_joint(
        robot,
        f"{side}_camera_stand_joint",
        f"{side}_link6",
        stand_link_name,
        WRIST_CAMERA_STAND_XYZ,
        WRIST_CAMERA_STAND_RPY,
    )

    ET.SubElement(robot, "link", {"name": camera_mount_link_name})
    add_fixed_joint(
        robot,
        f"{side}_d435i_camera_joint",
        stand_link_name,
        camera_mount_link_name,
        WRIST_D435I_MOUNT_XYZ,
        WRIST_D435I_MOUNT_RPY,
    )

    ET.SubElement(robot, "link", {"name": bottom_screw_link_name})
    add_fixed_joint(
        robot,
        f"{side}_camera_joint",
        camera_mount_link_name,
        bottom_screw_link_name,
        (0.0, 0.0, 0.0),
    )

    camera_link = ET.SubElement(robot, "link", {"name": camera_link_name})
    add_mesh_geometry(
        camera_link,
        "visual",
        WRIST_D435I_MESH,
        (0.0043, -0.0175, 0.0),
        (1.5707963267948966, 0.0, 1.5707963267948966),
    )
    collision = ET.SubElement(camera_link, "collision")
    ET.SubElement(collision, "origin", {
        "xyz": "-0.0085 -0.0175 0",
        "rpy": "0 0 0",
    })
    collision_geometry = ET.SubElement(collision, "geometry")
    ET.SubElement(collision_geometry, "box", {
        "size": "0.02505 0.09 0.025",
    })
    inertial = ET.SubElement(camera_link, "inertial")
    ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(inertial, "mass", {"value": "0.072"})
    ET.SubElement(inertial, "inertia", {
        "ixx": "0.003881243",
        "ixy": "0.0",
        "ixz": "0.0",
        "iyy": "0.000498940",
        "iyz": "0.0",
        "izz": "0.003879257",
    })
    add_fixed_joint(
        robot,
        f"{side}_camera_link_joint",
        bottom_screw_link_name,
        camera_link_name,
        (0.0106, 0.0175, 0.0125),
    )


def generate_dual_urdf(
    source_xml: str,
    left_origin: tuple[float, float, float],
    right_origin: tuple[float, float, float],
) -> str:
    """Build the dual-arm URDF while preserving package:// mesh URIs."""
    source_robot = ET.fromstring(source_xml)
    source_root_link = find_root_link(source_robot)

    robot = ET.Element("robot", {"name": "agilex_dual_piper_rviz"})
    ET.SubElement(robot, "link", {"name": WORLD_LINK})

    added_materials: set[str | None] = set()
    origins = {"left": left_origin, "right": right_origin}
    for side in ("left", "right"):
        for element in source_robot:
            tag = local_name(element.tag)
            if tag in ("link", "joint"):
                robot.append(prefixed_copy(element, side))
            elif tag == "material":
                material_name = element.attrib.get("name")
                if material_name not in added_materials:
                    added_materials.add(material_name)
                    robot.append(copy.deepcopy(element))

        add_base_joint(
            robot,
            side,
            child_link=f"{side}_{source_root_link}",
            origin=origins[side],
        )
        add_wrist_camera(robot, side)

    return ET.tostring(robot, encoding="unicode")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate the path_planning_server dual-Piper URDF offline. "
            "package://piper_description mesh URIs are preserved."
        )
    )
    parser.add_argument(
        "--model",
        type=Path,
        default=DEFAULT_MODEL,
        metavar="PATH",
        help=f"single-arm Piper URDF (default: {DEFAULT_MODEL})",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        metavar="PATH",
        help=f"generated dual-arm URDF (default: {DEFAULT_OUTPUT})",
    )
    parser.add_argument(
        "--left-origin",
        type=parse_xyz,
        default=parse_xyz("0 0.3 0"),
        metavar='"X Y Z"',
        help='left base origin in the shared world frame (default: "0 0.3 0")',
    )
    parser.add_argument(
        "--right-origin",
        type=parse_xyz,
        default=parse_xyz("0 -0.3 0"),
        metavar='"X Y Z"',
        help='right base origin in the shared world frame (default: "0 -0.3 0")',
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    model_path = args.model.expanduser().resolve()
    output_path = args.output.expanduser().resolve()

    if not model_path.is_file():
        raise FileNotFoundError(f"single-arm Piper URDF not found: {model_path}")

    generated_xml = generate_dual_urdf(
        model_path.read_text(encoding="utf-8"),
        args.left_origin,
        args.right_origin,
    )
    ET.fromstring(generated_xml)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(generated_xml + "\n", encoding="utf-8")
    print(f"Generated dual-Piper URDF: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
