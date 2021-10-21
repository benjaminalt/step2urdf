"""
Copyright (C) 2021 ArtiMinds Robotics GmbH
"""
import os
from argparse import ArgumentParser

import numpy as np
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.gp import gp_Vec
from OCC.Extend.DataExchange import read_step_file_with_names_colors, write_stl_file
from scipy.spatial.transform import Rotation

from utils.rescale_mesh import rescale_stl


def urdf_for_part(link_name: str, parent_link_name: str, mesh_filepath: str, pos: np.ndarray, ori: Rotation,
                  package_substitutions: dict = None):
    for dirpath in package_substitutions.keys():
        if dirpath in mesh_filepath:
            mesh_filepath = mesh_filepath.replace(dirpath, package_substitutions[dirpath])
    return f"""<link name="{link_name}">
    <visual>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_filepath}"/>
      </geometry>
    </visual>
    <collision>
      <origin rpy="0 0 0" xyz="0 0 0"/>
      <geometry>
        <mesh filename="{mesh_filepath}"/>
      </geometry>
    </collision>
</link>
<joint name="{link_name}_{parent_link_name}_joint" type="fixed">
    <origin rpy="{' '.join([str(i) for i in ori.as_euler("xyz")])}" xyz="{' '.join([str(i) for i in pos])}"/>
    <!--origin rpy="0 0 0" xyz="0 0 0"/-->
    <child link="{link_name}"/>
    <parent link="{parent_link_name}"/>
</joint>
"""


def ensure_unique_key(name, dic) -> str:
    if name not in dic.keys():
        return name
    cnt = 1
    while f"{name}_{cnt}" in dic.keys():
        cnt += 1
    return f"{name}_{cnt}"


def parse_step(step_filepath: str, mesh_output_dir: str, scaling_factor: float) -> dict:
    assembly = read_step_file_with_names_colors(step_filepath)
    parts = {}
    for shape, [name, color] in assembly.items():
        if len(name) == 0:
            continue
        pos = shape.Location().Transformation().TranslationPart()
        pos = np.array([pos.X() * scaling_factor, pos.Y() * scaling_factor, pos.Z() * scaling_factor])
        ori = shape.Location().Transformation().GetRotation()
        axis = gp_Vec()
        angle = ori.GetVectorAndAngle(axis)
        axis = [axis.X(), axis.Y(), axis.Z()]
        print(f"{name}: {pos} / {axis}, {angle}")
        mesh_filepath = os.path.join(mesh_output_dir, f"{name}.stl")
        rot = Rotation.from_quat((ori.X(), ori.Y(), ori.Z(), ori.W()))
        if not os.path.exists(mesh_filepath):
            try:
                # If I just export the shape as a mesh, the mesh will include the shape's transformation
                # I want to have each mesh have its "original" origin, and apply the transformations "externally"
                # via the URDF joints --> first apply the inverse transformation to the shape
                brep = BRepBuilderAPI_Transform(shape, shape.Location().Transformation().Inverted())
                write_stl_file(brep.Shape(), mesh_filepath, mode="binary")
                rescale_stl(mesh_filepath, scaling_factor)
            except IOError:
                continue
        unique_name = ensure_unique_key(name, parts)
        parts[unique_name] = {
            "mesh": mesh_filepath,
            "pos": pos,
            "ori": rot
        }
    return parts


def main(args):
    assert args.urdf_output_filepath.endswith(".urdf")
    if not os.path.exists(args.mesh_output_dir):
        os.makedirs(args.mesh_output_dir)

    # Parse package prefixes
    package_substitutions = {}
    if args.package_substitutions is not None:
        for subst in args.package_substitutions:
            path, package_url = subst.split("=")
            package_substitutions[path] = f"package://{package_url}"

    parts = parse_step(args.step_filepath, args.mesh_output_dir, args.scaling_factor)
    assembly_name = os.path.splitext(os.path.basename(args.step_filepath))[0]
    parent_link_name = f"{assembly_name}_root_link"
    urdf_str = f"""<?xml version="1.0" ?>
<robot name="{assembly_name}">
<link name="{parent_link_name}"/>
"""
    for part_name, part in parts.items():
        urdf_str += urdf_for_part(part_name, parent_link_name, part["mesh"], part["pos"], part["ori"],
                                  package_substitutions)
    urdf_str += "</robot>"
    with open(args.urdf_output_filepath, "w") as urdf_file:
        urdf_file.write(urdf_str)


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("step_filepath", type=str)
    parser.add_argument("urdf_output_filepath", type=str)
    parser.add_argument("mesh_output_dir", type=str)
    parser.add_argument("--package_substitutions", nargs="+")
    parser.add_argument("--scaling_factor", type=float, default=0.001)
    main(parser.parse_args())
