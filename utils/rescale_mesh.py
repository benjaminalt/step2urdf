from argparse import ArgumentParser

import trimesh


def rescale_stl(mesh_filepath: str, scaling_factor: float, mesh_output_path=None):
    mesh = trimesh.load_mesh(mesh_filepath)
    mesh.apply_scale(scaling_factor)
    output_path = mesh_filepath
    if mesh_output_path is not None:
        output_path = mesh_output_path
    mesh.export(output_path)


def main(args):
    rescale_stl(args.mesh_filepath, args.scaling_factor, args.mesh_output_path)


if __name__ == '__main__':
    parser = ArgumentParser()
    parser.add_argument("mesh_filepath", type=str)
    parser.add_argument("scaling_factor", type=float)
    parser.add_argument("--mesh_output_path", type=str)
    main(parser.parse_args())
