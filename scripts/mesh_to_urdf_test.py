import os
import glob
import argparse
from itertools import repeat
import xml.etree.cElementTree as ET
from multiprocessing import Pool
import numpy as np
import trimesh
# import parmap
# import multiprocessing


# mesh rescale prameters
GRIPPER_WIDTH = 0.08
GRIPPER_FRAC = 0.8
gripper_target = GRIPPER_WIDTH * GRIPPER_FRAC

# Parse arguments
parser = argparse.ArgumentParser(description='This script converts mesh data to Isaac Gym asset.')
parser.add_argument('--root', required=True, help='Path to mesh folder')
parser.add_argument('--target', required=True, help='Path to asset folder')
parser.add_argument('--mesh_name', required=True, help='Mesh mesh_name')
args = parser.parse_args()

mesh_root_dir = args.root
target_root_dir = args.target
obj_ext = args.mesh_name

if not os.path.exists(target_root_dir):
    os.makedirs(target_root_dir)

def indent(elem, level=0, more_sibs=False):
    ''' Add indent when making URDF file'''
    # https://stackoverflow.com/questions/749796/pretty-printing-xml-in-python
    i = "\n"
    if level:
        i += (level-1) * '  '
    num_kids = len(elem)
    if num_kids:
        if not elem.text or not elem.text.strip():
            elem.text = i + "  "
            if level:
                elem.text += '  '
        count = 0
        for kid in elem:
            indent(kid, level+1, count < num_kids - 1)
            count += 1
        if not elem.tail or not elem.tail.strip():
            elem.tail = i
            if more_sibs:
                elem.tail += '  '
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = i
            if more_sibs:
                elem.tail += '  '


def obj_to_urdf(mesh_file):
    # if target exists, skip
    target_name = os.path.basename(mesh_file).split('.')[0]  # A0, A1, ...
    # if os.path.exists(os.path.join(target_root_dir, target_name)):
        # print('overide existing file for: ', target_name)

    # print('processing: ', mesh_file)
    # Load mesh
    mesh = trimesh.load(os.path.join(mesh_file))
    if not mesh.is_watertight:
        print('\t{} is not watertight.'.format(mesh_file))
    else:
        pass

    # make directory
    os.makedirs(os.path.join(target_root_dir, target_name), exist_ok=True)

    # rescale mesh based on gripper width(EGAD paper)
    exts = mesh.bounding_box_oriented.primitive.extents
    max_dim = np.max(exts)
    scale = GRIPPER_WIDTH / max_dim
    scale = scale * 1.34
    # mesh.show()
    # mesh.apply_scale(0.001) # mm to m scale
    mesh.apply_scale(scale) # mm to m scale
    # mesh.show()

    mass = 0.050
    if mesh.volume < 0:
        mesh.invert()
        # print(mesh.convex_hull.volume, '\n\t', mesh_file, '\tmax_dim:\t',max_dim,'\tscale:\t', scale, '\n\tvolume:\t',mesh.volume, '\tconvex volume:\t', mesh.convex_hull.volume)
        # mesh.show()
    elif mesh.volume == 0:
        # mesh.show()
        pass
    # mesh.vertices -= mesh.center_mass
    if mass / 2450 < mesh.volume:
        mesh.density = mass/mesh.volume
    else:
        mesh.density = 2450
    mesh.vertices -= mesh.center_mass

    # save mesh
    mesh.export(os.path.join(target_root_dir, target_name, obj_ext))
    print('name:\t',target_name)

    # create urdf file
    urdf = ET.Element('robot', name=target_name)
    link = ET.SubElement(urdf, 'link', name=target_name)
    inertial = ET.SubElement(link, 'inertial')
    mass = ET.SubElement(inertial, 'mass', value=str(mesh.mass))
    inertia_dict = {'ixx': str(mesh.moment_inertia[0, 0]),
                    'ixy': str(mesh.moment_inertia[0, 1]),
                    'ixz': str(mesh.moment_inertia[0, 2]),
                    'iyy': str(mesh.moment_inertia[1, 1]),
                    'iyz': str(mesh.moment_inertia[1, 2]),
                    'izz': str(mesh.moment_inertia[2, 2])}
    inertia = ET.SubElement(inertial, 'inertia', inertia_dict)

    visual = ET.SubElement(link, 'visual')
    origin = ET.SubElement(visual, 'origin', xyz='0 0 0', rpy='0 0 0')
    geometry = ET.SubElement(visual, 'geometry')
    _mesh = ET.SubElement(geometry, 'mesh', filename=os.path.join(target_root_dir, target_name, obj_ext), scale='1 1 1')

    collision = ET.SubElement(link, 'collision')
    origin = ET.SubElement(collision, 'origin', xyz='0 0 0', rpy='0 0 0')
    geometry = ET.SubElement(collision, 'geometry')
    _mesh = ET.SubElement(geometry, 'mesh', filename=os.path.join(target_root_dir, target_name, obj_ext), scale='1 1 1')

    # save urdf file
    indent(urdf)
    tree = ET.ElementTree(urdf)
    with open(os.path.join(target_root_dir, target_name, target_name + '.urdf'), 'wb') as f:
        tree.write(f, encoding='utf-8', xml_declaration=True)

    # get stable poses
    # mesh.apply_scale(1000.0)
    # stable_poses, prob = mesh.compute_stable_poses(n_samples=10, sigma=0.1)
    stable_poses, prob = mesh.compute_stable_poses(center_mass=mesh.center_mass,n_samples=10, sigma=0.1)
    for i in range(len(stable_poses)):
        stable_poses[i][0:3, 3] *= 0.001

    np.save(os.path.join(target_root_dir, target_name, 'stable_poses.npy'), stable_poses)
    np.save(os.path.join(target_root_dir, target_name, 'stable_prob.npy'), prob)

    # save log as txt
    with open(os.path.join(target_root_dir, target_name, 'log.txt'), 'w') as f:
        f.write('num stable poses: {}\n'.format(len(stable_poses)))
        s = 'prob: '
        for i, p in enumerate(prob):
            s += '{:.3f}'.format(p)
            if i < len(prob) - 1:
                s += ', '
        s += '\n'
        f.write(s)
    print('\tfinish', target_name)


if __name__ == '__main__':
    # get file list
    obj_files = glob.glob(os.path.join(mesh_root_dir + '/' + obj_ext))

    obj_to_urdf(mesh_root_dir + '/' + obj_ext)