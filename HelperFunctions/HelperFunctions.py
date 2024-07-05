#!/usr/bin/python3
# -*- coding: utf-8 -*-

# General
import os
import numpy as np

# open3D
import open3d as o3d


def calculateBallPivotingRadius(PointCloud, scalingFactor: int):
    """
    Takes simplified (no outlier, no noise, down sampled) PointCloud and return list of
    Ball-Pivoting-Radius for Mesh calculation

    :param PointCloud: open3d PointCloud, simplified PointCloud
    :param scalingFactor: factor to scale the calculated radius

    :return: list of Ball-Pivoting-Radius for Mesh calculation
    """

    """Calculate average distance between neighbouring points"""
    distances = PointCloud.compute_nearest_neighbor_distance()
    avg_dist = np.mean(distances)
    print('Average:', avg_dist)

    """Calculating Ball-Pivoting-Radius"""
    radius = abs(1.25 * (avg_dist / 2))

    """List of scaled radii to make sure to cover all points of PointCloud"""
    radii = [(1 / scalingFactor) * radius, radius, scalingFactor * radius]
    print('list of radii:', radii)

    return radii


def PointCloud_Visualization(PointCloud, path: str, filename: str, SaveImage: bool):

    """
    Takes simplified PointCloud with Normals and saves it as image

    :param PointCloud: open3d PointCloud, simplified PointCloud
    :param path: sting, path to save figure
    :param filename: string, path to TextFile
    :param SaveImage: True --> Images are saved/ False --> Images are not saved

    :return Image of PointCloud
    """

    """create filename to save image"""
    name = '\\' + os.path.splitext(os.path.basename(filename))[0] + f'_Cloud_.png'
    filename = path + name
    if SaveImage:
        print('Image saved as:', filename)
    else:
        print('Image not saved')

    """Set Color for Points"""
    PointCloud.paint_uniform_color([0.0, 0.0, 0.0])

    """Save simplified PointCloud with Normals as png or Plot as Image"""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f'PointCloud with Normals')
    vis.add_geometry(PointCloud)
    vis.get_render_option().point_show_normal = True
    vis.update_geometry(PointCloud)
    vis.poll_events()
    vis.update_renderer()
    vis.run()
    if SaveImage:
        vis.capture_screen_image(filename, True)
        vis.destroy_window()
    else:
        vis.destroy_window()


def Mesh_Visualization(PointCloud, Mesh, path: str, filename: str, SaveImage: bool):

    """
    Takes simplified PointCloud with Normals and saves it as image

    :param PointCloud: open3d PointCloud, simplified PointCloud
    :param Mesh: open3d Mesh, reconstructed Mesh
    :param path: path to save figure
    :param filename: path to TextFile
    :param SaveImage: True -> save Image, False -> do not save image

    :return Image of PointCloud
    """

    """create filename to save image"""
    name = '\\' + os.path.splitext(os.path.basename(filename))[0] + f'_single_Mesh.png'
    filename = path + name
    if SaveImage:
        print('Image saved as:', filename)
    else:
        print('Image not saved')

    """Set Color for Points"""
    PointCloud.paint_uniform_color([0.0, 0.0, 0.0])
    Mesh.paint_uniform_color([0.0, 1.0, 0.0])

    """Save simplified PointCloud with Normals as png or Plot as Image"""
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=f'Mesh')
    vis.add_geometry(PointCloud)
    vis.add_geometry(Mesh)
    vis.update_geometry(PointCloud)
    vis.poll_events()
    vis.update_renderer()
    vis.run()
    if SaveImage:
        vis.capture_screen_image(filename, True)
        vis.destroy_window()
    else:
        vis.destroy_window()


def CalculateTriangleVertexNormals(Mesh):

    """
    Function to return Triangle and vertices and to calculate their normals
    :param Mesh: triangulated Mesh (using HortControl)

    :return: xyz coordinate of Vertices, Vertex Normals, Indices of points creating one triangle, their Normals
    """

    '''Get triangles and vertices of mesh'''
    Vertices = np.asarray(Mesh.vertices)
    Triangles = np.asarray(Mesh.triangles)

    '''Compute Vertex Normals'''
    Mesh.vertex_normals = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    Mesh.compute_vertex_normals()
    VertexNormals = np.asarray(Mesh.vertex_normals)

    '''Compute Triangle Normals'''
    Mesh.triangle_normals = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    Mesh.compute_triangle_normals()
    TriangleNormals = np.asarray(Mesh.triangle_normals)

    return Vertices, VertexNormals, Triangles, TriangleNormals


def CalculateTriangleSurface(Mesh):

    """
    Function to calculate the area of each triangle of the mesh
    :param Mesh: triangulated mesh
    :return: area for each triangle saved as an list
    """

    vert, vert_normals, tria, tria_normals = CalculateTriangleVertexNormals(Mesh)
    length = np.shape(tria)

    area = []
    for i in range(length[0]):
        vector1 = np.subtract(vert[tria[i][1]], vert[tria[i][2]])
        vector2 = np.subtract(vert[tria[i][2]], vert[tria[i][0]])
        CrossProduct = np.cross(vector1, vector2)
        s = 0.5 * np.sqrt(CrossProduct.dot(CrossProduct))
        area.append(s)

    return area
