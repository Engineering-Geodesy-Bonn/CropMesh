#!/usr/bin/python3
# -*- coding: utf-8 -*-

# General
import os
import numpy as np
import math

# open3D
import open3d as o3d

# PyVista, PyMeshFix and TriMesh
import pyvista as pv
import pymeshfix as pmf

# Plotting
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter

# Helper Functions
from HelperFunctions.HelperFunctions import calculateBallPivotingRadius, PointCloud_Visualization, Mesh_Visualization, \
    CalculateTriangleSurface, CalculateTriangleVertexNormals


def ProcessPointCloudToMesh(path: str, filename: str, PointCloud, unit, PlotImage: bool = False,
                            SaveImage: bool = False):

    """
    Takes separated PointCloud, simplifies the PointCloud and creates a Mesh using the Ball-Pivoting Algorithm.
    Remaining holes in Mesh are closed via PyMeshFix (PyVista based)

    :param path: path for saving image
    :param filename: name for image
    :param PointCloud: open3D PointCloud, input PointClout
    :param unit: unit of the point cloud
    :param PlotImage: Default=False --> Images are not plotted
    :param SaveImage: Default=False --> Images are not saved; True --> Images are saved

    :return: surface size: (with/without holes) --> float, reconstructed mesh (with/without holes) --> PyVista PolyData
    """

    """Set Parameter for Mesh Generation"""
    '''Outlier Removal'''
    nb_neighbors = 10  # number of neighbours for outlier removal
    std = 1.0  # standard deviation

    '''Down Sampling'''
    # voxel size v in mm
    if unit == 'mm':
        DownSamplingNumber = 4
    elif unit == 'cm':
        DownSamplingNumber = 0.4
    elif unit == 'dm':
        DownSamplingNumber = 0.04
    elif unit == 'm':
        DownSamplingNumber = 0.004

    '''BallPivoting Radius Scale'''
    scalingFactor = 5  # scaling factor for Ball-Pivoting-Radius (see description calculateBallPivotingRadius)

    '''Smoothing Filter (if applied)'''
    SmoothingFilter = 'none'  # chosen filter; (POSSIBLE: none, taubin)
    noi = 10  # number of iterations for smoothing
    lam = 0.5  # lambda value for Laplacian Filter

    """Create Normals for Point Cloud, orientate them in the same direction"""
    print('Original:', PointCloud)
    PointCloud.normals = o3d.utility.Vector3dVector(np.zeros((1, 3)))
    PointCloud.estimate_normals()
    # PointCloud.orient_normals_consistent_tangent_plane(k=30)
    PointCloud.orient_normals_to_align_with_direction()

    """Remove Outlier"""
    cloud_simplified, ind = PointCloud.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std)

    """Down sampling of point Cloud"""
    downSampled_pcd = cloud_simplified.voxel_down_sample(DownSamplingNumber)
    # downSampled_pcd = PointCloud.voxel_down_sample(DownSamplingNumber)
    print('Subsampled:', downSampled_pcd)

    """Calculate Normals for simplified PointCloud, orientate them in the same direction (important for Mesh 
    Reconstruction)"""

    print('reconstruction')
    downSampled_pcd.estimate_normals()
    downSampled_pcd.orient_normals_consistent_tangent_plane(k=30)
    # downSampled_pcd.orient_normals_to_align_with_direction()

    """Surface Reconstruction using Ball-Pivoting. IMPORTANT: choosing the right Radius of the Ball -> slightly larger 
    than average distance between the points"""

    radii = calculateBallPivotingRadius(downSampled_pcd, scalingFactor=scalingFactor)
    reconstructedMesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(downSampled_pcd,
                                                                                        o3d.utility.DoubleVector(radii))
    reconstructedMesh.remove_duplicated_triangles()
    reconstructedMesh.remove_duplicated_vertices()
    reconstructedMesh.remove_non_manifold_edges()
    holes = reconstructedMesh.is_watertight()
    print(holes)

    """Smooth reconstructed Mesh (with taubin method) and define name for saving screenshot of reconstructed 
    and repaired Mesh"""

    if SmoothingFilter == 'taubin':
        name = path + '\\' + os.path.splitext(os.path.basename(filename))[0] + f'_Mesh_' \
                                                                               f'{noi}_{lam}_{-lam}.png'
        smoothedMesh = reconstructedMesh.filter_smooth_taubin(noi, lam, -(lam+0.05))

    else:
        name = path + '\\' + os.path.splitext(os.path.basename(filename))[0] + f'_Mesh.png'
        smoothedMesh = reconstructedMesh

    """Filling holes in Mesh using PyVista and PyMeshFix"""

    v = np.asarray(smoothedMesh.vertices)
    f2 = np.array(smoothedMesh.triangles)
    f = np.c_[np.full(len(f2), 3), f2]
    reconstructedMesh2 = pv.PolyData(v, f)
    meshFix = pmf.MeshFix(reconstructedMesh2)
    meshFix.extract_holes()

    """Create Triangular Mesh"""

    tin = pmf.PyTMesh()
    tin.load_array(v, f2)
    tin.fill_small_boundaries(nbe=50, refine=False)
    vert, faces = tin.return_arrays()
    triangles = np.c_[np.full(len(faces), 3), faces]
    repairedMesh = pv.PolyData(vert, triangles)

    """Plotting"""

    if PlotImage:
        """Plot Process from Point Cloud to Mesh"""
        PointCloud_Visualization(PointCloud, path, filename, SaveImage)
        PointCloud_Visualization(downSampled_pcd, path, filename, SaveImage)
        Mesh_Visualization(downSampled_pcd, reconstructedMesh, path, filename, SaveImage)
        if SmoothingFilter == 'taubin':
            Mesh_Visualization(downSampled_pcd, smoothedMesh, path, filename, SaveImage)
        else:
            print('No Smoothing applied')

        """Plot repaired surface"""

        plotter = pv.Plotter(shape=(1, 1), window_size=[1920, 1080])
        plotter.set_background(color='white')
        plotter.add_mesh(repairedMesh, show_edges=True)
        if SaveImage:
            plotter.show(auto_close=False)
            plotter.screenshot(name)
        else:
            plotter.show(auto_close=False)
    else:
        print('Plotting not desired')

    """Calculate Surface Area before and after closing remaining holes in Mesh"""

    print('Surface Area before closing holes:', reconstructedMesh2.area, unit + '^2')
    print('Surface Area after closing holes:', repairedMesh.area, unit + '^2')

    return reconstructedMesh2.area, repairedMesh.area, reconstructedMesh, repairedMesh


def CalculateLeafAngles(Mesh, Path, name):

    """
    Calculates the angles between a reference vector and the normal vectors of the triangles
    :param Mesh: triangulated Mesh with Normals
    :param Path: Pathname to save figure
    :param name: Name of Plot

    :return: list of angles
    """

    angles_z = []
    angles_a = []
    normalization = []
    triangle_area = []

    vert, vert_normals, tria, tria_normals = CalculateTriangleVertexNormals(Mesh)
    TriangleArea = CalculateTriangleSurface(Mesh)
    Area = np.sum(TriangleArea)
    length = np.shape(tria_normals)

    for i in range(length[0]):
        triangle_area.append(TriangleArea[i])
        norm = TriangleArea[i] / Area
        angle_zenith = abs(math.degrees(math.acos(tria_normals[i][2])))  # zenith angle z component of normal vector
        angle_azimuth = math.degrees(math.atan2(tria_normals[i][1], tria_normals[i][0])) % 360

        normalization.append(norm)
        angles_z.append(angle_zenith)
        angles_a.append(angle_azimuth)

    median_z = np.median(angles_z)
    median_a = np.median(angles_a)

    results = pd.DataFrame(columns=['zenith', 'azimuth', 'triangle_area', 'normalization_factor'])
    results['zenith'] = angles_z
    results['azimuth'] = angles_a
    results['triangle_area'] = triangle_area
    results['normalization_factor'] = normalization

    if not os.path.exists(rf'{Path}\Files'):
        os.makedirs(rf'{Path}\Files')

    results.to_csv(rf'{Path}\Files\{name}.csv', index=False )

    return angles_z, median_z, angles_a, median_a
    
