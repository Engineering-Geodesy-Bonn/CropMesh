# open3D
import os
import open3d as o3d

# Own functions
from HelperFunctions.PointCloudProcessing import ProcessPointCloudToMesh, CalculateLeafAngles


if __name__ == '__main__':

    Path = rf'Data'
    name = 'banane_2'
    FilePath = rf'Data\{name}.txt'
    MeshPath = rf'Data\meshes\{name}.ply'
    FigurePath = rf'Data\figures'

    """unit the point cloud is measured in"""
    unit = 'mm'

    if not os.path.exists(rf'{Path}\meshes'):
        os.makedirs(rf'{Path}\meshes')
    if not os.path.exists(rf'{Path}\figures'):
        os.makedirs(rf'{Path}\figures')
    if not os.path.exists(rf'{Path}\Files'):
        os.makedirs(rf'{Path}\Files')

    pc = o3d.io.read_point_cloud(FilePath, format='xyz')

    SurfaceSize, SurfaceSizeRepaired, mesh, repairedMesh = \
        ProcessPointCloudToMesh(FigurePath, FilePath, pc, unit, PlotImage=False, SaveImage=False)

    repairedMesh.save(MeshPath)

    """Optional"""
    Zenith, ZenithMedian, Azimuth, AzimuthMedian = CalculateLeafAngles(mesh, Path, name)
