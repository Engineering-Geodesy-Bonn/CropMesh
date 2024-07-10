# CropMesh - An automated Surface Reconstruction Pipeline for Plant Phenotyping

Plant phenotyping plays a crucial role in crop science and plant breeding. 
The accurate assessment of plant traits is essential for understanding plant growth, improving crop yields, and developing resilient plant varieties. 
In recent years, 3D sensing systems such as laser scanners have gained popularity due to their ability to capture detailed structural plant parameters that are challenging to obtain using traditional spectral sensors.

## Project Description
This repository provides tools and methods for processing and analyzing 3D point clouds generated from different laser scanning systems, ranging from high-presicion scanners used under laboratory conditions to mobile mapping systems used on the field.
Unlike images, point clouds are unstructured and require several pre-processing steps to extract precise information, handle noise, and address missing data. 
This project focuses on converting raw point cloud data into useful mesh-based surface representations through triangulation techniques. 
This mesh-based surface then can be used to extract phenotypic traits like leaf or plant area and leaf inclination angles.

## Features
**Point Cloud Pre-processing:** Methods for cleaning and denoising raw point cloud data to improve the quality of the final output.
<br>
**Triangulation:** Tools for generating mesh-based surface representations from processed point clouds using triangulation.
<br>
**Data Handling:** Techniques for managing and filling in missing points in the point cloud data.
<br>
**Visualization:** Tools for visualizing both the raw and processed point clouds and meshes to facilitate analysis.

![Workflow](https://github.com/Engineering-Geodesy-Bonn/CropMesh/assets/28858970/b9fb3d64-f9ad-4b3a-b313-dbff54697836)
