# Level-Sets
Project for "Computational Geometry &amp; 3D Modeling Applications" class

## Tasks
1. Estimate the differential coordinates of the mesh
2. Perform "Taubin smoothing" and "Taubin inflation"
3. Generate the signed distance field (SDF) of the mesh surface
4. Generate the vector force/normal field (VFF)
5. Perform collision detection and estimate the direction and magnitude of the collision response force
6. Neural approach to extract the SDF using the [SIREN](https://github.com/vsitzmann/siren) and perform collision detection using that SDF

## Approach for generating the SDF
The SDF was implemend as 3D-Matrix and every element of it contains the distance from the nearest vertex. To perform this task very efficiently, KD-Trees is utilised.

## Results 
The SDFs plotted with the help of Matlab <br />
![armadilo_SDF](Code/SDF_plot/Demos/armadillo.gif)
![unicorn_SDF](Code/SDF_plot/Demos/unicorn.gif) <br />
More [here](documentation/ΑΝΑΦΟΡΑ.pdf)

## Dependencies
VVR_Framework (created by [VVR](https://www.vvr.ece.upatras.gr/))
