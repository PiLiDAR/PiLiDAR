'''
https://pymeshlab.readthedocs.io/en/latest/

pip install pymeshlab

on Raspberry Pi, build:
git pull https://github.com/cnr-isti-vclab/PyMeshLab.git
bash PyMeshLab/scripts/Linux/0_setup_env.sh  1_build.sh  2_deploy.sh  make_wheel.sh

Screened Poisson Reconstruction: https://www.youtube.com/watch?v=fZI925P1aXw


compute_texcoord_parametrization_and_texture_from_registered_rasters:
python==3.9.18
pymeshlab==2021.10

'''
import pymeshlab

compute_poisson = True
hausdorff_color = False

legacy = False  # <= 2021.10

project_texture = False


ms = pymeshlab.MeshSet()

# Load project and pointcloud
print("loading project file")
ms.load_project('meshlab/box/box.mlp')
ms.load_new_mesh('meshlab/pointcloud.ply')

# Simplify the point cloud, apply normal smoothing and create Poisson Surface mesh 
if compute_poisson:
    print("simplifying..")
    ms.generate_simplified_point_cloud(samplenum = 10000000)
    ms.apply_normal_point_cloud_smoothing(k = 10, usedist = True)

    print("surface reconstruction started.")
    ms.generate_surface_reconstruction_screened_poisson(depth = 15, samplespernode = 4.5)
    ms.save_current_mesh('meshlab/poisson.ply')
else:
    print("loading existing poisson mesh..")
    ms.load_new_mesh('meshlab/poisson.ply')


# Apply Laplacian smoothing to reduce noise
print("apply laplacian smoothing.")
if legacy:
    ms.apply_filter('laplacian_smooth', stepsmoothnum = 4)
else:
    ms.apply_coord_laplacian_smoothing(stepsmoothnum = 4)


# 1. Compute Hausdorff distance between mesh and pointcloud
if legacy:
    ms.apply_filter('hausdorff_distance')
else:
    ms.get_hausdorff_distance()


# 2. Compute color from scalar per vertex (optional)
if hausdorff_color:
    ms.compute_color_from_scalar_per_vertex(perc = 20.0, zerosym = True)


# 3. Select faces by scalar value range with close distancee to the pointcloud
if legacy:
    ms.apply_filter('select_by_vertex_quality', maxq = 0.05, inclusive = True) 
else:
    ms.compute_selection_by_scalar_per_vertex(maxq = 0.05, inclusive = True)


# enlarge and invert to select superfluous faces
if legacy:
    ms.apply_filter('dilate_selection')
    ms.apply_filter('erode_selection')
    ms.apply_filter('invert_selection')
else:
    ms.apply_selection_dilatation()
    ms.apply_selection_erosion()
    ms.apply_selection_inverse()


# 4. Delete selected faces and vertices
if legacy:
    ms.apply_filter('delete_selected_faces')
    ms.apply_filter('delete_selected_vertices')
else:
    ms.meshing_remove_selected_faces()
    ms.meshing_remove_selected_vertices()


# Rotate the mesh from Z-up to Y-up
if legacy:
    ms.apply_filter('transform_rotate', rotaxis = 0, angle = -90.0)
else:
    ms.compute_matrix_from_rotation(rotaxis = 0, angle = -90.0)


# Save the output mesh
print("saving mesh")
ms.save_current_mesh('meshlab/meshed.ply')


# project UV coordinates and bake texture
if project_texture:
    import os
    import subprocess
    
    # Start Xvfb
    xvfb_process = subprocess.Popen(['Xvfb', ':99', '-screen', '0', '1024x768x24'])

    os.environ['QT_QPA_PLATFORM'] = 'offscreen'
    os.environ['LIBGL_ALWAYS_SOFTWARE'] = '1'
    os.environ['DISPLAY'] = ':99'
    
    pymeshlab.use_cpu_opengl()

    print("projecting UV coordinates and baking texture")
    if legacy:
        ms.apply_filter('parameterization__texturing_from_registered_rasters', texturesize = 8192, 
                                                                                texturename = 'texture.jpg', 
                                                                                colorcorrection = False, 
                                                                                usedistanceweight = False, 
                                                                                useimgborderweight = False, 
                                                                                cleanisolatedtriangles = True, 
                                                                                texturegutter = 2)
    else:
        ms.compute_texcoord_parametrization_and_texture_from_registered_rasters(texturesize = 8192, 
                                                                                texturename = 'texture.jpg', 
                                                                                colorcorrection = False, 
                                                                                usedistanceweight = False, 
                                                                                useimgborderweight = False, 
                                                                                cleanisolatedtriangles = True, 
                                                                                texturegutter = 2)

    ms.save_current_mesh('meshlab/textured.ply')

    # Stop Xvfb
    xvfb_process.terminate()
    xvfb_process.wait()
