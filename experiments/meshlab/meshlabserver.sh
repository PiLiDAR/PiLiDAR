sudo apt-get install meshlab  # installs MeshLab 2020.09
sudo apt-get install mesa-utils libgl1-mesa-glx

# Set environment variables for offscreen rendering
export QT_QPA_PLATFORM=offscreen
export LIBGL_ALWAYS_SOFTWARE=1


cd meshlab

# Generate mesh
meshlabserver -l output/log_mesh.txt -i input/pointcloud.ply -o output/meshed.ply -l x -s meshing.mlx

# Map texture
meshlabserver -l output/log_map.txt -p box.mlp -i output/meshed.ply -o output/textured.ply -m vn wt -s mapping.mlx
