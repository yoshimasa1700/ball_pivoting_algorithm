import open3d as o3d
import shutil

data = o3d.data.BunnyMesh()

# for visualization
# mesh = o3d.io.read_triangle_mesh(data.path)
# o3d.visualization.draw_geometries([mesh])

shutil.copy(data.path, "data")
