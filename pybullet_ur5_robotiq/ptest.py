import trimesh
import numpy as np

# trimesh.points.PointCloud(np.random.rand(10, 3)).scene().show()
mesh = trimesh.points.PointCloud(trimesh.primitives.Sphere().sample(5000)).scene().show()