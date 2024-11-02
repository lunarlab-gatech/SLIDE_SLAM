
import open3d as o3d

# fname = "trash_can"
# fname = "cabinet"
fname = "robot"
# fname = "door"
# load object.stl and translate along y -1.0 m
object = o3d.io.read_triangle_mesh(fname+"_input.stl")
# translate to 0,0,0
object = object.translate([-object.get_center()[0], -object.get_center()[1], -object.get_center()[2]])
# scale
# object = object.scale(5, center=(0, 0, 0))
# scale to 0.25
# object = object.scale(0.25, center=(0, 0, 0))
# scale to 0.6
object = object.scale(0.5, center=(0, 0, 0))

# visualize origin frame
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([object, mesh], mesh_show_wireframe=True, mesh_show_back_face=True)
# the new object using write_triangle_mesh as .stl
# compute normals first
object.compute_vertex_normals()
o3d.io.write_triangle_mesh(fname+".stl", object)