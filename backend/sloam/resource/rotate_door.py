
import open3d as o3d

# load object.stl and translate along y -1.0 m
object = o3d.io.read_triangle_mesh("door_input.stl")
# translate to 0,0,0
object.translate([-object.get_center()[0], -object.get_center()[1], -object.get_center()[2]])
# rotate 90 degrees to face up
R = object.get_rotation_matrix_from_xyz((0, 1.57, 0))
object.rotate(R, center=(0, 0, 0))

# visualize origin frame
mesh = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([object, mesh], mesh_show_wireframe=True, mesh_show_back_face=True)
# the new object using write_triangle_mesh as .stl
# compute normals first
object.compute_vertex_normals()
o3d.io.write_triangle_mesh("door.stl", object)