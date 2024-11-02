from stl import mesh
import numpy as np
import glob

# Load the meshes from the STL files
# load all files in urrent directory
files = glob.glob("*.stl")
meshes = [mesh.Mesh.from_file(f) for f in files]

# Create a new mesh that contains all the other meshes
combined_mesh_data = np.concatenate([m.data for m in meshes])
    
combined_mesh = mesh.Mesh(combined_mesh_data)

# Save the combined mesh to a new file
combined_mesh.save('combined.stl')

