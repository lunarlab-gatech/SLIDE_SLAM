import time
import numpy as np
import open3d as o3d
from scipy.spatial.transform import Rotation
import clipperpy
import matplotlib.pyplot as plt

def read_data_from_txt(txt_file):
    '''
    the txt file should be in the following format in each line:
    label x y z
    get the x y value return a 2 by n numpy array
    '''
    data = np.loadtxt(txt_file, usecols=(0, 1, 2))
    return data.T

def visualize_data(data):
    '''
    data: 3 by n numpy array, label, x , y
    plot label of 0 as square, label of -1 as circle, label of -2 as triangle
    '''
    for i in range(data.shape[1]):
        if data[0, i] == 0:
            plt.plot(data[1, i], data[2, i], 's')
        elif data[0, i] == -1:
            plt.plot(data[1, i], data[2, i], 'o')
        elif data[0, i] == -2:
            plt.plot(data[1, i], data[2, i], '^')

def generate_DA(size_a, size_b):
    '''
    size_a: number of points in data A
    size_b: number of points in data B
    A: min(size_a, size_b) by 2 numpy array, mapping point indexed by row in data A to data B 
    '''
    DA = np.zeros((size_a*size_b, 2))
    for i in range(size_a):
        for j in range(size_b):
            DA[i*size_b+j, 0] = i
            DA[i*size_b+j, 1] = j
    return DA
    


D1 = read_data_from_txt("/home/jiuzhou/clipper_semantic_object/examples/data/robot0Map_parking.txt")
D2 = read_data_from_txt("/home/jiuzhou/clipper_semantic_object/examples/data/robot1Map_parking.txt")
print(D1.shape)
print(D2.shape)
D1_position = D1[1:3, :]
D2_position = D2[1:3, :]
A = generate_DA(D1.shape[1], D2.shape[1])


iparams = clipperpy.invariants.EuclideanDistanceParams()
iparams.sigma = 0.01
iparams.epsilon = 0.02
invariant = clipperpy.invariants.EuclideanDistance(iparams)

params = clipperpy.Params()
params.rounding = clipperpy.Rounding.DSD_HEU
clipper = clipperpy.CLIPPER(invariant, params)

t0 = time.perf_counter()
# D1 3 by n
# D2 3 by m
# A: min(m,n) by 2
clipper.score_pairwise_consistency(D1, D2, A)
t1 = time.perf_counter()
print(f"Affinity matrix creation took {t1-t0:.3f} seconds")

t0 = time.perf_counter()
clipper.solve()
t1 = time.perf_counter()

# A = clipper.get_initial_associations()
# Ain: k by 2 association
Ain = clipper.get_selected_associations()

# p = np.isin(Ain, Agt)[:,0].sum() / Ain.shape[0]
# r = np.isin(Ain, Agt)[:,0].sum() / Agt.shape[0]
# print(f"CLIPPER selected {Ain.shape[0]} inliers from {A.shape[0]} "
#       f"putative associations (precision {p:.2f}, recall {r:.2f}) in {t1-t0:.3f} s")
