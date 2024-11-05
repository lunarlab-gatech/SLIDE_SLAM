## Note
The repo is forked from CLIPPER and modified to extract triangulation descriptors from point landmarks and robustly estimate point correspondences betweentwo set of points. 


#### Dependencies for clipper and triangulation
Eigen3, MKL, Boost, Opencv, qhull, pmc, scs

#### How to integrate into sloam
```
cd <path to generic_sloam>
cd sloam
git clone https://github.com/RollingOat/clipper_semantic_object.git
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```