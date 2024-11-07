## Note
The repo is forked from CLIPPER and modified to extract triangulation descriptors from point landmarks and robustly estimate point correspondences betweentwo set of points. 

Original CLIPPER Git Repo: https://github.com/mit-acl/clipper
```
@inproceedings{lusk2021clipper,
  title={{CLIPPER}: A graph-theoretic framework for robust data association},
  author={Lusk, Parker C and Fathian, Kaveh and How, Jonathan P},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={13828--13834},
  year={2021},
  organization={IEEE}
}
```

#### Dependencies for clipper and triangulation
Eigen3, MKL, Boost, Opencv, qhull, pmc, scs

#### How to integrate into sloam
```
cd <path to generic_sloam>
cd sloam
git clone https://github.com/RollingOat/clipper_semantic_object.git
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```
