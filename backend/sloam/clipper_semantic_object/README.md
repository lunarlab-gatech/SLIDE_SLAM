## Note
The repo is forked from CLIPPER and modified to extract triangulation descriptors from point landmarks and robustly estimate point correspondences betweentwo set of points. The code is put inside the slide_slam for the convenience of debugging and implementation.

Original CLIPPER Git Repo: https://github.com/mit-acl/clipper

CLIPPER citation
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
