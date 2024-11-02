/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao, Guilherme Nardari
*
* TODO: License information
*
*/

#pragma once

#include <cube.h>
#include <definitions.h>

#include <set>

class CubeMapManager {
 public:
  explicit CubeMapManager();
  // get map for visualization
  std::vector<Cube> getFinalMap(const int &num_min_observations = 3);
  const std::vector<Cube> &getConstMap() const;
  std::vector<Cube> &getRawMap();
  std::map<int, int> getMatchesMap() const;
  void getSubmap(const SE3 &pose, std::vector<Cube> &submap);

  void updateMap(const SE3 &pose, std::vector<Cube> &obs_tms,
                 const std::vector<int> &matches, const int &robotID);
  void getkeyPoseSubmap(const SE3 &pose, std::vector<Cube> &submap,
                        double submap_radius, const int robotID);

 private:
  // landmarks_ variable only records the 3D positions of semantic landmarks for
  // purposes such as to get a sub map
  CloudT::Ptr cube_landmarks_;
  std::vector<size_t> cube_hits_;
  std::vector<Cube> cube_models_;
  std::map<int, int> cubeMatchesMap_;
};