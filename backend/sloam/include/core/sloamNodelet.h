/**
* This file is part of SlideSLAM
*
* Copyright (C) 2024 Guilherme Nardari, Xu Liu, Jiuzhou Lei, Ankit Prabhu, Yuezhan Tao
*
* TODO: License information
*
*/

#pragma once

#include <nodelet/nodelet.h>
#include <sloamNode.h>

namespace sloam {
class SLOAMNodelet : public nodelet::Nodelet {
 public:
  SLOAMNodelet() {}
  ~SLOAMNodelet() {}
  virtual void onInit();

 private:
  SLOAMNode::Ptr sloamNode;
};
}  // namespace sloam
