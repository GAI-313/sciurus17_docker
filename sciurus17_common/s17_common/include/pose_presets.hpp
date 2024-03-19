#ifndef POSE_PRESETS_HPP_
#define POSE_PRESETS_HPP_

#include "geometry_msgs/msg/pose.hpp"

namespace pose_presets
{
// Pose型の位置姿勢を作成
geometry_msgs::msg::Pose generate_pose(
  const double x, const double y, const double z,
  const double roll, const double pitch, const double yaw);
// 右グリッパを下に向ける姿勢を作成
geometry_msgs::msg::Pose right_arm_downward(const double x, const double y, const double z);
// 左グリッパを下に向ける姿勢を作成
geometry_msgs::msg::Pose left_arm_downward(const double x, const double y, const double z);
}  // namespace pose_presets

#endif  // POSE_PRESETS_HPP_
