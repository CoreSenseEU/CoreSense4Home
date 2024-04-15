#ifndef MOTION_NAVIGATION_NAV2_PLUGINS_CLEAR_PEOPLE_LAYER_HPP_
#define MOTION_NAVIGATION_NAV2_PLUGINS_CLEAR_PEOPLE_LAYER_HPP_

#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

namespace nav2_costmap_2d
{

class ClearPeopleLayer : public Layer
{
public:
  ClearPeopleLayer() = default;
  ~ClearPeopleLayer() = default;


  void reset() override;

  void updateBounds(
    double robot_x, double robot_y, double robot_yaw, double * min_x, double * min_y,
    double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_x, int min_y, int max_x, int max_y) override;

  bool isClearable() {return false;}

protected:
  virtual void onInitialize() override;

private:
  void removePerson(
    const geometry_msgs::msg::TransformStamped & person_transform,
    nav2_costmap_2d::Costmap2D & master_grid);
  double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
  std::string person_frame_{};
  geometry_msgs::msg::TransformStamped person_transform_;

  // Indicates that the entire gradient should be recalculated next time.
  bool need_recalculation_;

  // Size of gradient in cells
  int GRADIENT_SIZE = 20;
  // Step of increasing cost per one cell in gradient
  int GRADIENT_FACTOR = 10;
};

}  // namespace nav2_costmap_2d

#endif  // MOTION_NAVIGATION_NAV2_PLUGINS_CLEAR_PEOPLE_LAYER_HPP_
