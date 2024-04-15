#include "motion/navigation/nav2_plugins/clear_people_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace nav2_costmap_2d
{

// ClearPeopleLayer::ClearPeopleLayer()
// {}
// // implement default destructor:
// ClearPeopleLayer::~ClearPeopleLayer()
// {}
// This method is called at the end of plugin initialization.
// It contains ROS parameter(s) declaration and initialization
// of need_recalculation_ variable.
void ClearPeopleLayer::onInitialize()
{
  declareParameter("person_frame", rclcpp::ParameterValue("person_0"));

  const auto node = node_.lock();

  if (!node) {
    throw std::runtime_error("ClearPeopleLayer::onInitialize: Failed to lock node");
  }
  if (!tf_) {
    throw std::runtime_error("ClearPeopleLayer::onInitialize: Failed to initialize tf buffer");
  }

  // Declaring ROS parameters:
  auto getString = [&](const std::string & parameter_name) {
      std::string param{};
      node->get_parameter(name_ + "." + parameter_name, param);
      return param;
    };

  person_frame_ = getString("person_frame");
  RCLCPP_INFO(logger_, "Initialized plugin clear_people_layer");

}

// The method is called to ask the plugin: which area of costmap it needs to update.
// Inside this method window bounds are re-calculated if need_recalculation_ is true
// and updated independently on its value.
void ClearPeopleLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double * min_x, double * min_y,
  double * max_x, double * max_y)
{}

// The method is called when costmap recalculation is required.
// It updates the costmap within its window bounds.
// Inside this method the costmap gradient is generated and is writing directly
// to the resulting costmap master_grid without any merging with previous layers.
void ClearPeopleLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_x, int min_y, int max_x, int max_y)
{
  if (!enabled_) {
    return;
  }

  if (min_x >= max_x || min_y >= max_y) {
    return;
  }

  try {
    person_transform_ = tf_->lookupTransform(
      "map", person_frame_,
      tf2::TimePointZero);
  } catch (std::exception & ex) {
    RCLCPP_ERROR(
      logger_, "ClearPeopleLayer::updateCosts error transforming map to %s : %s ",
      person_frame_.c_str(), ex.what());
    return;
  }
  try {
    removePerson(person_transform_, master_grid);
  } catch (std::exception & ex) {
    RCLCPP_ERROR(logger_, "%s", (std::string("Inner error: ") + ex.what()).c_str());
  }

  current_ = true;
}

void ClearPeopleLayer::removePerson(
  const geometry_msgs::msg::TransformStamped & person_transform,
  nav2_costmap_2d::Costmap2D & master_grid)
{
  // Getting the person position in the costmap grid
  unsigned int person_x, person_y;
  if (!master_grid.worldToMap(
      person_transform.transform.translation.x,
      person_transform.transform.translation.x, person_x, person_y))
  {
    RCLCPP_ERROR(
      logger_,
      "ClearPeopleLayer::removePerson error transforming person to costmap grid");
    return;
  }

  // Getting the person radius in the costmap grid
  double person_radius = 0.5;
  int person_radius_cells = std::ceil(person_radius / master_grid.getResolution());

  // Removing the person from the costmap grid
  for (int j = person_y - person_radius_cells; j <= person_y + person_radius_cells; j++) {
    for (int i = person_x - person_radius_cells; i <= person_x + person_radius_cells; i++) {
      master_grid.worldToMapEnforceBounds(i, j, i, j);
      int index = master_grid.getIndex(i, j);
      master_grid.setCost(i, j, FREE_SPACE);
    }
  }
  RCLCPP_ERROR(logger_, "ClearPeopleLayer::removePerson Person cleraed from costmap");
}

void ClearPeopleLayer::reset()
{
  current_ = false;
}

// bool ClearPeopleLayer::isBackground(uint8_t pixel) const
// {
//   bool is_obstacle =
//     pixel == LETHAL_OBSTACLE ||
//     pixel == INSCRIBED_INFLATED_OBSTACLE ||
//     (pixel == NO_INFORMATION && no_information_is_obstacle_);
//   return !is_obstacle;
// }

}  // namespace nav2_costmap_2d

// This is the macro allowing a nav2_costmap_2d::ClearPeopleLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_costmap_2d::ClearPeopleLayer, nav2_costmap_2d::Layer)
