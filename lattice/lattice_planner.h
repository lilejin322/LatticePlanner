#include <memory>
#include <string>

#include "modules/planning/planning_base/proto/planning_config.pb.h"

#include "cyber/plugin_manager/plugin_manager.h"
#include "modules/common/status/status.h"
#include "modules/planning/planning_base/common/frame.h"
#include "modules/planning/planning_base/common/reference_line_info.h"
#include "modules/planning/planning_interface_base/planner_base/planner.h"

namespace apollo {
namespace planning {

class LatticePlanner : public PlannerWithReferenceLine {
 public:


  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status Plan(const common::TrajectoryPoint& planning_init_point,
                      Frame* frame,
                      ADCTrajectory* ptr_computed_trajectory) override;

  /**
   * @brief Override function Plan in parent class Planner.
   * @param planning_init_point The trajectory point where planning starts.
   * @param frame Current planning frame.
   * @param reference_line_info The computed reference line.
   * @return OK if planning succeeds; error otherwise.
   */
  common::Status PlanOnReferenceLine(
      const common::TrajectoryPoint& planning_init_point, Frame* frame,
      ReferenceLineInfo* reference_line_info) override;
};

CYBER_PLUGIN_MANAGER_REGISTER_PLUGIN(apollo::planning::LatticePlanner,
                                     PlannerWithReferenceLine)

}  // namespace planning
}  // namespace apollo
