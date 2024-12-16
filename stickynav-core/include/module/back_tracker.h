#ifndef STICKYNAV_MODULE_BACK_TRACKER_H_
#define STICKYNAV_MODULE_BACK_TRACKER_H_

#include <data/trajectory_segment.h>
#include <planner_base.h>

namespace stickynav_planning
{

    class BasePlanner;

    // Base class for backtrackers to provide uniform interface with other classes
    // A backtracker's responsibility is to handle the states where no new
    // trajectories were found for execution.
    class BackTracker : public Module
    {
    public:
        explicit BackTracker(BasePlanner &planner); // NOLINT

        // This function lets the backtracker know when a trajectory is executed
        virtual bool segmentIsExecuted(const TrajectorySegment &segment);

        // This function is executed when no new trajectories are found.
        virtual bool trackBack(TrajectorySegment *target) = 0;
    };

} // namespace stickynav_planning
#endif // STICKYNAV_MODULE_BACK_TRACKER_H_