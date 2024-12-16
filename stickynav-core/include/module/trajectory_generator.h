#ifndef STICKYNAV_MODULE_TRAJECTORY_GENERATOR_H_
#define STICKYNAV_MODULE_TRAJECTORY_GENERATOR_H_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>

#include "data/bounding_volume.h"
#include "data/trajectory_segment.h"
#include "planner_base.h"

namespace stickynav_planning
{
    // Forward declaration
    class SegmentSelector;
    class GeneratorUpdater;

    // Base class for trajectory generation to provide uniform interface with other
    // classes
    class TrajectoryGenerator : public Module
    {
    public:
        explicit TrajectoryGenerator(BasePlanner &planner); // NOLINT

        virtual ~TrajectoryGenerator() = default;

        // Expansion policy where to expand (from full tree)
        virtual bool selectSegment(TrajectorySegment **result,
                                   TrajectorySegment *root);

        // Expand a selected trajectory segment. Return true for successful expansion.
        virtual bool expandSegment(TrajectorySegment *target,
                                   std::vector<TrajectorySegment *> *new_segments) = 0;

        // Update an existing segment when a new trajectory is executed, return true
        // if the segment is to be kept alive, false if it should be removed from the
        // tree
        virtual bool updateSegment(TrajectorySegment *segment);

        // Utility function for collision checking. Returns true if the position is
        // reachable.
        bool checkTraversable(const Eigen::Vector3d &position);

        // in case a trajectory needs to be modified to be published
        virtual bool extractTrajectoryToPublish(
            TrajectoryPoint::Vector *trajectory, const TrajectorySegment &segment);

        void setupFromParamMap(Module::ParamMap *param_map) override;

    protected:
        // bounding box
        std::unique_ptr<BoundingVolume> bounding_volume_;

        // default modules
        std::unique_ptr<SegmentSelector> segment_selector_;
        std::unique_ptr<GeneratorUpdater> generator_updater_;

        // Parameters
        bool p_collision_optimistic_;
        double p_clearing_radius_; // Unknown space within clearing radius is
        // considered traversable
        std::string p_selector_args_;
        std::string p_updater_args_;
    };

    // Abstract encapsulation for default/modular implementations of the
    // selectSegment method
    class SegmentSelector : public Module
    {
    public:
        explicit SegmentSelector(BasePlanner &planner); // NOLINT

        virtual bool selectSegment(TrajectorySegment **result,
                                   TrajectorySegment *root) = 0;
    };

    // Abstract encapsulation for default/modular implementations of the
    // updateSegments method
    class GeneratorUpdater : public Module
    {
    public:
        explicit GeneratorUpdater(BasePlanner &planner); // NOLINT

        virtual bool updateSegment(TrajectorySegment *segment) = 0;
    };

} // namespace stickynav_planning
#endif // STICKYNAV_MODULE_TRAJECTORY_GENERATOR_H_