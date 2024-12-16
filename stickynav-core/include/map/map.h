#ifndef STICKYNAV_MAP_MAP_H_
#define STICKYNAV_MAP_MAP_H_

#include "data/trajectory_segment.h"
#include "module/module.h"
#include "planner_base.h"

namespace stickynav_planning
{

    // base interface for the bare minimum any map must provide
    class Map : public Module
    {
    public:
        virtual ~Map() = default;

        explicit Map(BasePlanner &planner); // NOLINT

        // check collision for a single pose
        virtual bool isTraversable(const Eigen::Vector3d &position,
                                   const Eigen::Quaterniond &orientation =
                                       Eigen::Quaterniond(1, 0, 0, 0)) = 0;

        // check collision for a path
        virtual bool isTraversablePath(const TrajectoryPoint::Vector &trajectory);

        // check whether point is part of the map
        virtual bool isObserved(const Eigen::Vector3d &point) = 0;
    };

} // namespace stickynav_planning

#endif // STICKYNAV_MAP_MAP_H_