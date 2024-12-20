#ifndef STICKY_NAV_PLANNER_BASE_H_
#define STICKY_NAV_PLANNER_BASE_H_

#include <string>

#include <Eigen/Dense>

namespace stickynav_planning
{

    class BackTracker;

    class TrajectoryGenerator;

    class TrajectoryEvaluator;

    class ModuleFactory;

    class Map;

    struct VisualizationMarkers;
    struct SystemConstraints;

    // basic interface of planners for module-access
    class BasePlanner
    {
    public:
        virtual ~BasePlanner() = default;

        // state accessors
        virtual const Eigen::Vector3d &getCurrentPosition() const = 0;

        virtual const Eigen::Quaterniond &getCurrentOrientation() const = 0;

        // member accessors
        virtual BackTracker &getBackTracker() = 0;

        virtual TrajectoryGenerator &getTrajectoryGenerator() = 0;

        virtual TrajectoryEvaluator &getTrajectoryEvaluator() = 0;

        virtual Map &getMap() = 0;

        virtual SystemConstraints &getSystemConstraints() = 0;

        // methods
        virtual void publishVisualization(const VisualizationMarkers &markers) = 0;

        virtual void printInfo(const std::string &text) = 0;

        virtual void printWarning(const std::string &text) = 0;

        virtual void printError(const std::string &text) = 0;
    };
} // namespace stickynav_planning

#endif // STICKYNAV_PLANNER_BASE_H_