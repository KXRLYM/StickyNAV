#ifndef STICKYNAV_PLANNING_CORE_DATA_TRAJECTORY_SEGMENT_H_
#define STICKYNAV_PLANNING_CORE_DATA_TRAJECTORY_SEGMENT_H_

#include <deque>
#include <iostream>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace stickynav_planning
{
    inline double yawFromQuaternion(const Eigen::Quaterniond &q)
    {
        return atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    }

    inline Eigen::Quaterniond quaternionFromYaw(double yaw)
    {
        return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
    }

    // Base struct that contains trajectory evaluator information associated with
    // the segment.
    struct TrajectoryInfo
    {
        virtual ~TrajectoryInfo() = default;
    };

    struct TrajectoryPoint
    {
        typedef std::vector<TrajectoryPoint, Eigen::aligned_allocator<TrajectoryPoint>> Vector;

        TrajectoryPoint()
            : time_from_start(0),
              position(Eigen::Vector3d::Zero()),
              velocity(Eigen::Vector3d::Zero()),
              acceleration(Eigen::Vector3d::Zero()),
              orientation(Eigen::Quaterniond::Identity()) {}

        TrajectoryPoint(int64_t _time_from_start,
                        const Eigen::Vector3d &_position,
                        const Eigen::Vector3d &_velocity,
                        const Eigen::Vector3d &_acceleration,
                        const Eigen::Quaterniond &_orientation)
            : time_from_start(_time_from_start),
              position(_position),
              velocity(_velocity),
              acceleration(_acceleration),
              orientation(_orientation) {}

        TrajectoryPoint(int64_t _time_from_start,
                        const Eigen::Vector3d &_position)
            : TrajectoryPoint(_time_from_start, _position,
                              Eigen::Vector3d::Zero(),
                              Eigen::Vector3d::Zero(),
                              Eigen::Quaterniond::Identity()) {}

        int64_t time_from_start;
        Eigen::Vector3d position;
        Eigen::Vector3d velocity;
        Eigen::Vector3d acceleration;
        Eigen::Quaterniond orientation;

        inline void setFromYaw(double yaw)
        {
            orientation = quaternionFromYaw(yaw);
        }
    };

    inline TrajectoryPoint operator*(
        const Eigen::Affine3d &lhs,
        const TrajectoryPoint &rhs)
    {
        TrajectoryPoint transformed(rhs);
        transformed.position = lhs * rhs.position;
        transformed.velocity = lhs.rotation() * rhs.velocity;
        transformed.acceleration = lhs.rotation() * rhs.acceleration;
        transformed.orientation = lhs.rotation() * rhs.orientation;
        return transformed;
    }

    // Struct to store trajectory tree data
    struct TrajectorySegment
    {
        TrajectorySegment();

        // All trajectory points
        TrajectoryPoint::Vector trajectory;

        // Associated values
        double gain;
        double cost;
        double value;

        // flags
        bool tg_visited; // trajectory generator visited during expansion site
        // selection

        // pointer to parent trajectory, nullptr for currently active segment (root)
        TrajectorySegment *parent;

        // Pointers to successive trajectory nodes, all nodes are owned by the parent
        std::vector<std::unique_ptr<TrajectorySegment>> children;

        // Information to be carried with this segment, e.g. virtual voxels
        std::unique_ptr<TrajectoryInfo> info;

        // compare function for sorting etc
        static bool compare(TrajectorySegment a, TrajectorySegment b);

        static bool comparePtr(TrajectorySegment *a, TrajectorySegment *b);

        // Safely create a child node and return a pointer to it
        TrajectorySegment *spawnChild();

        // The following utility functions assume a tree structure (no loops)
        // Add pointers to all immediate children to the result vector
        void getChildren(std::vector<TrajectorySegment *> *result);

        // Recursively add pointers to all leaf nodes (have no children) to the result
        // vector
        void getLeaves(std::vector<TrajectorySegment *> *result);

        // Recursively add pointers to all subsequent nodes to the result vector
        void getTree(std::vector<TrajectorySegment *> *result);

        // Recursively add pointers to all subsequent nodes to the result vector up to
        // given depth
        void getTree(std::vector<TrajectorySegment *> *result, int maxdepth);

        // Create a shallow copy of the segment (Includes everything except the unique
        // pointers children and info, which will not be setup!)
        TrajectorySegment shallowCopy();
    };

} // namespace active_3d_planning
#endif // ACTIVE_3D_PLANNING_CORE_DATA_TRAJECTORY_SEGMENT_H_