#include <vector>

#include <data/visualization_markers.h>

namespace stickynav_planning
{

    void VisualizationMarkers::addMarker(const VisualizationMarker &marker)
    {
        markers.push_back(marker);
    }

    int VisualizationMarkers::getNextVisualizationId()
    {
        if (markers.empty())
        {
            return 0;
        }
        else
        {
            return markers.back().id + 1;
        }
    }

    const std::vector<VisualizationMarker> &VisualizationMarkers::getMarkers()
        const
    {
        return markers;
    }

    std::vector<VisualizationMarker> &VisualizationMarkers::getMarkers()
    {
        return markers;
    }

} // namespace stickynav_planning