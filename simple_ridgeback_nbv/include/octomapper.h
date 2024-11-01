#ifndef OCTOMAPPER_H
#define OCTOMAPPER_H

#include <octomap/OcTreeNode.h>
#include <octomap/OccupancyOcTreeBase.h>

static constexpr float LOW_INTEREST = 0.0;
static constexpr float OCCLUSION = 1.0;
static constexpr float SEEN = 2.0;
static constexpr float POTENTIAL_OBJECT = 3.0;
static constexpr float TO_EXPLORE = 4.0;

namespace octomap {

    class OctoNode : public OcTreeNode {
    public:
        OctoNode(): OcTreeNode(), value(LOW_INTEREST) {}

        void setTargetValue(int value) {
            value = value;
        }

        int getTargetValue() {
            return value;
        }

    protected:
        unsigned int value;

    };

    class OctoMapper : public OccupancyOcTreeBase <OctoNode> {
        public: 
        
        OctoMapper(double resolution) : OccupancyOcTreeBase<OctoNode>(resolution) {
            octoMapperMemberInit.ensureLinking();
        }

        // Constructor that creates an OctoMapper from an existing OcTree
        OctoMapper(double resolution, const octomap::OcTree& ocTree) 
            : OccupancyOcTreeBase<OctoNode>(resolution) {
            
            octoMapperMemberInit.ensureLinking();
            // Copy data from the provided OcTree to this OctoMapper
            for (auto it = ocTree.begin_leafs(); it != ocTree.end_leafs(); ++it) {
                octomap::point3d point = it.getCoordinate();
                float occupancy = it->getLogOdds(); // Assuming occupancy is an int
                this->updateNode(point, occupancy, false); // Set occupancy

                // Set additional values as needed; here we can set a default or custom value
                OctoNode* node = this->search(point);
                if (node) {
                    node->setTargetValue(LOW_INTEREST); // Set your custom target value here
                }
            }
        }

        std::string getTreeType() const {return "OcTree";}


        virtual AbstractOcTree* create() const override {
            return new OctoMapper(this->getResolution());
        }

        protected:
        class StaticMemberInitializer{
            public:
            StaticMemberInitializer() {
                OctoMapper* tree = new OctoMapper(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            void ensureLinking() {}
        };

        static StaticMemberInitializer octoMapperMemberInit;

    };


}

#endif 