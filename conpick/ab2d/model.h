#if !defined(AB2D_MODEL_H)
#define AB2D_MODEL_H

#include "trans.h"
#include <vector>

namespace ab2d
{
    struct Link;
    typedef std::vector<Link> LinkArray;

    enum class JointType
    {
        HINGE = 0, SLIDER, FLOATING, FIXED, NUM_TYPES
    };

    struct Link
    {
        int numDims();
        bool isAncestorOf(const Link& descendant) const;

        std::string name_;
        double inertia_;
        JointType jointType_;
        Vec2 subspace_;         /* only for defining slider's axis */
        double hiStop_;
        double loStop_;
        Trans offset_;
        int index_;
        int dimIndex_;
        Link* pParent_;
    };

    struct Model
    {
        LinkArray links_;
        Vec2 gravity_;
    };
}

#endif /* AB2D_MODEL_H */
