#if !defined(AB2D_TRANS_H)
#define AB2D_TRANS_H

#include <Eigen/Dense>

namespace ab2d
{
    typedef Eigen::Vector2d Vec2;

    //! 2D transformation (only translation and rotation. no scaling, skewing nor mirroring)
    class Trans
    {
    public:
        Vec2 pos_;              //!< position x, y
        double ang_;            //!< rotation angle

        Trans();
        Trans(const Vec2& pos, double ang);
        Trans(double x, double y, double z);

        Trans operator*(const Trans& rhs) const;
        Trans& operator*=(const Trans& rhs);

        Trans operator*(double scale) const;
        Trans& operator*=(double scale);

        Vec2 rotate(const Vec2& pos) const;
        Trans rotate(const Trans& trans) const;

        Trans inv() const;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
}

#endif /* AB2D_TRANS_H */
