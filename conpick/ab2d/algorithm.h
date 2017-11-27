#ifndef AB2D_ALGORITHM_H
#define AB2D_ALGORITHM_H

#include "model.h"

namespace ab2d
{
    void fk(const Robot& robot, const Eigen::VectorXd& pos, TransArray& rLinkTransform);
    void fk(const Robot& robot, const Eigen::VectorXd& pos, Cache& rCache);
    void propagateVel(const Robot& robot, const Eigen::VectorXd& vel, Cache& rCache);

    void fd(const Robot& robot, const Eigen::VectorXd& vel, const Eigen::VectorXd& force, const WrenchArray& extf,
            Cache& rCache, Eigen::VectorXd& rAcc);
    void fd(const Robot& robot, const Eigen::VectorXd& force, const WrenchArray& extf,
            Cache& rCache, Eigen::VectorXd& rAcc);
    void id(const Robot& robot, const Eigen::VectorXd& vel, const Eigen::VectorXd& acc, const WrenchArray& extf,
            Cache& rCache, Eigen::VectorXd& rForce);
    void id(const Robot& robot, const Eigen::VectorXd& acc, const WrenchArray& extf,
            Cache& rCache, Eigen::VectorXd& rForce);
    void calcBiasForce(const Robot& robot, const WrenchArray& extf,
                       Cache& rCache, Eigen::VectorXd& rForce);
    void calcInertia(const Robot& robot,
                     Cache& rCache, Eigen::MatrixXd& rInertia);
}

#endif /* AB2D_ALGORITHM_H */
