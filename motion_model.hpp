#ifndef _MOTION_MODEL_HPP_
#define _MOTION_MODEL_HPP_
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Core>

namespace Scenelib2
{
    class MotionModel
    {
    public:
        MotionModel() {
            fvRES_.resize(13);
            
            fvRES_.setZero();
        }
        ~MotionModel() {}
        void func_fv_and_dfv_by_dxv(const Eigen::VectorXd &xv,
                                   const Eigen::VectorXd &u, const double delta_t);

        //test
        Eigen::VectorXd get_fv()const {return fvRES_;}

    private:
        void extract_r_q_v_omega(Eigen::VectorXd xv,Eigen::VectorXd r,Eigen::Quaterniond q,
        Eigen::VectorXd v,Eigen::VectorXd omega);
        Eigen::Quaterniond QuaternionFromAngularVelocity(const Eigen::VectorXd&u);
        void compose_xv(const Eigen::VectorXd &r,const Eigen::Quaterniond &q,
         const Eigen::VectorXd &v,const Eigen::VectorXd &omega, Eigen::VectorXd xv);

        Eigen::VectorXd fvRES_;
    };
} // namespace Scenelib2
#endif