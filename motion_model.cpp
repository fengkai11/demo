#include "motion_model.hpp"
namespace Scenelib2
{
    void MotionModel::func_fv_and_dfv_by_dxv(const Eigen::VectorXd &xv,
                                             const Eigen::VectorXd &u, const double delta_t)
    {
        Eigen::VectorXd rnew, rold, vnew, vold, omeganew, omegaold;
        Eigen::Quaterniond qold, qnew;
        extract_r_q_v_omega(xv, rold, qold, vold, omegaold);
        Eigen::Vector3d acceleration(u);
        //r = r + v*delta_t;
        rnew = rold + vold * delta_t;
        //q = qwt*q;
        Eigen::Quaterniond qwt = QuaternionFromAngularVelocity(omegaold * delta_t);
        qnew = qold * qwt;
        vnew = vold + acceleration * delta_t;
        omeganew = omegaold;
        //put it all together;
        compose_xv(rnew, qnew, vnew, omeganew, fvRES_);
        //jaccabin;
    }

    void MotionModel::extract_r_q_v_omega(Eigen::VectorXd xv, Eigen::VectorXd r, Eigen::Quaterniond q,
                                          Eigen::VectorXd v, Eigen::VectorXd omega)
    {
        r << xv(0), xv(1), xv(2);
        q.x() = xv(3);
        q.y() = xv(4);
        q.z() = xv(5);
        q.w() = xv(6);
        v << xv(7), xv(8), xv(9);
        omega << xv(10), xv(11), xv(12);
    }

    Eigen::Quaterniond MotionModel::QuaternionFromAngularVelocity(const Eigen::VectorXd &av)
    {
        const double angle = sqrt(av(0) * av(0) + av(1) * av(1) + av(2) * av(2));
        Eigen::Quaterniond q;
        if (angle > 0.0)
        {
            const double s = sin(angle / 2) / angle;
            const double c = cos(angle / 2);
            q.x() = s * av(0);
            q.y() = s * av(1);
            q.z() = s * av(2);
            q.w() = c;
        }
        else
        {
            q.x() = q.y() = q.z() = 0;
            q.w() = 1.0;
        }
        return q;
    }
    void MotionModel::compose_xv(const Eigen::VectorXd &r, const Eigen::Quaterniond &q,
                                 const Eigen::VectorXd &v, const Eigen::VectorXd &omega, Eigen::VectorXd xv)
    {
        xv << r, q.x(), q.y(), q.z(), q.w(), v, omega;
    }
} // namespace Scenelib2