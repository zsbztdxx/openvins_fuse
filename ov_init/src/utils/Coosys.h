#ifndef OV_INIT_C00SYS_H
#define OV_INIT_C00SYS_H

#include <iostream>
#include <ceres/ceres.h>
#include <eigen3/Eigen/Dense>
#include <math.h>

class myPoseLocalParameterization : public ceres::LocalParameterization
{
    virtual bool Plus(const double *x, const double *delta, double *x_plus_delta) const
    {
        for(int i=0;i<5;i++){
            x_plus_delta[i] = x[i] + delta[i];
        }

        return true;
    };
    virtual bool ComputeJacobian(const double *x, double *jacobian) const
    {
        Eigen::Map<Eigen::Matrix<double, 5, 5, Eigen::RowMajor>> j(jacobian);

        j.topRows<5>().setIdentity();

        return true;
    };
    virtual int GlobalSize() const { return 5; };
    virtual int LocalSize() const { return 5; };
};

class MyScalarCostFunctor
{

public:
    MyScalarCostFunctor(Eigen::Vector3d pw, Eigen::Vector3d pc) : pw_(pw), pc_(pc) {}
    template <typename T>
    bool operator()(const T *const par, T *residuals) const
    {

        Eigen::Matrix<T, 3, 1> pww, pcc;
        pww[0] = T(pw_[0]);
        pww[1] = T(pw_[1]);
        pww[2] = T(pw_[2]);
        pcc[0] = T(pc_[0]);
        pcc[1] = T(pc_[1]);
        pcc[2] = T(pc_[2]);
        T s = par[0];
        Eigen::Matrix<T, 3, 1> tt(par[1], par[2], par[3]);

        T yaw = par[4];
        Eigen::Matrix<T, 3, 3> Rz;
        Rz << cos(yaw), -sin(yaw), T(0),
            sin(yaw), cos(yaw), T(0),
            T(0), T(0), T(1);

        Eigen::Matrix<T, 3, 1> pre = pww - (s * (Rz * pcc) + tt);
        residuals[0] = pre[0];
        residuals[1] = pre[1];
        residuals[2] = pre[2];
        return true;
    }

    static ceres::CostFunction *Create(const Eigen::Vector3d pww, const Eigen::Vector3d pcc)
    {
        return (new ceres::AutoDiffCostFunction<MyScalarCostFunctor, 3, 5>(
            new MyScalarCostFunctor(pww, pcc)));
    }
private:
    Eigen::Vector3d pw_;
    Eigen::Vector3d pc_;
};

inline bool getRTWC(const std::vector<Eigen::Vector3d> &pws, const std::vector<Eigen::Vector3d> &pcs, 
                    Eigen::Matrix3d &RWC, Eigen::Vector3d &TWC, double &SWC)
{
    double par[5] = {1, 0, 0, 0, 0};
    ceres::Problem problem;
    ceres::LossFunction *loss_function = new ceres::HuberLoss(1.0);

    ceres::LocalParameterization *local_parameterization = new myPoseLocalParameterization();
    problem.AddParameterBlock(par, 5, local_parameterization);

    for (size_t i = 0; i < pws.size(); i++)
    {
        ceres::CostFunction *cost_function = MyScalarCostFunctor::Create(pws[i], pcs[i]);
        problem.AddResidualBlock(cost_function, loss_function, par);
    }

    ceres::Solver::Options options;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.num_threads = 2;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    double yaw = par[4];
    Eigen::Matrix<double, 3, 3> Rz;
    Rz << cos(yaw), -sin(yaw), 0,
          sin(yaw), cos(yaw), 0,
          0, 0, 1;

    RWC = Rz;
    TWC[0] = par[1];
    TWC[1] = par[2];
    TWC[2] = par[3];
    SWC = par[0];

    double sum = 0;
    int num = pws.size();

    for (size_t i = 0; i < pws.size(); i++)
    {
        Eigen::Vector3d pww = SWC * (RWC * pcs[i]) + TWC;
        Eigen::Vector3d dis = pws[i] - pww;
        std::cout << "********" << dis.transpose() << std::endl;
        //double dd = sqrt(dis[0] * dis[0] + dis[1] * dis[1] + dis[2] * dis[2]);
        double dd = sqrt(dis[0] * dis[0] + dis[1] * dis[1]);
        sum += dd;
    }

    double mean = sum / num;
    std::cout << "mean error:" << mean << std::endl;
    if (mean > 0.05){
        return false;
    }
        
    return true;
}

#endif
