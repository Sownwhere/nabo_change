#include "LinearKFPositionVelocityEstimator.h"
#include <iostream>

/*!
 * Initialize the state estimator
 */
namespace Est{
LinearKFPositionVelocityEstimator::LinearKFPositionVelocityEstimator(StateEstimate &stateEst)
:StateEstimateResult(stateEst){
    setup();
}

void LinearKFPositionVelocityEstimator::setup() {
    double dt = 0.001;
    _xhat.setZero();
    _ps.setZero();
    _vs.setZero();
    _A.setZero();
    _A.block(0, 0, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(0, 3, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    _A.block(3, 3, 3, 3) = Eigen::Matrix<double, 3, 3>::Identity();
//    _A.block(6, 6, 12, 12) = Eigen::Matrix<double, 12, 12>::Identity();
    _A.block(6, 6, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
    _B.setZero();
    _B.block(3, 0, 3, 3) = dt * Eigen::Matrix<double, 3, 3>::Identity();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C1(3, 6);
    C1 << Eigen::Matrix<double, 3, 3>::Identity(), Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> C2(3, 6);
    C2 << Eigen::Matrix<double, 3, 3>::Zero(), Eigen::Matrix<double, 3, 3>::Identity();
    _C.setZero();

    _C.block(0, 0, 3, 6) = C1;
    _C.block(3, 0, 3, 6) = C1;
    _C.block(6, 0, 3, 6) = C2;
    _C.block(9, 0, 3, 6) = C2;
    _C.block(0, 6, 6, 6) = double(-1) * Eigen::Matrix<double, 6, 6>::Identity();
    _C(13, 11) = double(1);
    _C(12, 8) = double(1);
    _P.setIdentity();
    _P = double(100) * _P;
    _Q0.setIdentity();
    _Q0.block(0, 0, 3, 3) = (dt / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(3, 3, 3, 3) = (dt * 9.8f / 20.f) * Eigen::Matrix<double, 3, 3>::Identity();
    _Q0.block(6, 6, 6, 6) = dt * Eigen::Matrix<double, 6, 6>::Identity();
    _R0.setIdentity();
}

/*!
 * Run state estimator
 */

void LinearKFPositionVelocityEstimator::run(vec3d *hip2 , vec3d *ankleP2B, vec3d *ankleV2B, double *swPgs) {;
    static int sleep_time=0;
    sleep_time++;

    if(sleep_time>5) {
        std::cout<<"-------------------2----------------------"<<std::endl;
        For(2){
            StateEstimateResult.contactEstimate[i] = swPgs[i];
        }
         std::cout<<"------------------"<<swPgs[1]<<"----------------------"<<std::endl;
        double process_noise_pimu = 0.02;
        double process_noise_vimu = 0.02;
        // double process_noise_vimu = 0.1;
        double process_noise_pfoot = 0.002;
        double sensor_noise_pimu_rel_foot = 0.001;
        double sensor_noise_vimu_rel_foot = 0.1;
        double sensor_noise_zfoot = 0.001;

        Eigen::Matrix<double, 12, 12> Q = Eigen::Matrix<double, 12, 12>::Identity();
        Q.block(0, 0, 3, 3) = _Q0.block(0, 0, 3, 3) * process_noise_pimu;
        Q.block(3, 3, 3, 3) = _Q0.block(3, 3, 3, 3) * process_noise_vimu;
        Q.block(6, 6, 6, 6) = _Q0.block(6, 6, 6, 6) * process_noise_pfoot;

        Eigen::Matrix<double, 14, 14> R = Eigen::Matrix<double, 14, 14>::Identity();

        R.block(0, 0, 6, 6) = _R0.block(0, 0, 6, 6) * sensor_noise_pimu_rel_foot;
        R.block(6, 6, 6, 6) = _R0.block(6, 6, 6, 6) * sensor_noise_vimu_rel_foot;
        R.block(12, 12, 2, 2) = _R0.block(12, 12, 2, 2) * sensor_noise_zfoot;
        // std::cout<<"- recive orientation-\n"<< StateEstimateResult.orientation.transpose() <<""<<std::endl;
        int qindex = 0;
        int rindex1 = 0;
        int rindex2 = 0;
        int rindex3 = 0;

        vec3d g(0, 0, double(-9.81));
        mat3d  Rbod = StateEstimateResult.rBody.transpose();

        std::cout<<"- Rbod-\n"<< Rbod <<""<<std::endl;
        // in old code, Rbod * se_acc + g
        vec3d  a = StateEstimateResult.aWorld + g;
        // Vec3<double> a;
        // a << 0,0,0;
        std::cout << "A WORLD\n" << a << "\n";
        vec2d pzs = vec2d::Zero();
        vec2d trusts = vec2d::Zero();
        vec3d p0, v0;
        p0 << _xhat[0], _xhat[1], _xhat[2];
        v0 << _xhat[3], _xhat[4], _xhat[5];

//        for (int i = 0; i < 4; i++) {
        for (int i = 0; i < 2; i++) {

            int i1 = 3 * i;
            // Quadruped<double> &quadruped = *(this->_stateEstimatorData.legControllerData->quadruped);
            vec3d ph =  hip2[i];  // hip positions relative to Co
            std::cout<<"hip2 "<<i<<": " << hip2[i]<<std::endl;

            vec3d p_rel =  ankleP2B[i];
            std::cout<<"ankleP2B "<<i<<": " <<ankleP2B[i] <<std::endl;

            vec3d dp_rel =  ankleV2B[i];  
            vec3d p_f = Rbod * p_rel;
            vec3d dp_f = Rbod * (StateEstimateResult.omegaBody.cross(p_rel) + dp_rel);

            qindex = 6 + i1;
            rindex1 = i1;

            rindex2 = 6 + i1;
            rindex3 = 12 + i;

            double trust = double(1);
            double phase = fmin(StateEstimateResult.contactEstimate(i), double(1));
        //     std::cout<<"phase"<<phase<<std::endl;
            //double trust_window = double(0.25);
            double trust_window = double(0.2);

            if (phase < trust_window) {
                trust = phase / trust_window;
            } else if (phase > (double(1) - trust_window)) {
                trust = (double(1) - phase) / trust_window;
            }
            double high_suspect_number(999999);
//            double high_suspect_number(100);
            // printf("Trust %d: %.3f\n", i, trust);
            Q.block(qindex, qindex, 3, 3) =
                    (double(1) + (double(1) - trust) * high_suspect_number) * Q.block(qindex, qindex, 3, 3);
            R.block(rindex1, rindex1, 3, 3) = 1 * R.block(rindex1, rindex1, 3, 3);
            R.block(rindex2, rindex2, 3, 3) =
                    (double(1) + (double(1) - trust) * high_suspect_number) * R.block(rindex2, rindex2, 3, 3);
            R(rindex3, rindex3) =
                    (double(1) + (double(1) - trust) * high_suspect_number) * R(rindex3, rindex3);

            trusts(i) = trust;

            _ps.segment(i1, 3) = -p_f;
            _vs.segment(i1, 3) = (1.0f - trust) * v0 + trust * (-dp_f);
            pzs(i) = (1.0f - trust) * (p0(2) + p_f(2));


        }
//        Eigen::Matrix<double, 28, 1> y;
        Eigen::Matrix<double, 14, 1> y;
        y << _ps, _vs, pzs;

        _xhat = _A * _xhat + _B * a;


        Eigen::Matrix<double, 12, 12> At = _A.transpose();
        Eigen::Matrix<double, 12, 12> Pm = _A * _P * At + Q;
        Eigen::Matrix<double, 12, 14> Ct = _C.transpose();
        Eigen::Matrix<double, 14, 1> yModel = _C * _xhat;
        Eigen::Matrix<double, 14, 1> ey = y - yModel;
        Eigen::Matrix<double, 14, 14> S = _C * Pm * Ct + R;

        // todo compute LU only once
//        Eigen::Matrix<double, 28, 1> S_ey = S.lu().solve(ey);
        Eigen::Matrix<double, 14, 1> S_ey = S.lu().solve(ey);
        _xhat += Pm * Ct * S_ey;

//        Eigen::Matrix<double, 28, 18> S_C = S.lu().solve(_C);
//        _P = (Eigen::Matrix<double, 18, 18>::Identity() - Pm * Ct * S_C) * Pm;
        Eigen::Matrix<double, 14, 12> S_C = S.lu().solve(_C);
        _P = (Eigen::Matrix<double, 12, 12>::Identity() - Pm * Ct * S_C) * Pm;

//        Eigen::Matrix<double, 18, 18> Pt = _P.transpose();
        Eigen::Matrix<double, 12, 12> Pt = _P.transpose();
        _P = (_P + Pt) / double(2);

        if (_P.block(0, 0, 2, 2).determinant() > double(0.000001)) { //? 表示xy位置的增益协方差大于0
            _P.block(0, 2, 2, 10).setZero();
            _P.block(2, 0, 10, 2).setZero();
            _P.block(0, 0, 2, 2) /= double(10);
        }
        if(1) {
            StateEstimateResult.position = _xhat.block(0, 0, 3, 1);
            StateEstimateResult.vWorld = _xhat.block(3, 0, 3, 1);
            StateEstimateResult.vBody =
                    StateEstimateResult.rBody * StateEstimateResult.vWorld;
        }
        // zp added
        // StateEstimateResult.vWorld<<0,0,0;
        // StateEstimateResult.vWorld[2]=0;
        // StateEstimateResult.vWorld[0]=0;
        // StateEstimateResult.vBody =
        //             StateEstimateResult.rBody * StateEstimateResult.vWorld;
        // StateEstimateResult.position<<0,0,0.65;
        // StateEstimateResult.position(0) = 0;
        // StateEstimateResult.position(1) = 0;
        // StateEstimateResult.position(2) = 0.75;
        std::cout << "StateEstimateResult.position" << StateEstimateResult.position.transpose()
                 << std::endl;

        std::cout << "StateEstimateResult.vWord" << StateEstimateResult.vWorld.transpose()
                 << std::endl;
        // zp added
//        std::cout << "StateEstimateResult.position" << this->_stateEstimatorData.result->position
//                  << std::endl;
    }
}



}