#include "VectorNavOrientationEstimator.h"
#define USE_MIT_VEC_NAV


#include <iostream>
namespace Est{

VectorNavOrientationEstimator::VectorNavOrientationEstimator(StateEstimate &stateEst): StateEstimateResult(stateEst){}  
 
void VectorNavOrientationEstimator::run(Rbt::imuClass &imu) {

    // std::cout << "\033[1;32m VectorNavOrientationEstimator<T>::run() hahaha  \033[0m \n";
    StateEstimateResult.orientation[0] =
        imu.orientation.x();
    StateEstimateResult.orientation[1] =
        imu.orientation.y();
    StateEstimateResult.orientation[2] =
        imu.orientation.z();
    StateEstimateResult.orientation[3] =
        imu.orientation.w();
    std::cout<<" StateEstimateResult.orientation"<< StateEstimateResult.orientation.transpose()<<endl;
    

    Eigen::Matrix<double, 3, 1> rpy_init_hl_original;
    Eigen::Matrix<double, 3, 1>rpy_init_hl_after_r_p_0;
    Eigen::Matrix<double, 3, 1>rpy_ini;



    if(_b_first_visit){
        rpy_ini = ori::quatToRPY(StateEstimateResult.orientation);
        rpy_init_hl_original= rpy_ini;
        rpy_ini[0] = 0;
        rpy_ini[1] = 0;
        _ori_ini_inv = ori::rpyToQuat(-rpy_ini);
        _b_first_visit = false;
    }


    StateEstimateResult.orientation = 
        ori::quatProduct(_ori_ini_inv,StateEstimateResult.orientation);

    StateEstimateResult.rpy =
        ori::quatToRPY(StateEstimateResult.orientation);

    //   std::cout << "\033[1;32m this->_stateEstimatorData.result->rpy=\n"<<this->_stateEstimatorData.result->rpy<<" \033[0m \n";

    StateEstimateResult.rBody = ori::quaternionToRotationMatrix(
        StateEstimateResult.orientation);
        //   this->_stateEstimatorData.result->rBody <<1,0,0,0,1,0,0,0,1;
    #ifdef USE_JN_SDU_BIPED
    //   Correct_Omega_Acc();
    #endif
    for(int i =0 ; i<3;i++){
        StateEstimateResult.omegaBody[i] = imu.w[i];
    }


    StateEstimateResult.omegaWorld =
        StateEstimateResult.rBody.transpose() * StateEstimateResult.omegaBody;

    StateEstimateResult.aBody = imu.acc;
    StateEstimateResult.aWorld =
        StateEstimateResult.rBody.transpose() * StateEstimateResult.aBody;


    // std::cout<<"StateEstimateResult.aBody"<<StateEstimateResult.aBody.transpose()<<std::endl;
    // std::cout<<"StateEstimateResult.rBody" << StateEstimateResult.rBody<<std::endl;
    // std::cout<<"StateEstimateResult.orientation" << StateEstimateResult.orientation.transpose()<<std::endl;
    // std::cout<<" StateEstimateResult.omegaBody" <<  StateEstimateResult.omegaBody.transpose()<<std::endl;
    std::cout<<" StateEstimateResult.rpy" <<StateEstimateResult.rpy.transpose()<<std::endl;
    std::cout<<"StateEstimateResult.aBody"<<StateEstimateResult.aBody.transpose()<<std::endl;
    std::cout<<"StateEstimateResult.aWorld " << StateEstimateResult.aWorld.transpose() <<std::endl;
    // std::cout<<"" << <<std::endl;
    // std::cout<<"" << <<std::endl;
    // std::cout<<"" << <<std::endl;
    // std::cout<<" ------------------OrientationEstimator end------------------------"<<std::endl;


}
}


