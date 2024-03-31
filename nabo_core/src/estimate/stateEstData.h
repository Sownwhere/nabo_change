#ifndef NABO_CORE_SRC_ESTIMATE_STATEESTDATA_H_
#define NABO_CORE_SRC_ESTIMATE_STATEESTDATA_H_
#include"eigen.h"
namespace Est{
struct StateEstimate {
    vec2d contactEstimate;
    vec3d position;
    vec3d vBody;
    Eigen::Matrix<double, 4, 1> orientation;
    vec3d omegaBody;
    mat3d rBody;
    vec3d rpy;

    vec3d omegaWorld;
    vec3d vWorld;
    vec3d aBody, aWorld;
};
}
#endif  // NABO_CORE_SRC_ESTIMATE_STATEESTDATA_H_