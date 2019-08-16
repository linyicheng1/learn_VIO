#include <iostream>
#include <cmath>
using namespace std; 

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.hpp"
#include "sophus/se3.hpp"

/*
 * 用计算出来的w对某旋转更新时有两种不同方式
 * 旋转矩阵*exp()
 * 四元数
 * 下面通过代码验证两种方式对小量w，可等同
 */
int main( int argc, char** argv )
{
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SO3<double> SO3_R(R);//初始位姿
    cout<<"SO3  = "<<SO3_R.matrix()<<endl;
    cout<<"******************"<<endl;
    Eigen::Vector3d update_so3(0.01, 0.02, 0.03);
    Sophus::SO3<double> SO3_updated  = SO3_R*Sophus::SO3<double>::exp(update_so3);
    cout<<"SO3 updated = "<<SO3_updated.matrix()<<endl;
    cout<<"******************"<<endl;
    Eigen::Quaterniond q(SO3_R.matrix());
    Eigen::Quaterniond update_q(1,0.005,0.01,0.015);
    Eigen::Quaterniond q_update = q*update_q;
    cout<<"q updated = "<<q_update.matrix()<<endl;
    cout<<"******************"<<endl;
    return 0;
}