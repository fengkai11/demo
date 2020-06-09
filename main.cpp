#include<iostream>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Eigen>
#include"motion_model.hpp"
using namespace std;
using namespace Eigen;

int main(){
    Scenelib2::MotionModel t;
    cout<<t.get_fv()<<endl;
    return 0;
}