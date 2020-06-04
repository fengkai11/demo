#include<iostream>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Eigen>
using namespace std;
using namespace Eigen;
int main(){
    Vector3d u;
    u<<2,5,6;
    cout<<u<<endl;
    return 0;
}