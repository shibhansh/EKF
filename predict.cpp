#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

template <typename Derived>


float pi_to_pi(float angle){
	angle = fmod(angle, 2*3.14);
	if(angle > 3.14){
		angle = angle - 2*3.14;
	}
	else if (angle < -3.14){
		angle = angle + 2*3.14;
	}
	return angle;
}

Matrix3f EKF_predict(float* x,const Ref<const Matrix3f>& P,float v,float g,const Ref<const Matrix2f>& Q,float dt){
// function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)

//  Inputs:
//    x, P - state and covariance
//    v, g - control inputs: velocity and gamma (steer angle)
//    Q - covariance matrix for velocity and gamma
//    dt - timestep

//  Outputs: 
//    x, P - predicted state and covariance
 
//  <------------------------- TO DO -------------------------->
 
	Matrix<float, 3, 3> J;
	J <<	1, 0, -v*dt*sin(g + x[2]),
		 	0, 1, v*dt*cos(g + x[2]),
	 		0, 0, 1;

	Matrix<float, 3, 2> M;
	M << 	dt*cos(g + x[2]),-v*dt*sin(g + x[2]),
			dt*sin(g + x[2]),v*dt*cos(g + x[2]),
			0,1;

	Matrix<float, 3, 3> Temp;
	Temp = J * P * J.transpose() + M * Q * M.transpose();
	
	x[0] = x[0] + v * dt * cos(g + x[2]);
	x[1] = x[1] + v * dt * sin(g + x[2]); 
	x[2] = x[2] + g;
	return Temp;
}

int main(){
	float x[3] = {1,2,3} ;
	Matrix<float, 3, 3> P;
	P <<	1, 0, 1,
		 	0, 1, 1,
	 		0, 0, 1;
	float v = 0.1;
	float g = 0.001;
	Matrix<float, 2, 2> Q;
	Q <<	1, 0,
		 	0, 1;
	float dt = 0.2;
	Matrix3f alpha;
	alpha = EKF_predict(x,P,v,g,Q,dt);
	return 0;
}