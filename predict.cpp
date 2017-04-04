#include <iostream>
#include <vector>
#include <eigen3>

using namespace Eigen;
using namespace std;

double pi_to_pi(double angle){
	angle = fmod(angle, 2*3.14);
	if(angle > 3.14){
		angle = angle - 2*3.14;
	}
	else if (angle < -3.14){
		angle = angle + 2*3.14
	}
	return ;
}

void EKF_predict(double* x,double** P,double v,double g,double** Q,double dt){
// function [xn,Pn]= predict (x,P,v,g,Q,WB,dt)

//  Inputs:
//    x, P - state and covariance
//    v, g - control inputs: velocity and gamma (steer angle)
//    Q - covariance matrix for velocity and gamma
//    dt - timestep

//  Outputs: 
//    x, P - predicted state and covariance
 
//  <------------------------- TO DO -------------------------->
 
	// double J = {	{1, 0, -v*dt*sin(g + x[3])},
	// 	 	   		{0, 1, v*dt*cos(g + x[3])},
	//  		   		{0, 0, 1}		};
	// double M = {	{dt*cos(g + x[3]),-v*dt*sin(g + x[3])},
	// 				{dt*sin(g + x[3]),v*dt*cos(g + x[3])},
	// 				{0,1}		};

	// // P = J * P * J' + M * Q * M';

	// x[1] = x[1] + v * dt * cos(g + x[3]);
	// x[2] = x[2] + v * dt * sin(g + x[3]); 
	x[3] = pi_to_pi(x[3] + g);
}