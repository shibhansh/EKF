// #include <predict>
#include <iostream>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

template <typename Derived>

Matrix3f void EKF_update(float* x, const Ref<const Matrix3f>& P,float *z,const Ref<const Matrix3f>& R,const Ref<const VectorXf>& idf,int lm){
	//  Inputs:
	//    x, P -  state and covariance
	//    z, R - range-bearing measurements and covariances
	//    idf - feature index for each z
	//  Outputs:
	//    x, P - updated state and covariance
	// % <---------------------------- TO DO -------------------------->

	int size_idf = 3;
	size_idf = size(idf,1);
	if(numel(idf) == 0)
		return;
	else{
	    int i = 0;
		for( i=1, i<=size_idf, i++){
			l = lm(:,idf(i));
			z(2,i) = pi_to_pi(z(2,i));
			float dist = sqrt((x[1]-l[1])^2+(x[2]-l[2])^2);
			VectorXf z_predicted (dist,	pi_to_pi(atan2(l(2)-x(2),l(1)-x(1)) - x(3)));

			Matrix<float, 3,2>	H;
			H << (x(1)-l(1))/dist, (x(2)-l(2))/dist, 0,
				-(x(2)-l(2))/dist^2, (x(1)-l(1))/dist^2, -1;

			Matrix<float, 3,2>	H;
			S = H * P * H.transpose() + R;
			Matrix<float, 3,2>	K;
			K = P*H.transpose()*pseudoInverse(S);
			a = (z(:,i) - z_predicted);
			a(2) = pi_to_pi(a(2));
			x = x + K*a;
			x(3) = pi_to_pi(x(3));
			P = (MatrixXd::Identity(3,3) - K * H)*P;
		}
	}
	return K;
}