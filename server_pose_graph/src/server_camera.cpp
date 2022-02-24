
#include "../include/server_camera.h"

using namespace std;

ServerCamera::ServerCamera(){

}


//Without distortion.
// Eigen::Vector3d ServerCamera::LiftProject(Eigen::Vector2d mPixelPoint){
// 	Eigen::Vector3d mHomogeneousPixel(1.0 , 1.0 , 1.0);
// 	mHomogeneousPixel(0) = mPixelPoint(0);
// 	mHomogeneousPixel(1) = mPixelPoint(1);
// 	Eigen::Vector3d mNormalizedPoint = this->m_mK.inverse() * mHomogeneousPixel;
// 	return mNormalizedPoint;
// }

//With distortion.
Eigen::Vector3d ServerCamera::LiftProject(Eigen::Vector2d mPixelPoint){
	double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
    double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
    //double lambda;


	Eigen::Vector3d mHomogeneousPixel(1.0 , 1.0 , 1.0);
	mHomogeneousPixel(0) = mPixelPoint(0);
	mHomogeneousPixel(1) = mPixelPoint(1);
	Eigen::Vector3d mNormalizedPoint = this->m_mK.inverse() * mHomogeneousPixel;

    // Lift points to normalised plane
    mx_d = mNormalizedPoint(0)/mNormalizedPoint(2);
    my_d = mNormalizedPoint(1)/mNormalizedPoint(2);

    
    // Recursive distortion model
    int n = 8;
    Eigen::Vector2d d_u;
    Eigen::Vector2d mUndistortPoint = Eigen::Vector2d(mx_d, my_d);
    this->Distortion(mUndistortPoint, d_u);
    // Approximate value
    mx_u = mx_d - d_u(0);
    my_u = my_d - d_u(1);

    for (int i = 1; i < n; ++i)
    {
    	Eigen::Vector2d mLocalUndistortPoint = Eigen::Vector2d(mx_u, my_u);
        this->Distortion(mLocalUndistortPoint, d_u);
        mx_u = mx_d - d_u(0);
        my_u = my_d - d_u(1);
    }

    Eigen::Vector3d mDistortedResult;
    // Obtain a projective ray
    mDistortedResult << mx_u, my_u, 1.0;

	return mDistortedResult;
}

//Without distortion
// Eigen::Vector2d ServerCamera::Project(Eigen::Vector3d mNormalizedPoint){
// 	Eigen::Vector3d mHomogeneousPixel = this->m_mK * mNormalizedPoint;
// 	Eigen::Vector2d mPixel(	mHomogeneousPixel.x()/mHomogeneousPixel.z(),
// 							mHomogeneousPixel.y()/mHomogeneousPixel.z());
// 	return mPixel;
// }


Eigen::Vector2d ServerCamera::Project(Eigen::Vector3d mNormalizedPoint){
	Eigen::Vector2d p_u, p_d;

    // Project points to the normalised plane
    p_u << mNormalizedPoint(0) / mNormalizedPoint(2), mNormalizedPoint(1) / mNormalizedPoint(2);

    // Apply distortion
    Eigen::Vector2d d_u;
    this->Distortion(p_u, d_u);
    p_d = p_u + d_u;


    Eigen::Vector2d mDistortedResult;
    Eigen::Vector3d mDistortedNormalizedPoint;

    double nFx = this->m_mK(0 , 0);
    double nFy = this->m_mK(1 , 1);
    double nCx = this->m_mK(0 , 2);
    double nCy = this->m_mK(1 , 2);


    // Apply generalised projection matrix
    mDistortedResult << nFx * p_d(0) + nCx,
         				nFy * p_d(1) + nCy;

    return mDistortedResult;

}


void ServerCamera::Distortion(Eigen::Vector2d& p_u, Eigen::Vector2d& d_u)
{
    double k1 = this->m_mD(0 , 0);
    double k2 = this->m_mD(1 , 0);
    double p1 = this->m_mD(2 , 0);
    double p2 = this->m_mD(3 , 0);

    double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

    mx2_u = p_u(0) * p_u(0);
    my2_u = p_u(1) * p_u(1);
    mxy_u = p_u(0) * p_u(1);
    rho2_u = mx2_u + my2_u;
    rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
    d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
           p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
}





// void
// PinholeCamera::liftProjective(const Eigen::Vector2d& p, Eigen::Vector3d& P) const
// {
//     double mx_d, my_d,mx2_d, mxy_d, my2_d, mx_u, my_u;
//     double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
//     //double lambda;

//     // Lift points to normalised plane
//     mx_d = m_inv_K11 * p(0) + m_inv_K13;
//     my_d = m_inv_K22 * p(1) + m_inv_K23;

//     if (m_noDistortion)
//     {
//         mx_u = mx_d;
//         my_u = my_d;
//     }
//     else
//     {
//         if (0)
//         {
//             double k1 = mParameters.k1();
//             double k2 = mParameters.k2();
//             double p1 = mParameters.p1();
//             double p2 = mParameters.p2();

//             // Apply inverse distortion model
//             // proposed by Heikkila
//             mx2_d = mx_d*mx_d;
//             my2_d = my_d*my_d;
//             mxy_d = mx_d*my_d;
//             rho2_d = mx2_d+my2_d;
//             rho4_d = rho2_d*rho2_d;
//             radDist_d = k1*rho2_d+k2*rho4_d;
//             Dx_d = mx_d*radDist_d + p2*(rho2_d+2*mx2_d) + 2*p1*mxy_d;
//             Dy_d = my_d*radDist_d + p1*(rho2_d+2*my2_d) + 2*p2*mxy_d;
//             inv_denom_d = 1/(1+4*k1*rho2_d+6*k2*rho4_d+8*p1*my_d+8*p2*mx_d);

//             mx_u = mx_d - inv_denom_d*Dx_d;
//             my_u = my_d - inv_denom_d*Dy_d;
//         }
//         else
//         {
//             // Recursive distortion model
//             int n = 8;
//             Eigen::Vector2d d_u;
//             distortion(Eigen::Vector2d(mx_d, my_d), d_u);
//             // Approximate value
//             mx_u = mx_d - d_u(0);
//             my_u = my_d - d_u(1);

//             for (int i = 1; i < n; ++i)
//             {
//                 distortion(Eigen::Vector2d(mx_u, my_u), d_u);
//                 mx_u = mx_d - d_u(0);
//                 my_u = my_d - d_u(1);
//             }
//         }
//     }

//     // Obtain a projective ray
//     P << mx_u, my_u, 1.0;
// }


// /**
//  * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
//  *
//  * \param P 3D point coordinates
//  * \param p return value, contains the image point coordinates
//  */
// void
// PinholeCamera::spaceToPlane(const Eigen::Vector3d& P, Eigen::Vector2d& p) const
// {
//     Eigen::Vector2d p_u, p_d;

//     // Project points to the normalised plane
//     p_u << P(0) / P(2), P(1) / P(2);

//     if (m_noDistortion)
//     {
//         p_d = p_u;
//     }
//     else
//     {
//         // Apply distortion
//         Eigen::Vector2d d_u;
//         distortion(p_u, d_u);
//         p_d = p_u + d_u;
//     }

//     // Apply generalised projection matrix
//     p << mParameters.fx() * p_d(0) + mParameters.cx(),
//          mParameters.fy() * p_d(1) + mParameters.cy();
// }