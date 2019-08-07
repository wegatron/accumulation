#ifndef GZ_TRANSFORM_H
#define GZ_TRANSFORM_H

#include <Eigen/Dense>

/**
 * Transform x, y, z, roll(rx), pitch(ry), yaw(rz) in radius to Eigen::Isometry3d. 
 */
template<typename SCALAR>
Eigen::Transform<SCALAR, 3, Eigen::Isometry> gz_pose2eigen_iso(const SCALAR x, const SCALAR y, const SCALAR z,
	const SCALAR roll, const SCALAR pitch, const SCALAR yaw)
{
	Eigen::AngleAxis<SCALAR> ry(pitch, Eigen::Matrix<SCALAR, 3, 1>::UnitY());
	Eigen::AngleAxis<SCALAR> rx(roll, Eigen::Matrix<SCALAR, 3, 1>::UnitX());
	Eigen::AngleAxis<SCALAR> rz(yaw, Eigen::Matrix<SCALAR, 3, 1>::UnitZ());

	Eigen::Transform<SCALAR, 3, Eigen::Isometry> ret;
	ret.setIdentity(); // very imoprtant!!!
	ret.rotate(rx*ry*rz);
	ret.pretranslate(Eigen::Matrix<SCALAR, 3, 1>(x, y,z));
	return ret;
}

/**
 * Transfrom rt matrix to x,y,z, yaw(rz), pitch(ry), roll(rx) in radius.
 * do not use this function any more.
 */
template<typename SCALAR>
Eigen::Matrix<SCALAR, 6, 1> rt_mat2pose_eular(const Eigen::Matrix<SCALAR, 4, 4> &rt)
{
	Eigen::Matrix<SCALAR, 6, 1> pose;
	pose.block<3,1>(0,0) = rt.block<3, 1>(0, 3);
	//pose.block<3,1>(3,0) = rt.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

	// assume pitch, and roll is small
	pose(3) = atan2(rt(1), rt(0)); // rz
	pose(4) = atan2(-rt(2), sqrt(rt(1)*rt(1)+rt(0)*rt(0))); // ry 
	pose(5) = atan2(rt(6), rt(10)); // rx
	return pose;
}

#endif //GZ_TRANSFORM_H