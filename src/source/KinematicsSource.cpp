#include <arm_move/Source/KinematicsSource.h>

/**
 * @brief Computes the inverse kinematics of the arm.
 *
 * @param[in] dimensions distance from each joint 
 * @param[in] tcp position and orientation of the TCP
 * @param[out] joints angle of rotation for each joint
 */
void Kinematics::ik(double *dimensions, double *tcp, double *joints)
{
    double alpha = 0, beta = 0, alpha3 = 0;

    int waist_tolerance = 2.2;

    double l2 = dimensions[2];
    double l3 = dimensions[3] + dimensions[4];
    double b_2 = dimensions[0] + dimensions[1];
    double l6 = dimensions[5];

    double wx, wy, wz;

    Eigen::Matrix4d R,Rx,Ry,Rz;
    translation_matrix(0,0,tcp[5],0,0,0,&Rz);
    translation_matrix(0,tcp[4],0,0,0,0,&Ry);
    translation_matrix(tcp[3],0,0,0,0,0,&Rx);
    R = Rz*Ry*Rx;

    wx = tcp[0] - l6*R.block(0,0,3,3).inverse()(0,2);
    wy = tcp[1] - l6*R.block(0,0,3,3).inverse()(1,2);
    wz = tcp[2] - l6*R.block(0,0,3,3).inverse()(2,2);

    double d_w = sqrt(wx*wx + wy*wy);
    double h_w = abs(wz - b_2);

    double l = sqrt(d_w*d_w + h_w*h_w);

    if(wx == 0 && wy == 0)
    {
        joints[0] = 0;
        joints[1] = 0;
        joints[2] = 0;
    }
    else
    {
        joints[0] = atan2(wy,wx);

        if(joints[0] < 0)
            joints[0] = -M_PI - joints[0];   
        else
            joints[0] = M_PI - joints[0];

        if(l2+l3 <= l)
        {
            ROS_WARN_STREAM("Kinematic edge case!");
            joints[2] = 0;
            joints[1] = M_PI_2 - acos(((l2+l3)*(l2+l3) + d_w*d_w - h_w*h_w)/(2*(l2+l3)*d_w));
        }
        else
        {
            alpha = acos((l*l + d_w*d_w - h_w*h_w)/(2*l*d_w));
            beta = acos((l2*l2 + l*l - l3*l3)/(2*l2*l));

            alpha3 = acos((l2*l2 + l3*l3 - l*l)/(2*l2*l3));

            if(alpha + beta == M_PI_2)
            {
                joints[1] = 0;
                joints[2] = -M_PI + alpha3;
            }
            else
            {
                joints[2] = M_PI - alpha3;
                joints[1] = M_PI_2 - alpha - beta;
                if(wz < waist_tolerance)
                    joints[1] = M_PI_2 - alpha + beta;
            }
        }
    }

    if(joints[1] > M_PI_2)
        joints[1] = M_PI - joints[1];

    if(joints[1] == 0)
        joints[1] = -joint_sign(tcp[1],tcp[2])*joints[1];

    if(joints[1] == 0 && joints[0] == 0)
        joints[2] = -joint_sign(tcp[1],tcp[2])*joints[2];

    rotm_to_euler_wrist(R.block(0,0,3,3), joints);

    if(joints[2] == 0 && joints[1] == 0 && joints[0] == 0)
        joints[4] = -joint_sign(tcp[1],tcp[2])*joints[4];
}

/**
 * @brief Computes the direct kinematics of the arm.
 *
 * @param[in] dimensions distance from each joint 
 * @param[in] tcp position and orientation of the TCP
 * @param[out] joints angle of rotation for each joint
 */
void Kinematics::dk(double *dimensions, double *joints, double *tcp)
{
    Eigen::Vector4d mask;
    mask << 0, 0, 0, 1;

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity(4,4);

    Eigen::Matrix4d T_aux;
    
    translation_matrix(0,0,0,0,0,dimensions[0], &T_aux);
    T = T_aux*T;

    translation_matrix(0,0,joints[0],0,0,dimensions[1], &T_aux);
    T = T_aux*T;

    translation_matrix(0,joints[1],0,0,0,dimensions[2], &T_aux);
    T = T_aux*T;

    translation_matrix(0,joints[2],0,0,0,dimensions[3], &T_aux);
    T = T_aux*T;

    translation_matrix(0,0,joints[3],0,0,dimensions[4], &T_aux);
    T = T_aux*T;

    translation_matrix(0,joints[4],0,0,0,dimensions[5], &T_aux);
    T = T_aux*T;

    translation_matrix(0,0,joints[5],0,0,0, &T_aux);
    T = T_aux*T;

    Eigen::Vector4d position;
    position = T*mask;
    position.segment(0,3) = T.block(0,0,3,3).inverse()*position.segment(0,3);

    double rotation[3];
    rotm_to_euler(T.block(0,0,3,3),&rotation[0],&rotation[1],&rotation[2]);

    tcp[0] = position(0);
    tcp[1] = position(1);
    tcp[2] = position(2);
    tcp[3] = rotation[0];
    tcp[4] = rotation[1];
    tcp[5] = rotation[2];
} 

/**
 * @brief Converts the Euler angels in a rotation matrix.
 *
 * @param[in] rx rotation on x 
 * @param[in] ry rotation on y
 * @param[in] rz rotation on z 
 * @param[out] rotation rotation matrix with the given rotations
 */
void Kinematics::euler_to_rotm(double rx, double ry, double rz, Eigen::Matrix3d *rotation)
{
    (*rotation)(0,0) = cos(ry)*cos(rz);
    (*rotation)(0,1) = cos(rz)*sin(rx)*sin(ry) - cos(rx)*sin(rz);
    (*rotation)(0,2) = sin(rx)*sin(rz) + cos(rx)*cos(rz)*sin(ry);
    (*rotation)(1,0) = cos(ry)*sin(rz);
    (*rotation)(1,1) = cos(rx)*cos(rz) + sin(rx)*sin(ry)*sin(rz);
    (*rotation)(1,2) = cos(rx)*sin(ry)*sin(rz) - cos(rz)*sin(rx);
    (*rotation)(2,0) = -sin(ry);
    (*rotation)(2,1) = cos(ry)*sin(rx);
    (*rotation)(2,2) = cos(rx)*cos(ry);
}

/**
 * @brief Converts a rotation matrix into Euler angels.
 *
 * @param[in] rotation rotation matrix 
 * @param[out] rx rotation on x 
 * @param[out] ry rotation on y
 * @param[out] rz rotation on z 
 */
void Kinematics::rotm_to_euler(Eigen::Matrix3d rotation, double *rx, double *ry, double *rz)
{
    if(rotation(2,0) != -1 && rotation(2,0) != 1)
    {
        *ry = -asin(rotation(2,0));
        *rx = atan2(rotation(2,1)/cos(*ry),rotation(2,2)/cos(*ry));
        *rz = atan2(rotation(1,0)/cos(*ry),rotation(0,0)/cos(*ry));
    }
    else
    {
        *rz = 0;
        if(rotation(2,0) == -1)
        {
            *ry = M_PI_2;
            *rx = atan2(rotation(0,1),rotation(0,2));
        }
        else
        {   
            *ry = -M_PI_2;
            *rx = atan2(-rotation(0,1),-rotation(0,2));    
        }
    }
}

/**
 * @brief Extracts the Euler angels from the rotation matrix for the wrist joint angels.
 *
 * @param[in] rotation rotation matrix of the wrist
 * @param[out] joints all the rotation for the joints, here the last 3 are computed
 */
void Kinematics::rotm_to_euler_wrist(Eigen::Matrix3d rotation, double *joints)
{
    Eigen::Matrix4d R,Rx,Ry,Rz;
    translation_matrix(0,joints[2],0,0,0,0,&Rz);
    translation_matrix(0,joints[1],0,0,0,0,&Ry);
    translation_matrix(0,0,joints[0],0,0,0,&Rx);
    R = Rz*Ry*Rx;

    rotation = rotation*R.block(0,0,3,3).inverse();

    if(rotation(2,2) == 1 || rotation(2,2) == -1)
    {
        joints[4] = atan2(sqrt(1-rotation(2,2)*rotation(2,2)),rotation(2,2));
        joints[3] = 0;
        joints[5] = atan2(rotation(1,0),rotation(0,0));
    }
    else
    {
        joints[4] = atan2(sqrt(1-rotation(2,2)*rotation(2,2)),rotation(2,2));
        joints[3] = atan2(rotation(2,1),-rotation(2,0));
        joints[5] = atan2(rotation(1,2),rotation(0,2));
    }
}

/**
 * @brief Converts a rotation matrix to a quaternion.
 *
 * @param[in] rotation rotation matrix to be converted
 * @param[out] quaternion converted quaternion
 */
void Kinematics::rotm_to_quaternion(Eigen::Matrix3d rotation, double *quaternion)
{
    quaternion[3] = 0.5 * sqrt(1 + rotation(0,0) + rotation(1,1) + rotation(2,2));
    quaternion[0] = 1/(4 * quaternion[3]) * (rotation(2,1) - rotation(1,2));
    quaternion[1] = 1/(4 * quaternion[3]) * (rotation(0,2) - rotation(2,0));
    quaternion[2] = 1/(4 * quaternion[3]) * (rotation(1,0) - rotation(0,1));
}

/**
 * @brief Converts a quaternion to a rotation matrix.
 *
 * @param[in] quaternion quaternion to be converted
 * @param[out] rotation converted rotation matrix 
 */
void Kinematics::quaternion_to_rotm(double *quaternion, Eigen::Matrix3d *rotation)
{
    (*rotation)(0,0) = 1 - 2 * quaternion[1]*quaternion[1] - 2 * quaternion[2]*quaternion[2];
    (*rotation)(0,1) = 2*quaternion[0]*quaternion[1] - 2*quaternion[2]*quaternion[3];
    (*rotation)(0,2) = 2*quaternion[0]*quaternion[2] + 2*quaternion[1]*quaternion[3]; 

    (*rotation)(1,0) = 2*quaternion[0]*quaternion[1] + 2*quaternion[2]*quaternion[3];
    (*rotation)(1,1) = 1 - 2 * quaternion[0]*quaternion[0] - 2 * quaternion[2]*quaternion[2];
    (*rotation)(1,2) = 2*quaternion[1]*quaternion[2] - 2*quaternion[0]*quaternion[3];

    (*rotation)(2,0) = 2*quaternion[0]*quaternion[2] - 2*quaternion[1]*quaternion[3];
    (*rotation)(2,1) = 2*quaternion[1]*quaternion[2] + 2*quaternion[0]*quaternion[3];
    (*rotation)(2,2) = 1 - 2 * quaternion[0]*quaternion[0] - 2 * quaternion[1]*quaternion[1];;
}

/**
 * @brief Spherical linear interpolation, interpoltion between two angels.
 *
 * @param[in] qs start quaternion
 * @param[in] qe end quaternion
 * @param[in] t variable or the interpolation
 * @param[out] q interpolated angle
 */
void Kinematics::slerp(double *qs, double *qe, double q[4], double t)
{
	double cosHalfTheta = qs[3] * qe[3] + qs[0] * qe[0] + qs[1] * qe[1] + qs[2] * qe[2];

	if (abs(cosHalfTheta) >= 1.0)
    {
		q[3] = qs[3];
        q[0] = qs[0];
        q[1] = qs[1];
        q[2] = qs[2];
        return;
	}
	// Calculate temporary values.
	double halfTheta = acos(cosHalfTheta);
	double sinHalfTheta = sqrt(1.0 - cosHalfTheta*cosHalfTheta);
	// if theta = 180 degrees then result is not fully defined
	// we could rotate around any axis normal to qa or qb
	if (fabs(sinHalfTheta) < 0.001)
    { // fabs is floating point absolute
		q[3] = qs[3] * 0.5 + qe[3] * 0.5;
		q[0] = qs[0] * 0.5 + qe[0] * 0.5;
		q[1] = qs[1] * 0.5 + qe[1] * 0.5;
		q[2] = qs[2] * 0.5 + qe[2] * 0.5;
	}
	double ratioA = sin((1 - t) * halfTheta) / sinHalfTheta;
	double ratioB = sin(t * halfTheta) / sinHalfTheta; 
	//calculate Quaternion.
    q[3] = qs[3] * ratioA + qe[3] * ratioB;
    q[0] = qs[0] * ratioA + qe[0] * ratioB;
    q[1] = qs[1] * ratioA + qe[1] * ratioB;
    q[2] = qs[2] * ratioA + qe[2] * ratioB;
}

/**
 * @brief Homogenous matrix.
 *
 * @param[in] rx rotation on x
 * @param[in] ry rotation on y
 * @param[in] rz rotation on z
 * @param[in] x translation on x
 * @param[in] y translation on y
 * @param[in] z translation on z
 * @param[out] trans_matrix final homogenous matrix
 */
void Kinematics::translation_matrix(double rx, double ry, double rz, double x, double y, double z, Eigen::Matrix4d *trans_matrix)
{
    Eigen::Matrix4d Rx;
    Rx << 1, 0,       0,        0,
          0, cos(rx), -sin(rx), 0,
          0, sin(rx), cos(rx),  0,
          0, 0,       0,        0;

    Eigen::Matrix4d Ry;
    Ry <<  cos(ry), 0, sin(ry), 0,
           0,       1, 0,       0,   
          -sin(ry), 0, cos(ry), 0,
           0,       0, 0,       0;

    Eigen::Matrix4d Rz;
    Rz << cos(rz), -sin(rz), 0, 0,
          sin(rz), cos(rz),  0, 0,
          0,       0,        1, 0,
          0,       0,        0, 0;            

    (*trans_matrix) = Rz*Ry*Rx;
    (*trans_matrix)(0,3) = x;
    (*trans_matrix)(1,3) = y;
    (*trans_matrix)(2,3) = z;
    (*trans_matrix)(3,3) = 1;
}

/**
 * @brief Sign of the non-zero paramrter.
 * Used in edge cases
 *
 * @param[in] j1 number 
 * @param[in] j2 number
 * 
 * @return sign  
 */
int Kinematics::joint_sign(double j1, double j2)
{
    if(j1 != 0 && j2 != 0)
        return (j1*j2)/abs(j1*j2);
    
    if(j1 != 0 && j2 == 0)
        return j1/abs(j1);

    if(j1 == 0 && j2 != 0)
        return j2/abs(j2);

    if(j1 == 0 && j2 == 0)
        return 1;

    return 0;
}

/**
 * @brief Convert a quaternion to Euler angles.
 *
 * @param[in] q quaternion to be converted 
 * @param[out] rx rotation on x
 * @param[out] ry rotation on y
 * @param[out] rz rotation on z
 */
void Kinematics::quaternion_to_euler(double* q, double *rx, double *ry, double *rz) 
{
    (*rx) = atan2(2*(q[0]*q[1]+q[3]*q[2]), q[3]*q[3]+q[0]*q[0]-q[1]*q[1]-q[2]*q[2]);
    (*ry) = asin(-2*(q[0]*q[2]-q[3]*q[1]));
    (*rz) = atan2(2*(q[1]*q[2]+q[3]*q[0]), q[3]*q[3]-q[0]*q[0]-q[1]*q[1]+q[2]*q[2]);
}

/**
 * @brief Convert a Euler angles to quaternion.
 *
 * @param[in] rx rotation on x
 * @param[in] ry rotation on y
 * @param[in] rz rotation on z
 * @param[out] q converted quaternion  
 */
void Kinematics::euler_to_quaternion(double rx, double ry, double rz, double* q)
{
    // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles

    double cy = cos(rx * 0.5);
    double sy = sin(rx * 0.5);
    double cp = cos(ry * 0.5);
    double sp = sin(ry * 0.5);
    double cr = cos(rz * 0.5);
    double sr = sin(rz * 0.5);

    q[3] = cr * cp * cy + sr * sp * sy;
    q[0] = sr * cp * cy - cr * sp * sy;
    q[1] = cr * sp * cy + sr * cp * sy;
    q[2] = cr * cp * sy - sr * sp * cy;
}