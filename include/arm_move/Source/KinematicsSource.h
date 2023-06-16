#include <ros/ros.h>
#include <Eigen/Dense>

/**
 * Helper class that deals with the solutions for the direct and inverse kinametics.
 * provides different utility methods for different conversions.
 */
class Kinematics
{
public:
    // Computes the inverse kinematics of the arm.
    static void ik(double *dimensions, double *tcp, double *joints);
    // Computes the direct kinematics of the arm.
    static void dk(double *dimensions, double *joints, double *tcp);

    // Converts the Euler angels in a rotation matrix.
    static void euler_to_rotm(double rx, double ry, double rz, Eigen::Matrix3d *rotation);
    // Converts a rotation matrix into Euler angels.
    static void rotm_to_euler(Eigen::Matrix3d rotation, double *rx, double *ry, double *rz);
    // Extracts the Euler angels from the rotation matrix for the wrist joint angels.
    static void rotm_to_euler_wrist(Eigen::Matrix3d rotation, double *joints);

    // Converts a rotation matrix to a quaternion.
    static void rotm_to_quaternion(Eigen::Matrix3d rotation, double *quaternion);
    // Converts a quaternion to a rotation matrix.
    static void quaternion_to_rotm(double *quaternion, Eigen::Matrix3d *rotation);

    // Spherical linear interpolation, interpoltion between two angels.
    static void slerp(double *qs, double *qe, double q[4], double t);

    // Homogenous matrix.
    static void translation_matrix(double rx, double ry, double rz, double x, double y, double z, Eigen::Matrix4d *trans_matrix);

    // Sign of the non-zero paramrter.
    static int joint_sign(double j1, double j2);

    // Convert a quaternion to Euler angles.
    static void quaternion_to_euler(double* q, double *rx, double *ry, double *rz);
    // Convert a Euler angles to quaternion.
    static void euler_to_quaternion(double rx, double ry, double rz, double* q);
};

#pragma once
