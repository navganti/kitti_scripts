/* Copyright (c) 2018, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * University of Waterloo. All Rights Reserved.
 *
 * ############################################################################
 *                   ____
 *                  /    \
 *     ____         \____/
 *    /    \________//      ______
 *    \____/--/      \_____/     \\
 *            \______/----/      // __        __  __  __      __  ______
 *            //          \_____//  \ \  /\  / / /  \ \ \    / / / ____/
 *       ____//                      \ \/  \/ / / /\ \ \ \  / / / /__
 *      /      \\                     \  /\  / / /  \ \ \ \/ / / /____
 *     /       //                      \/  \/ /_/    \_\ \__/ /______/
 *     \______//                     LABORATORY
 *
 * ############################################################################
 *
 * File: convert_moose_poses_to_kitti.cpp
 * Desc: Convert Autonomoose GPS poses to the Kitti format.
 * Auth: Pranav Ganti, Jonathan Smith
 *
 * Copyright (c) 2018, Waterloo Autonomous Vehicles Laboratory (WAVELab),
 * University of Waterloo. All Rights Reserved.
 *
 * ###########################################################################
 */


#include <Eigen/Core>
#include <Eigen/Geometry>
#include <GeographicLib/Geocentric.hpp>

#include <cmath>
#include <iostream>

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
}

/** Calculate the transformation expressing the pose of the ENU (locallevel)
 * frame with respect to the ECEF (earth) frame.
 *
 * @param lla The latitude, longitude, and altitude coordinates of the point.
 * @return The transformation matrix expressing the pose of the ENU frame with
 * respect to the ECEF frame.
 */
Eigen::Affine3d convertLLAtoECEF(const Eigen::Vector3d &lla) {
    Eigen::Affine3d T_earth_locallevel = Eigen::Affine3d::Identity();

    GeographicLib::Geocentric earth = GeographicLib::Geocentric::WGS84();

    // Create temporary variables for calculation
    std::vector<double> R_ecef_locallevel(9, 0.0);
    double x_ecef_locallevel, y_ecef_locallevel, z_ecef_locallevel;

    // Extract LLA
    double latitude = lla(0);
    double longitude = lla(1);
    double altitude = lla(2);

    // Compute forward computation.
    earth.Forward(latitude,
                  longitude,
                  altitude,
                  x_ecef_locallevel,
                  y_ecef_locallevel,
                  z_ecef_locallevel,
                  R_ecef_locallevel);

    // Extract values.
    T_earth_locallevel(0, 0) = R_ecef_locallevel.at(0);
    T_earth_locallevel(0, 1) = R_ecef_locallevel.at(1);
    T_earth_locallevel(0, 2) = R_ecef_locallevel.at(2);
    T_earth_locallevel(0, 3) = x_ecef_locallevel;
    T_earth_locallevel(1, 0) = R_ecef_locallevel.at(3);
    T_earth_locallevel(1, 1) = R_ecef_locallevel.at(4);
    T_earth_locallevel(1, 2) = R_ecef_locallevel.at(5);
    T_earth_locallevel(1, 3) = y_ecef_locallevel;
    T_earth_locallevel(2, 0) = R_ecef_locallevel.at(6);
    T_earth_locallevel(2, 1) = R_ecef_locallevel.at(7);
    T_earth_locallevel(2, 2) = R_ecef_locallevel.at(8);
    T_earth_locallevel(2, 3) = z_ecef_locallevel;

    return T_earth_locallevel;
}

/** Calculates the pose of the body frame with respect to the locallevel frame.
 * This should just be a rotation.
 *
 * This rotation calculation is discussed here:
 * https://www.novatel.com/assets/Documents/Bulletins/apn037.pdf
 *
 * @param rpy The euler angles from the INSPVAX measurement.
 * @return The transformation matrix expressing the pose of the body frame with
 * respect to the locallevel frame.
 */
Eigen::Affine3d calculateTLocallevelBody(const Eigen::Vector3d &rpy) {
    Eigen::Affine3d T_locallevel_body = Eigen::Affine3d::Identity();

    double roll = rpy(1);
    double pitch = rpy(2);
    double yaw = rpy(3);

    double c_phi = std::cos(roll);
    double s_phi = std::sin(roll);
    double c_theta = std::cos(pitch);
    double s_theta = std::sin(pitch);
    double c_psi = std::cos(yaw);
    double s_psi = std::sin(yaw);

    T_locallevel_body(0, 0) = c_psi * c_phi - s_psi * s_theta * s_phi;
    T_locallevel_body(0, 1) = -s_psi * c_theta;
    T_locallevel_body(0, 2) = c_psi * s_phi + s_psi * s_theta * c_phi;
    T_locallevel_body(1, 0) = s_psi * c_phi + c_psi * s_theta * s_phi;
    T_locallevel_body(1, 1) = c_psi * c_theta;
    T_locallevel_body(1, 2) = s_psi * s_phi - c_psi * s_theta * c_phi;
    T_locallevel_body(2, 0) = -c_theta * s_phi;
    T_locallevel_body(2, 1) = s_theta;
    T_locallevel_body(2, 2) = c_theta * c_phi;

    return T_locallevel_body;
}

int main(int argc, char **argv) {
    // TODO @jajsmith: Read file, properly extract values, and write to new file
    // Need to read in original poses file, and extrinsic calibrations

    if (argc != 3) {
        std::cerr << std::endl
                  << "Usage ./convert_moose_to_kitti path_to_original poses "
                     "file path_to_extrinsic calibration"
                  << std::endl;

        return 1;
    }

    // The main steps are:
    // 1. Read in file data
    // 2. Extract LLA and RPY, convert to T_earth_locallevel. Locallevel is the
    // ENU frame located at the GPS IMU.
    // 3. Get pose of body (IMU) with respect to the locallevel. This is just a
    // rotation, determined by the GPS IMU itself. The origin point for the
    // GPS IMU and the ENU frame are the exact same.
    // 4. Compose with T_body_camera. This is an extrinsic calibration from
    // the moose calibration file.
    // 5. The first measurement is now our origin, of T_earth_camera. We now
    // left multiply each future T_earth_camera with the INVERSE of the first
    // T_earth_camera.
    // 6. Write out each transform to a new text file. Just the first 12 params.

    size_t num_measurements;
    Eigen::Affine3d T_ref_earth;
    for (size_t i = 0; i < num_measurements; ++i) {
        Eigen::Vector6d inspvax_measurement;

        // Read measurement from file

        // Extract lla and rpy from the INSPVAX measurement.
        Eigen::Vector3d lla = inspvax_measurement.block<3, 1>(0, 0);
        Eigen::Vector3d rpy = inspvax_measurement.block<3, 1>(3, 0);

        // Compute T_earth_locallevel
        Eigen::Affine3d T_earth_locallevel = convertLLAtoECEF(lla);
        Eigen::Affine3d T_locallevel_body = calculateTLocallevelBody(rpy);

        // Compose
        Eigen::Affine3d T_earth_body = T_earth_locallevel * T_locallevel_body;

        // Extract camera transform T_gpsimu_camera.
        // ...sorry for the inconsistent terminology.
        Eigen::Affine3d T_body_camera;

        // Read T_body_camera from file.

        // Compose and calculate inverse.
        Eigen::Affine3d T_earth_camera = T_earth_body * T_body_camera;

        if (i == 0) {
            T_ref_earth = T_earth_camera.inverse();
        }

        Eigen::Affine3d T_ref_camera = T_ref_earth * T_earth_camera;

        // Write T_ref_camera to file.
    }

    return 0;
}
