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
#include <Eigen/StdVector>
#include <GeographicLib/Geocentric.hpp>

#include <cmath>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>

namespace Eigen {
using Vector6d = Matrix<double, 6, 1>;
using Matrix4d = Matrix<double, 4, 4>;
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

    double roll = rpy(0);
    double pitch = rpy(1);
    double yaw = rpy(2);

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
    // Need to read in original poses file

    if (argc != 2) {
        std::cerr
          << std::endl
          << "Usage ./convert_moose_to_kitti path_to_original_poses_file"
          << std::endl;

        return 1;
    }

    // Read in pose data from CSV
    std::string poses_filename = argv[1];
    std::cout << "Extracting poses from file: " << poses_filename << std::endl;

    // TODO (Jonathan): Read file and extract values
    std::vector<Eigen::Vector6d> measurements;

    std::ifstream poses_file(poses_filename);
    std::string line;
    while (std::getline(poses_file, line)) {
        std::istringstream iss(line);
        double lat, lon, alt, rol, pit, yaw, ign;
        if (!(iss >> ign >> lat >> lon >> alt >> ign >> ign >> ign >> rol >>
              pit >> yaw >> ign >> ign >> ign)) {
            std::cout << "Error reading line: " << line << std::endl;
            continue;
        }

        Eigen::Vector6d line_measurement;
        line_measurement(0) = lat;
        line_measurement(1) = lon;
        line_measurement(2) = alt;
        line_measurement(3) = rol;
        line_measurement(4) = pit;
        line_measurement(5) = yaw;
        measurements.push_back(line_measurement);
    }

    std::cout << "Read all measurements: " << measurements.size() << std::endl;

    // TODO (Jonathan): Get extrinsic calibration from yaml files in calibration
    // publisher
    // "go through the chain from GPS_IMU -> LIDAR -> FRONT_CAM -> LEFT_CAM"
    // Retrieved from
    // https://github.com/wavelab/calibration_publisher/blob/master/calibrations/moose/2017_10_01.yaml
    Eigen::Affine3d T_LIDAR_GPSIMU, T_LIDAR_FCAMERA, T_FLCAMERA_FCAMERA;

    T_LIDAR_GPSIMU.matrix() << 0.00198557728013, 0.999851298907,
      0.0171300191978, -1.12584371634, -0.999995766779, 0.00194884664695,
      0.00216065276941, 0.0794562050592, 0.00212694769751, -0.0171342368257,
      0.999850935901, -1.31, 0.0, 0.0, 0.0, 1.0;

    T_LIDAR_FCAMERA.matrix() << 0.0442993181598, -0.0101754527732,
      0.998966481205, 0.5415, -0.998979231642, 0.00839248150057,
      0.0443853692301, -0.03, -0.00883544894162, -0.999913009424,
      -0.00979328478481, -0.6202, 0.0, 0.0, 0.0, 1.0;

    T_FLCAMERA_FCAMERA.matrix() << 0.999332564621, -0.0199978793068,
      -0.0305697581316, 0.51369, 0.0204171747396, 0.999700858999,
      0.0134659381941, 0.0051933, 0.0302913232569, -0.0140810986441,
      0.999441923473, -0.00031466, 0.0, 0.0, 0.0, 1.0;

    Eigen::Affine3d T_body_camera =
      T_LIDAR_GPSIMU.inverse() * T_LIDAR_FCAMERA * T_FLCAMERA_FCAMERA.inverse();


    // The main steps are:
    // 1. Read in file data
    // 2. Extract LLA and RPY, convert to T_earth_locallevel. Locallevel is
    // the
    // ENU frame located at the GPS IMU.
    // 3. Get pose of body (IMU) with respect to the locallevel. This is just
    // a
    // rotation, determined by the GPS IMU itself. The origin point for the
    // GPS IMU and the ENU frame are the exact same.
    // 4. Compose with T_body_camera. This is an extrinsic calibration from
    // the moose calibration file.
    // 5. The first measurement is now our origin, of T_earth_camera. We now
    // left multiply each future T_earth_camera with the INVERSE of the first
    // T_earth_camera.
    // 6. Write out each transform to a new text file. Just the first 12
    // params.

    size_t num_measurements = measurements.size();
    Eigen::Affine3d T_ref_earth;

    // Open file
    std::ofstream output_file;
    output_file.open("converted_poses.txt", std::ofstream::out | std::ofstream::trunc);
    output_file << std::fixed << std::setprecision(8);

    for (size_t i = 0; i < num_measurements; ++i) {
        Eigen::Vector6d inspvax_measurement;

        // Read measurement from file
        inspvax_measurement = measurements[i];

        // Extract lla and rpy from the INSPVAX measurement.
        Eigen::Vector3d lla = inspvax_measurement.block<3, 1>(0, 0);
        Eigen::Vector3d rpy = inspvax_measurement.block<3, 1>(3, 0);

        // Compute T_earth_locallevel
        Eigen::Affine3d T_earth_locallevel = convertLLAtoECEF(lla);
        Eigen::Affine3d T_locallevel_body = calculateTLocallevelBody(rpy);

        // Compose
        Eigen::Affine3d T_earth_body = T_earth_locallevel * T_locallevel_body;


        // Compose and calculate inverse.
        Eigen::Affine3d T_earth_camera = T_earth_body * T_body_camera;

        if (i == 0) {
            T_ref_earth = T_earth_camera.inverse();
        }

        Eigen::Affine3d T_ref_camera = T_ref_earth * T_earth_camera;

        if (output_file.is_open()) {
            // Write T_ref_camera to file.
            output_file << T_ref_camera(0, 0) << " " << T_ref_camera(0, 1) << " "
                        << T_ref_camera(0, 2) << " " << T_ref_camera(0, 3) << " "
                        << T_ref_camera(1, 0) << " " << T_ref_camera(1, 1) << " "
                        << T_ref_camera(1, 2) << " " << T_ref_camera(1, 3) << " "
                        << T_ref_camera(2, 0) << " " << T_ref_camera(2, 1) << " "
                        << T_ref_camera(2, 2) << " " << T_ref_camera(2, 3) << std::endl;
        }
    }

    output_file.close();

    return 0;
}
