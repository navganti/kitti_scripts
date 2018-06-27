#include <iostream>
#include <limits>
#include <cmath>
#include <vector>
#include <dirent.h>
#include <algorithm>
#include <sstream>

#include "matrix.hpp"

const float lengths[] = {100, 200, 300, 400, 500, 600, 700, 800};
int32_t num_lengths = 8;

struct Errors {
    int32_t first_frame;
    float r_err;
    float t_err;
    float len;
    float speed;

    Errors(
      int32_t first_frame, float r_err, float t_err, float len, float speed)
        : first_frame(first_frame),
          r_err(r_err),
          t_err(t_err),
          len(len),
          speed(speed) {}
};

std::vector<Matrix> loadPoses(const std::string &file_name) {
    std::vector<Matrix> poses;
    FILE *fp = fopen(file_name.c_str(), "r");

    if (!fp) {
        return poses;
    }

    while (!feof(fp)) {
        Matrix P = Matrix::eye(4);

        if (fscanf(fp,
                   "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P.val[0][0],
                   &P.val[0][1],
                   &P.val[0][2],
                   &P.val[0][3],
                   &P.val[1][0],
                   &P.val[1][1],
                   &P.val[1][2],
                   &P.val[1][3],
                   &P.val[2][0],
                   &P.val[2][1],
                   &P.val[2][2],
                   &P.val[2][3]) == 12) {
            poses.push_back(P);
        }
    }

    fclose(fp);

    return poses;
}

std::vector<float> trajectoryDistances(const std::vector<Matrix> &poses) {
    std::vector<float> dist;
    dist.push_back(0.0f);

    for (int32_t i = 1; i < poses.size(); i++) {
        Matrix P1 = poses[i - 1];
        Matrix P2 = poses[i];

        auto dx = static_cast<float>(P1.val[0][3] - P2.val[0][3]);
        auto dy = static_cast<float>(P1.val[1][3] - P2.val[1][3]);
        auto dz = static_cast<float>(P1.val[2][3] - P2.val[2][3]);

        dist.push_back(dist[i - 1] + std::sqrt(dx * dx + dy * dy + dz * dz));
    }
    return dist;
}

int32_t lastFrameFromSegmentLength(std::vector<float> &dist,
                                   int32_t first_frame,
                                   float len) {
    for (int32_t i = first_frame; i < dist.size(); i++)
        if (dist[i] > dist[first_frame] + len) {
            return i;
        }

    return -1;
}

inline float rotationError(Matrix &pose_error) {
    auto a = static_cast<float>(pose_error.val[0][0]);
    auto b = static_cast<float>(pose_error.val[1][1]);
    auto c = static_cast<float>(pose_error.val[2][2]);
    auto d = static_cast<float>(0.5 * (a + b + c - 1.0));

    return std::acos(std::max(std::min(d, 1.0f), -1.0f));
}

inline float translationError(Matrix &pose_error) {
    auto dx = static_cast<float>(pose_error.val[0][3]);
    auto dy = static_cast<float>(pose_error.val[1][3]);
    auto dz = static_cast<float>(pose_error.val[2][3]);

    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<Errors> calcSequenceErrors(
  const std::vector<Matrix> &poses_gt,
  const std::vector<Matrix> &poses_result) {
    // error vector
    std::vector<Errors> err;

    // parameters
    int32_t step_size = 10;  // every second

    // pre-compute distances (from ground truth as reference)
    std::vector<float> dist = trajectoryDistances(poses_gt);

    // for all start positions do
    for (int32_t first_frame = 0; first_frame < poses_gt.size();
         first_frame += step_size) {
        // for all segment lengths do
        for (int32_t i = 0; i < num_lengths; i++) {
            // current length
            float len = lengths[i];

            // compute last frame
            int32_t last_frame =
              lastFrameFromSegmentLength(dist, first_frame, len);

            // continue, if sequence not long enough
            if (last_frame == -1)
                continue;

            // compute rotational and translational errors
            Matrix pose_delta_gt =
              Matrix::inv(poses_gt[first_frame]) * poses_gt[last_frame];
            Matrix pose_delta_result =
              Matrix::inv(poses_result[first_frame]) * poses_result[last_frame];
            Matrix pose_error = Matrix::inv(pose_delta_result) * pose_delta_gt;
            float r_err = rotationError(pose_error);
            float t_err = translationError(pose_error);

            // compute speed
            auto num_frames = static_cast<float>(last_frame - first_frame + 1);
            auto speed = static_cast<float>(len / (0.1 * num_frames));

            // write to file
            err.emplace_back(
              Errors(first_frame, r_err / len, t_err / len, len, speed));
        }
    }

    // return error vector
    return err;
}

void saveSequenceErrors(std::vector<Errors> &err, std::string file_name) {
    // open file
    FILE *fp;
    fp = fopen(file_name.c_str(), "w");

    // write to file
    for (auto it = err.begin(); it != err.end(); it++)
        fprintf(fp,
                "%d %f %f %f %f\n",
                it->first_frame,
                it->r_err,
                it->t_err,
                it->len,
                it->speed);

    // close file
    fclose(fp);
}

void savePlot(const std::vector<Matrix> &poses, const std::string &file_name) {
    // parameters
    int32_t step_size = 3;

    // open file
    FILE *fp = fopen(file_name.c_str(), "w");

    // save x/z coordinates of all frames to file
    for (int32_t i = 0; i < poses.size(); i += step_size) {
        fprintf(fp, "%f %f\n", poses[i].val[0][3], poses[i].val[2][3]);
    }

    // close file
    fclose(fp);
}

std::vector<int32_t> computeRoi(const std::vector<Matrix> &poses_gt,
                                const std::vector<Matrix> &poses_result) {
    float x_min = std::numeric_limits<int32_t>::max();
    float x_max = std::numeric_limits<int32_t>::min();
    float z_min = std::numeric_limits<int32_t>::max();
    float z_max = std::numeric_limits<int32_t>::min();

    for (auto it = poses_gt.begin(); it != poses_gt.end(); it++) {
        auto x = static_cast<float>(it->val[0][3]);
        auto z = static_cast<float>(it->val[2][3]);

        if (x < x_min) {
            x_min = x;
        }

        if (x > x_max) {
            x_max = x;
        }

        if (z < z_min) {
            z_min = z;
        }

        if (z > z_max) {
            z_max = z;
        }
    }

    for (auto it = poses_result.begin(); it != poses_result.end(); it++) {
        auto x = static_cast<float>(it->val[0][3]);
        auto z = static_cast<float>(it->val[2][3]);

        if (x < x_min) {
            x_min = x;
        }

        if (x > x_max) {
            x_max = x;
        }

        if (z < z_min) {
            z_min = z;
        }

        if (z > z_max) {
            z_max = z;
        }
    }

    auto dx = static_cast<float>(1.1 * (x_max - x_min));
    auto dz = static_cast<float>(1.1 * (z_max - z_min));
    auto mx = static_cast<float>(0.5 * (x_max + x_min));
    auto mz = static_cast<float>(0.5 * (z_max + z_min));
    auto r = static_cast<float>(0.5 * std::max(dx, dz));

    std::vector<int32_t> roi;

    roi.push_back(static_cast<int32_t>(mx - r));
    roi.push_back(static_cast<int32_t>(mx + r));
    roi.push_back(static_cast<int32_t>(mz - r));
    roi.push_back(static_cast<int32_t>(mz + r));

    return roi;
}

void plotPathPlot(const std::string &dir,
                  std::vector<int32_t> &roi,
                  const std::vector<std::string> &directories,
                  const std::string &trajectory_num) {
    // gnuplot file name
    char command[1024];
    std::string full_name = dir + "/" + trajectory_num;

    // Colourscheme selected from
    // http://colorbrewer2.org/#type=qualitative&scheme=Set1&n=8
    std::vector<std::string> colours{"#377EB8",
                                     "#4DAF4A",
                                     "#984EA3",
                                     "#FF7F00",
                                     "#FFFF33",
                                     "#A65628",
                                     "#F781BF"};

    // create png + eps
    for (int32_t i = 0; i < 2; i++) {
        // open file
        FILE *fp = fopen(full_name.c_str(), "w");

        // save gnuplot instructions
        if (i == 0) {
            fprintf(fp, "set term png size 900,900\n");
            fprintf(fp, "set output \"%s.png\"\n", trajectory_num.c_str());
        } else {
            fprintf(fp, "set term postscript eps enhanced color\n");
            fprintf(fp, "set output \"%s.eps\"\n", trajectory_num.c_str());
        }

        fprintf(fp, "set size ratio -1\n");

        int32_t xmax = roi[1] + 175;

        if (trajectory_num == "07") {
            xmax = roi[1];
        }

        fprintf(fp, "set xrange [%d:%d]\n", roi[0], xmax);
        fprintf(fp, "set yrange [%d:%d]\n", roi[2], roi[3]);
        fprintf(fp, "set xlabel \"x [m]\"\n");
        fprintf(fp, "set ylabel \"z [m]\"\n");
        fprintf(fp,
                "plot \"%s.txt\" using 1:2 lc rgb \"#FF0000\" title 'Ground "
                "Truth' w lines,",
                trajectory_num.c_str());

        for (size_t i = 0; i < directories.size(); ++i) {
            std::string test_name = directories.at(i) + "_" + trajectory_num;

            fprintf(fp,
                    "\"%s.txt\" using 1:2 lc rgb \"%s\" title '%s' w lines,",
                    test_name.c_str(),
                    colours.at(i).c_str(),
                    directories.at(i).c_str());
        }

        fprintf(fp,
                "\"< head -1 %s.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 "
                "lw 2 title 'Sequence Start' w points\n",
                trajectory_num.c_str());

        // close file
        fclose(fp);

        // run gnuplot => create png + eps
        sprintf(
          command, "cd %s; gnuplot %s", dir.c_str(), trajectory_num.c_str());
        system(command);
    }

    // create pdf and crop
    sprintf(command,
            "cd %s; ps2pdf %s.eps %s_large.pdf",
            dir.c_str(),
            trajectory_num.c_str(),
            trajectory_num.c_str());
    system(command);

    sprintf(command,
            "cd %s; pdfcrop %s_large.pdf %s.pdf",
            dir.c_str(),
            trajectory_num.c_str(),
            trajectory_num.c_str());
    system(command);

    sprintf(
      command, "cd %s; rm %s_large.pdf", dir.c_str(), trajectory_num.c_str());
    system(command);
}

void saveErrorPlots(const std::vector<Errors> &seq_err,
                    const std::string &plot_error_dir,
                    const std::string &test_name) {
    // file names
    char file_name_tl[1024];
    sprintf(
      file_name_tl, "%s/%s_tl.txt", plot_error_dir.c_str(), test_name.c_str());

    char file_name_rl[1024];
    sprintf(
      file_name_rl, "%s/%s_rl.txt", plot_error_dir.c_str(), test_name.c_str());

    char file_name_ts[1024];
    sprintf(
      file_name_ts, "%s/%s_ts.txt", plot_error_dir.c_str(), test_name.c_str());

    char file_name_rs[1024];
    sprintf(
      file_name_rs, "%s/%s_rs.txt", plot_error_dir.c_str(), test_name.c_str());

    // open files
    FILE *fp_tl = fopen(file_name_tl, "w");
    FILE *fp_rl = fopen(file_name_rl, "w");
    FILE *fp_ts = fopen(file_name_ts, "w");
    FILE *fp_rs = fopen(file_name_rs, "w");

    // for each segment length do
    for (int32_t i = 0; i < num_lengths; i++) {
        float t_err = 0;
        float r_err = 0;
        float num = 0;

        // for all errors do
        for (auto it = seq_err.begin(); it != seq_err.end(); it++) {
            if (fabs(it->len - lengths[i]) < 1.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_tl, "%f %f\n", lengths[i], t_err / num);
            fprintf(fp_rl, "%f %f\n", lengths[i], r_err / num);
        }
    }

    // for each driving speed do (in m/s)
    for (float speed = 2; speed < 25; speed += 2) {
        float t_err = 0;
        float r_err = 0;
        float num = 0;

        // for all errors do
        for (auto it = seq_err.begin(); it != seq_err.end(); it++) {
            if (fabs(it->speed - speed) < 2.0) {
                t_err += it->t_err;
                r_err += it->r_err;
                num++;
            }
        }

        // we require at least 3 values
        if (num > 2.5) {
            fprintf(fp_ts, "%f %f\n", speed, t_err / num);
            fprintf(fp_rs, "%f %f\n", speed, r_err / num);
        }
    }

    // close files
    fclose(fp_tl);
    fclose(fp_rl);
    fclose(fp_ts);
    fclose(fp_rs);
}

void plotErrorPlots(const std::string &dir,
                    const std::vector<std::string> &directories,
                    const std::string &trajectory_num) {
    char command[1024];

    // Colourscheme selected from
    // http://colorbrewer2.org/#type=qualitative&scheme=Set1&n=8
    std::vector<std::string> colours{"#377EB8",
                                     "#4DAF4A",
                                     "#984EA3",
                                     "#FF7F00",
                                     "#FFFF33",
                                     "#A65628",
                                     "#F781BF"};


    // for all four error plots do
    for (int32_t i = 0; i < 4; i++) {
        // create suffix
        char suffix[16];

        switch (i) {
            case 0: sprintf(suffix, "tl"); break;
            case 1: sprintf(suffix, "rl"); break;
            case 2: sprintf(suffix, "ts"); break;
            case 3: sprintf(suffix, "rs"); break;
        }

        // gnuplot file name
        char file_name[1024];
        char full_name[1024];
        sprintf(file_name, "%s_%s.gp", trajectory_num.c_str(), suffix);
        sprintf(full_name, "%s/%s", dir.c_str(), file_name);

        // create png + eps
        for (int32_t j = 0; j < 2; j++) {
            // open file
            FILE *fp = fopen(full_name, "w");

            // save gnuplot instructions
            if (j == 0) {
                fprintf(fp,
                        "set term png size 500,250 font \"Helvetica\" 11\n");
                fprintf(fp,
                        "set output \"%s_%s.png\"\n",
                        trajectory_num.c_str(),
                        suffix);
            } else {
                fprintf(fp, "set term postscript eps enhanced color\n");
                fprintf(fp,
                        "set output \"%s_%s.eps\"\n",
                        trajectory_num.c_str(),
                        suffix);
            }

            // start plot at 0
            fprintf(fp, "set size ratio 0.5\n");
            fprintf(fp, "set yrange [0:*]\n");

            // x label
            if (i <= 1)
                fprintf(fp, "set xlabel \"Path Length [m]\"\n");
            else
                fprintf(fp, "set xlabel \"Speed [km/h]\"\n");

            // y label
            if (i == 0 || i == 2)
                fprintf(fp, "set ylabel \"Translation Error [%%]\"\n");
            else
                fprintf(fp, "set ylabel \"Rotation Error [deg/m]\"\n");

            // plot error curves for all plots.
            for (size_t k = 0; k < directories.size(); ++k) {
                std::string test_name =
                  directories.at(k) + "_" + trajectory_num;

                if (k == 0) {
                    fprintf(fp,
                            "plot \"%s_%s.txt\" using ",
                            test_name.c_str(),
                            suffix);
                } else {
                    fprintf(fp,
                            "\"%s_%s.txt\" using ",
                            test_name.c_str(),
                            suffix);
                }


                switch (i) {
                    case 0:
                        fprintf(fp,
                                "1:($2*100) title '%s'",
                                directories.at(k).c_str());
                        break;
                    case 1:
                        fprintf(fp,
                                "1:($2*57.3) title '%s'",
                                directories.at(k).c_str());
                        break;
                    case 2:
                        fprintf(fp,
                                "($1*3.6):($2*100) title '%s'",
                                directories.at(k).c_str());
                        break;
                    case 3:
                        fprintf(fp,
                                "($1*3.6):($2*57.3) title '%s'",
                                directories.at(k).c_str());
                        break;
                }

                if (k != directories.size() - 1) {
                    fprintf(fp,
                            " lc rgb \"%s\" pt 4 w linespoints, ",
                            colours.at(k).c_str());
                } else {
                    fprintf(fp,
                            " lc rgb \"%s\" pt 4 w linespoints",
                            colours.at(k).c_str());
                }
            }

            // close file
            fclose(fp);

            // run gnuplot => create png + eps
            sprintf(command, "cd %s; gnuplot %s", dir.c_str(), file_name);
            system(command);
        }

        // create pdf and crop
        sprintf(command,
                "cd %s; ps2pdf %s_%s.eps %s_%s_large.pdf",
                dir.c_str(),
                trajectory_num.c_str(),
                suffix,
                trajectory_num.c_str(),
                suffix);
        system(command);

        sprintf(command,
                "cd %s; pdfcrop %s_%s_large.pdf %s_%s.pdf",
                dir.c_str(),
                trajectory_num.c_str(),
                suffix,
                trajectory_num.c_str(),
                suffix);
        system(command);

        sprintf(command,
                "cd %s; rm %s_%s_large.pdf",
                dir.c_str(),
                trajectory_num.c_str(),
                suffix);
        system(command);
    }
}

std::vector<int32_t> calculateMaxROI(
  const std::vector<std::vector<int32_t>> &all_rois) {
    // Out of all the ROIs, need to figure out which one has the:
    // min x, max x, min y, max y.
    int32_t min_x, max_x, min_y, max_y;

    // Initialize values.
    min_x = all_rois.at(0).at(0);
    max_x = all_rois.at(0).at(1);
    min_y = all_rois.at(0).at(2);
    max_y = all_rois.at(0).at(3);

    // Go through all values, and find the maximum and minimum of each element
    for (size_t i = 1; i < all_rois.size(); ++i) {
        if (all_rois.at(i).at(0) < min_x) {
            min_x = all_rois.at(i).at(0);
        }

        if (all_rois.at(i).at(1) > max_x) {
            max_x = all_rois.at(i).at(1);
        }

        if (all_rois.at(i).at(2) < min_y) {
            min_y = all_rois.at(i).at(2);
        }

        if (all_rois.at(i).at(3) > max_y) {
            max_y = all_rois.at(i).at(3);
        }
    }

    std::vector<int32_t> max_roi{min_x, max_x, min_y, max_y};

    return max_roi;
}

bool eval(std::string trajectory_num,
          const std::string &gt_file,
          const std::vector<std::string> &test_files,
          const std::vector<std::string> &directories) {
    // ground truth and result directories
    std::string result_dir = trajectory_num + "/results";
    std::string error_dir = result_dir + "/errors";
    std::string plot_path_dir = result_dir + "/plot_path";
    std::string plot_error_dir = result_dir + "/plot_error";

    // create output directories
    system(("mkdir -p " + error_dir).c_str());
    system(("mkdir -p " + plot_path_dir).c_str());
    system(("mkdir -p " + plot_error_dir).c_str());

    // total errors
    std::vector<std::vector<Errors>> all_errors;
    std::vector<std::vector<int32_t>> all_rois;

    // read ground truth and save to file
    std::vector<Matrix> poses_gt = loadPoses(gt_file);
    savePlot(poses_gt, plot_path_dir + "/" + trajectory_num + ".txt");

    // Read all tests
    std::vector<std::vector<Matrix>> poses_tests;
    for (const auto &test : test_files) {
        std::vector<Matrix> test_result = loadPoses(test);

        poses_tests.emplace_back(test_result);
    }

    // Process each of the tests
    int count = 0;
    for (const auto &test : poses_tests) {
        std::string test_file = directories.at(static_cast<size_t>(count));
        std::string test_name = test_file + "_" + trajectory_num;
        // Print status
        std::cout << "Processing: " << test_file << std::endl;

        // check for errors
        if (poses_gt.size() == 0 || test.size() != poses_gt.size()) {
            std::cerr << "ERROR: Couldn't read poses of: " << test_file
                      << std::endl;
            std::cerr << "Ground truth size: " << poses_gt.size() << std::endl;
            std::cerr << "Results size: " << test.size() << std::endl;
            return false;
        }

        // Compute and save sequence errors.
        std::vector<Errors> seq_err = calcSequenceErrors(poses_gt, test);
        std::ostringstream error_path;
        error_path << error_dir << "/" << test_name << ".txt";
        saveSequenceErrors(seq_err, error_path.str());

        // add to total errors
        std::vector<Errors> test_error;
        test_error.insert(test_error.end(), seq_err.begin(), seq_err.end());
        all_errors.emplace_back(test_error);

        // Save path plots for each file.
        std::ostringstream path_plot;
        path_plot << plot_path_dir << "/" << test_name << ".txt";
        savePlot(test, path_plot.str());

        // Compute ROI for each test
        std::vector<int32_t> roi = computeRoi(poses_gt, test);
        all_rois.emplace_back(roi);

        // Save individual error plot text.
        saveErrorPlots(seq_err, plot_error_dir, test_name);

        ++count;
    }

    std::vector<int32_t> max_roi = calculateMaxROI(all_rois);

    // Plot all paths and errors
    plotPathPlot(plot_path_dir, max_roi, directories, trajectory_num);

    // // save + plot individual errors
    plotErrorPlots(plot_error_dir, directories, trajectory_num);

    // success
    return true;
}

// Directory parsing examples from
// https://stackoverflow.com/questions/5043403/listing-only-folders-in-directory
// https://stackoverflow.com/questions/612097/how-can-i-get-the-list-of-files-in-a-directory-using-c-or-c
std::vector<std::string> getAllDirectories(const std::string &trajectory_num) {
    std::vector<std::string> directories;

    const char *PATH = trajectory_num.c_str();
    DIR *dir = opendir(PATH);

    struct dirent *entry = readdir(dir);

    while (entry != nullptr) {
        if (entry->d_type == DT_DIR && std::string{entry->d_name} != "." &&
            std::string{entry->d_name} != ".." &&
            std::string{entry->d_name} != "results") {
            directories.emplace_back(std::string{entry->d_name});
        }

        entry = readdir(dir);
    }

    // Sort directories alphabetically.
    std::sort(directories.begin(), directories.end());

    return directories;
}

std::vector<std::string> generateTestNames(
  const std::vector<std::string> &directories,
  const std::string &trajectory_num) {
    std::vector<std::string> test_files;

    for (const auto &dir : directories) {
        std::ostringstream test;
        test << trajectory_num << "/" << dir << "/" << dir << "_"
             << trajectory_num << ".txt";

        test_files.emplace_back(test.str());
    }

    return test_files;
}

int main(int32_t argc, char *argv[]) {
    // We only need the trajectory number as an argument!
    if (argc != 2) {
        std::cerr << "Usage: ./eval_odometry trajectory_num" << std::endl;
        return 1;
    }

    // read arguments
    std::string trajectory_num = argv[1];
    std::string gt_file = trajectory_num + "/" + trajectory_num + ".txt";

    // Get all directories
    std::vector<std::string> directories =
      getAllDirectories(trajectory_num + "/");

    // Create all test files
    std::vector<std::string> test_files =
      generateTestNames(directories, trajectory_num);

    bool success = false;
    // run evaluation
    if (!test_files.empty()) {
        success = eval(trajectory_num, gt_file, test_files, directories);
    }

    if (success) {
        std::cout << "Trajectories generated successfully!" << std::endl;
    } else {
        std::cerr << "ERROR: Trajectories not generated successfully!"
                  << std::endl;
    }

    return 0;
}
