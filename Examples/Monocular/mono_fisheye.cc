/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>

#include <opencv2/core/core.hpp>

#include <System.h>
#include "toc.h"

using namespace std;

std::vector<double> LoadTimestamps(const std::string &path_to_timestamps);
void SaveImagePaths(const std::string &path_to_images, const size_t num_of_imgs,
                    std::vector<std::string> *const image_paths);

int main(int argc, char **argv) {
  if (argc != 3) {
    cerr << endl
         << "Usage: ./mono_tum path_to_vocabulary path_to_settings " << endl;
    return 1;
  }

  // Retrieve paths to images
  vector<string> vstrImageFilenames;
  vector<double> vTimestamps;
  //   string strFile = string(argv[3]) + "/rgb.txt";
  string file_timestamp =
      "/home/kang/Dataset/with_calib/slam_video/output/timestamps.txt";
  string folder_image_path =
      "/home/kang/Dataset/with_calib/slam_video/output/front/bgr";
  vTimestamps = LoadTimestamps(file_timestamp);
  std::cout << "file timestamp: " << vTimestamps.size() << std::endl;
  SaveImagePaths(folder_image_path, vTimestamps.size(), &vstrImageFilenames);

  int nImages = vstrImageFilenames.size();
  //   for (auto &test : vstrImageFilenames) {
  //     std::cout << "image: " << test << std::endl;
  //   }

  // Create SLAM system. It initializes all system threads and gets ready to
  // process frames.
  ORB_SLAM3::System SLAM(argv[1], argv[2], ORB_SLAM3::System::MONOCULAR, true);

  // Vector for tracking time statistics
  vector<float> vTimesTrack;
  vTimesTrack.resize(nImages);

  cout << endl << "-------" << endl;
  cout << "Start processing sequence ..." << endl;
  cout << "Images in the sequence: " << nImages << endl << endl;

  // Main loop
  cv::Mat im;
  for (int ni = 200; ni < nImages; ni++) {
    // Read image from file
    im = cv::imread(vstrImageFilenames[ni], cv::IMREAD_UNCHANGED);
    double tframe = vTimestamps[ni];

    if (im.empty()) {
      cerr << endl
           << "Failed to load image at: " << string(argv[3]) << "/"
           << vstrImageFilenames[ni] << endl;
      return 1;
    }

    // Pass the image to the SLAM system
    TicToc tic;
    SLAM.TrackMonocular(im, tframe);

    const double elapsed_time = tic.TocSecond();  // unit: s
    // std::cout << "Tracking one frame: " << elapsed_time << std::endl;

    // Wait to load the next frame
    usleep((10) * 1e3);
  }

  // Stop all threads
  SLAM.Shutdown();

  // Tracking time statistics
  sort(vTimesTrack.begin(), vTimesTrack.end());
  float totaltime = 0;
  for (int ni = 0; ni < nImages; ni++) {
    totaltime += vTimesTrack[ni];
  }
  cout << "-------" << endl << endl;
  cout << "median tracking time: " << vTimesTrack[nImages / 2] << endl;
  cout << "mean tracking time: " << totaltime / nImages << endl;

  // Save camera trajectory
  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

  return 0;
}

std::vector<double> LoadTimestamps(const std::string &path_to_timestamps) {
  std::vector<double> timestamps;
  std::ifstream in;
  in.open(path_to_timestamps.c_str());
  while (in.good()) {
    std::string s;
    getline(in, s);
    if (!s.empty()) {
      std::stringstream ss;
      ss << s;
      if (ss.good()) {
        double t;
        ss >> t;
        timestamps.emplace_back(t);
      }
    }
  }
  return timestamps;
}

void SaveImagePaths(const std::string &path_to_images, const size_t num_of_imgs,
                    std::vector<std::string> *const image_paths) {
  const std::string prefix = path_to_images + "/";
  for (size_t i = 0; i < num_of_imgs; ++i) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << i;
    image_paths->emplace_back(prefix + ss.str() + ".jpg");
  }
}