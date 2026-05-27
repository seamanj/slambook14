//
// Created by gaoxiang on 19-5-4.
//

#include <gflags/gflags.h>
#include "myslam/visual_odometry.h"

DEFINE_string(config_file, "", "config file path"); // tj : we skip default value here, since we want to force specify it in command line

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    myslam::VisualOdometry::Ptr vo(
        new myslam::VisualOdometry(FLAGS_config_file));
    assert(vo->Init() == true);
    vo->Run();

    return 0;
}
