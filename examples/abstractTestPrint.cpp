//temporary file for abstract class test only 
#include <future>
#include <limits>
#include <utility>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "kimera-vio/frontend/feature-detector/FeatureDetector.h"
#include "kimera-vio/frontend/feature-detector/FeatureDetectorParams.h"
#include "kimera-vio/mesh/Mesh.h"
#include "kimera-vio/mesh/MeshOptimization.h"
#include "kimera-vio/mesh/Mesher.h"
#include "kimera-vio/playground/EurocPlayground.h"
#include "kimera-vio/visualizer_stubs/BaseOpenCvVisualizer3D.h"
#ifdef KIMERA_BUILD_VISUALIZER
    #include "kimera-vio/visualizer/OpenCvVisualizer3D.h"
#endif

using namespace VIO;

int main(int argc, char* argv[]) {
    // Dummy camera init from testMeshOptimization.cpp
    std::string test_data_path = "/root/Kimera-VIO/tests"; //harcoded for docker container
    CameraParams camera_params;
    camera_params.parseYAML(test_data_path + "/data/EurocParams/LeftCameraParams.yaml");
    camera_params.body_Pose_cam_ = gtsam::Pose3();
    camera_params.image_size_ = cv::Size(2, 2);
    Camera::ConstPtr mono_camera = std::make_unique<Camera>(camera_params);

    #ifdef KIMERA_BUILD_VISUALIZER
        BaseOpenCvVisualizer3D::Ptr visualizer = std::make_shared<OpenCvVisualizer3D>(VisualizationType::kPointcloud,
                                                                                BackendType::kStereoImu);
    #else
        BaseOpenCvVisualizer3D::Ptr visualizer = std::make_shared<DummyOpenCvVisualizer3D>();
    #endif

    // Optimize mesh using MeshOpt
    MeshOptimization mesh_opt(MeshOptimizerType::kGtsamMesh,
                                MeshColorType::kVertexFlatColor,
                                mono_camera,
                                visualizer);

    mesh_opt.runVisualizerPrintTest();

    return 0;
}
