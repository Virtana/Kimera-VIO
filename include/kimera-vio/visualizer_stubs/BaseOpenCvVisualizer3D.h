#pragma once

#include <iostream>
#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#ifdef KIMERA_BUILD_VISUALIZER
    #include "kimera-vio/visualizer/Visualizer3D-definitions.h"
#endif
namespace VIO {

class BaseOpenCvVisualizer3D { // abstract class
 public:
    using Ptr = std::shared_ptr<BaseOpenCvVisualizer3D>;

    BaseOpenCvVisualizer3D() = default;
    virtual ~BaseOpenCvVisualizer3D() = default;

    virtual void printTest() = 0; // pure virtual function

    #ifdef KIMERA_BUILD_VISUALIZER
        virtual void addPoseToTrajectory(const cv::Affine3d& pose) = 0;

        virtual void visualizeTrajectory3D(WidgetsMap* widgets_map) = 0;

        virtual void visualizePlyMesh(const std::string& filename, WidgetsMap* widgets) = 0;

        virtual void visualizePoseWithImgInFrustum(
            const cv::Mat& frustum_image,
            const cv::Affine3d& frustum_pose,
            WidgetsMap* widgets_map,
            const std::string& widget_id = "Camera Pose with Frustum",
            const cv::Matx33d K =
                cv::Matx33d(458, 0.0, 360, 0.0, 458, 240, 0.0, 0.0, 1.0)) = 0;
        
        virtual void visualizePointCloud(const cv::Mat& point_cloud,
                            WidgetsMap* widgets,
                            const cv::Affine3d& pose = cv::Affine3d(),
                            const cv::Mat& colors = cv::Mat(),
                            const cv::Mat& normals = cv::Mat()) = 0;

        virtual void visualizeGlobalFrameOfReference(WidgetsMap* widgets, double scale = 1.0) = 0;

        virtual void visualizeMesh3D(const cv::Mat& map_points_3d,
                        const cv::Mat& colors,
                        const cv::Mat& polygons_mesh,
                        WidgetsMap* widgets,
                        const cv::Mat& tcoords = cv::Mat(),
                        const cv::Mat& texture = cv::Mat(),
                        //! This has to be the same than the id in OpenCvDisplay
                        const std::string& mesh_id = "Mesh") = 0;
    #endif
};

class DummyOpenCvVisualizer3D : public BaseOpenCvVisualizer3D { 
 public: 
    using Ptr = std::shared_ptr<DummyOpenCvVisualizer3D>; 
    
    DummyOpenCvVisualizer3D() = default;
    virtual ~DummyOpenCvVisualizer3D() = default;

    void printTest() override {
        LOG(INFO) << "DummyOpenCvVisualizer3D::printTest()";
    }

    #ifdef KIMERA_BUILD_VISUALIZER
        void addPoseToTrajectory(const cv::Affine3d& pose) override {}
        
        void visualizeTrajectory3D(WidgetsMap* widgets_map) override {}

        void visualizePlyMesh(const std::string& filename, WidgetsMap* widgets) override {}

        void visualizePoseWithImgInFrustum(
            const cv::Mat& frustum_image,
            const cv::Affine3d& frustum_pose,
            WidgetsMap* widgets_map,
            const std::string& widget_id = "Camera Pose with Frustum",
            const cv::Matx33d K =
                cv::Matx33d(458, 0.0, 360, 0.0, 458, 240, 0.0, 0.0, 1.0)) override {}
        
        void visualizePointCloud(const cv::Mat& point_cloud,
                            WidgetsMap* widgets,
                            const cv::Affine3d& pose = cv::Affine3d(),
                            const cv::Mat& colors = cv::Mat(),
                            const cv::Mat& normals = cv::Mat()) override {}

        void visualizeGlobalFrameOfReference(WidgetsMap* widgets, double scale = 1.0) override {}

        void visualizeMesh3D(const cv::Mat& map_points_3d,
                        const cv::Mat& colors,
                        const cv::Mat& polygons_mesh,
                        WidgetsMap* widgets,
                        const cv::Mat& tcoords = cv::Mat(),
                        const cv::Mat& texture = cv::Mat(),
                        //! This has to be the same than the id in OpenCvDisplay
                        const std::string& mesh_id = "Mesh") override {}
    #endif
};

}
