#include "kimera-vio/visualizer/abstract/BaseOpenCvVisualizer3D.h"

namespace VIO{
  
  class DummyOpenCvVisualizer3D : public BaseOpenCvVisualizer3D {
  public: 
    DummyOpenCvVisualizer3D() {
      std::cout << "Using dummy OpenCvVisualizer" << std::endl;
    }

    void addPoseToTrajectory(const cv::Affine3d& pose) override {
      // function does nothing
    }

    void visualizeTrajectory3D(WidgetsMap* widgets_map) override {
      // function does nothing
    }

    void visualizePoseWithImgInFrustum(const cv::Mat& frustum_image,
                                       const cv::Affine3d& frustum_pose,
                                       WidgetsMap* widgets_map,
                                       const std::string& widget_id,
                                       const cv::Matx33d K) override {
      // function does nothing
    }

    void visualizePlyMesh(const std::string& filename, 
                          WidgetsMap* widgets) override {
      // function does nothing
    }
    
    void visualizePointCloud(const cv::Mat& point_cloud, WidgetsMap* widgets,
                            const cv::Affine3d& pose, const cv::Mat& colors,
                            const cv::Mat& normals) override {
      // function does nothing
    }

    void visualizeGlobalFrameOfReference(WidgetsMap* widgets, 
                                         double scale) override {
      // function does nothing
    }

    void visualizeMesh3D(const cv::Mat& map_points_3d, const cv::Mat& colors,
                                 const cv::Mat& polygons_mesh,
                                 WidgetsMap* widgets, const cv::Mat& tcoords,
                                 const cv::Mat& texture,
                                 const std::string& mesh_id) override {
      // function does nothing
    }

    void draw3dMesh(const std::string& id, const Mesh3D& mesh_3d,
                    bool display_as_wireframe, const double& opacity) override {
      // function does nothing
    }

    void meshSpinDisplay() override {
      // function does nothing
    }
  };

} // namespace VIO