#include <opencv2/opencv.hpp>

#include "kimera-vio/visualizer/Visualizer3D-definitions.h"

namespace VIO {

  class BaseOpenCvVisualizer3D {
  public:
    KIMERA_POINTER_TYPEDEFS(BaseOpenCvVisualizer3D);

    virtual void addPoseToTrajectory(const cv::Affine3d& pose) = 0;

    virtual void visualizeTrajectory3D(WidgetsMap* widgets_map) = 0;

    virtual void visualizePoseWithImgInFrustum(
      const cv::Mat& frustum_image,
      const cv::Affine3d& frustum_pose,
      WidgetsMap* widgets_map,
      const std::string& widget_id,
      const cv::Matx33d K) = 0;

    virtual void visualizePlyMesh(const std::string& filename, 
                                  WidgetsMap* widgets) = 0;

    virtual void visualizePointCloud(const cv::Mat& point_cloud,
                                     WidgetsMap* widgets,
                                     const cv::Affine3d& pose,
                                     const cv::Mat& colors,
                                     const cv::Mat& normals) = 0;

    virtual void visualizeGlobalFrameOfReference(WidgetsMap* widgets, 
                                                 double scale) = 0;

    virtual void visualizeMesh3D(const cv::Mat& map_points_3d,
                                 const cv::Mat& colors,
                                 const cv::Mat& polygons_mesh,
                                 WidgetsMap* widgets,
                                 const cv::Mat& tcoords,
                                 const cv::Mat& texture,
                                 const std::string& mesh_id) = 0;

    virtual void draw3dMesh(const std::string& id, const Mesh3D& mesh_3d,
                            bool display_as_wireframe ,
                            const double& opacity) = 0;

    virtual void meshSpinDisplay() = 0;
  };

} // namespace VIO
