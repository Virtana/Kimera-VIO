/* ----------------------------------------------------------------------------
 * Copyright 2017, Massachusetts Institute of Technology,
 * Cambridge, MA 02139
 * All Rights Reserved
 * Authors: Luca Carlone, et al. (see THANKS for the full author list)
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 * @file   MonoImuPipeline.cpp
 * @brief  Implements MonoVIO pipeline workflow.
 * @author Marcus Abate
 */

#include "kimera-vio/pipeline/MonoImuPipeline.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <string>

#include "kimera-vio/backend/VioBackendFactory.h"
#include "kimera-vio/frontend/MonoVisionImuFrontend-definitions.h"
#include "kimera-vio/frontend/VisionImuFrontendFactory.h"
#include "kimera-vio/mesh/MesherFactory.h"
#include "kimera-vio/pipeline/Pipeline.h"
#include "kimera-vio/utils/Statistics.h"
#include "kimera-vio/utils/Timer.h"
// #include "kimera-vio/visualizer/DisplayFactory.h"
// #include "kimera-vio/visualizer/Visualizer3DFactory.h"

namespace VIO {

MonoImuPipeline::MonoImuPipeline(const VioParams& params)
    : Pipeline(params),
      camera_(nullptr) {
  // TODO(marcus): specify separate params for mono
  // CHECK_EQ(params.camera_params_.size(), 1u) << "Need one camera for MonoImuPipeline.";
  camera_ = std::make_shared<Camera>(params.camera_params_.at(0));

  data_provider_module_ = VIO::make_unique<MonoDataProviderModule>(
      &frontend_input_queue_,
      "Mono Data Provider",
      parallel_run_);

  data_provider_module_->registerVioPipelineCallback(
    std::bind(&MonoImuPipeline::spinOnce, this, std::placeholders::_1));

  LOG_IF(FATAL, params.frontend_params_.useStereoTracking_) 
      << "useStereoTracking is set to true, but this is a mono pipeline!";
  vio_frontend_module_ = VIO::make_unique<VisionImuFrontendModule>(
      &frontend_input_queue_,
      parallel_run_,
      VisionImuFrontendFactory::createFrontend(
          params.frontend_type_,
          params.imu_params_,
          gtsam::imuBias::ConstantBias(),
          params.frontend_params_,
          camera_,
          FLAGS_visualize ? &display_input_queue_ : nullptr,
          FLAGS_log_output));

  auto& backend_input_queue = backend_input_queue_;
  vio_frontend_module_->registerOutputCallback([&backend_input_queue](
      const FrontendOutputPacketBase::Ptr& output) {
    MonoFrontendOutput::Ptr converted_output = 
        VIO::safeCast<FrontendOutputPacketBase, MonoFrontendOutput>(output);

    if (converted_output->is_keyframe_) {
      //! Only push to Backend input queue if it is a keyframe!
      backend_input_queue.push(VIO::make_unique<BackendInput>(
          converted_output->frame_lkf_.timestamp_,
          converted_output->status_mono_measurements_,
          converted_output->tracker_status_,
          converted_output->pim_,
          converted_output->imu_acc_gyrs_,
          boost::none));  // don't pass stereo pose to Backend!
    } else {
      VLOG(5) << "Frontend did not output a keyframe, skipping Backend input.";
    }
  });

  //! Params for what the Backend outputs.
  // TODO(Toni): put this into Backend params.
  BackendOutputParams backend_output_params(
      true,
      FLAGS_min_num_obs_for_mesher_points,
      false);

  //! Create Backend
  // TODO(marcus): get rid of fake stereocam
  LOG_IF(FATAL, params.backend_params_->addBetweenStereoFactors_)
      << "addBetweenStereoFactors is set to true, but this is a mono pipeline!";
  VIO::StereoCamera::Ptr stereo_cam = std::make_shared<VIO::StereoCamera>(
      params.camera_params_.at(0), params.camera_params_.at(1));
  CHECK(backend_params_);
  vio_backend_module_ = VIO::make_unique<VioBackendModule>(
      &backend_input_queue_,
      parallel_run_,
      BackendFactory::createBackend(
          static_cast<BackendType>(params.backend_type_),
          // These two should be given by parameters.
          camera_->getBodyPoseCam(),
          stereo_cam->getStereoCalib(),
          *backend_params_,
          imu_params_,
          backend_output_params,
          FLAGS_log_output));

  vio_backend_module_->registerOnFailureCallback(
      std::bind(&MonoImuPipeline::signalBackendFailure, this));

  vio_backend_module_->registerImuBiasUpdateCallback(
      std::bind(&VisionImuFrontendModule::updateImuBias,
                // Send a cref: constant reference bcs updateImuBias is const
                std::cref(*CHECK_NOTNULL(vio_frontend_module_.get())),
                std::placeholders::_1));

  // TOOD(marcus): enable use of mesher for mono pipeline
  // if (static_cast<VisualizationType>(FLAGS_viz_type) ==
  //     VisualizationType::kMesh2dTo3dSparse) {
  //   mesher_module_ = VIO::make_unique<MesherModule>(
  //       parallel_run_,
  //       MesherFactory::createMesher(
  //           MesherType::PROJECTIVE,
  //           MesherParams(camera_->getBodyPoseCam(),
  //                        params.camera_params_.at(0u).image_size_)));

  //   //! Register input callbacks
  //   vio_backend_module_->registerOutputCallback(
  //       std::bind(&MesherModule::fillBackendQueue,
  //                 std::ref(*CHECK_NOTNULL(mesher_module_.get())),
  //                 std::placeholders::_1));

  //   vio_frontend_module_->registerOutputCallback(
  //       std::bind(&MesherModule::fillFrontendQueue,
  //                 std::ref(*CHECK_NOTNULL(mesher_module_.get())),
  //                 std::placeholders::_1));
  // }

  if (FLAGS_use_lcd) {
    lcd_module_ = VIO::make_unique<LcdModule>(
        parallel_run_,
        LcdFactory::createLcd(LoopClosureDetectorType::BoW,
                              params.lcd_params_,
                              stereo_cam,
                              params.frontend_params_.stereo_matching_params_,
                              FLAGS_log_output));
    //! Register input callbacks
    vio_backend_module_->registerOutputCallback(
        std::bind(&LcdModule::fillBackendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));
    vio_frontend_module_->registerOutputCallback(
        std::bind(&LcdModule::fillFrontendQueue,
                  std::ref(*CHECK_NOTNULL(lcd_module_.get())),
                  std::placeholders::_1));
  }

  launchThreads();
}

}  // namespace VIO
