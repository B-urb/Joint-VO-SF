
#include <Eigen/Core>
#include <OpenNI.h>
#include <iostream>
#include <librealsense2/rs.hpp>


class RGBD_Camera {
public:

    RGBD_Camera(unsigned int res_factor);

	unsigned int cam_mode;
	float max_distance;

  rs2_error *e;
  rs2_pipeline_profile *pipeline_profile;
  rs2_config *rs2config;
  rs2_pipeline *pipeline;
  rs2_device *dev;
  rs2_device_list *device_list;
  rs2_sensor_list *sensor_list;
  rs2_context *ctx;
  // align depth and RGB
  rs2_frame_queue *m_alignQueue;
  rs2_processing_block *m_alignProcessor;
//
//  // temporal filter
//  rs2_frame_queue *m_postprocess_temporal_Queue;
//  rs2_processing_block *m_postprocess_temporal_Processor;
//
//  // spatial filter
//  rs2_frame_queue *m_postprocess_spatial_Queue;
//  rs2_processing_block *m_postprocess_spatial_Processor;


  //Create a configuration for configuring the pipeline with a non default profile
  rs2::config cfg;

  void rs_print_error(rs2_error *e);
    bool openCamera();
    void closeCamera();
    void loadFrame(Eigen::MatrixXf &depth_wf, Eigen::MatrixXf &color_wf);
    void disableAutoExposureAndWhiteBalance();
};



