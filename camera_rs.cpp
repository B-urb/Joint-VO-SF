/*********************************************************************************
**Fast Odometry and Scene Flow from RGB-D Cameras based on Geometric Clustering	**
**------------------------------------------------------------------------------**
**																				**
**	Copyright(c) 2017, Mariano Jaimez Tarifa, University of Malaga & TU Munich	**
**	Copyright(c) 2017, Christian Kerl, TU Munich								**
**	Copyright(c) 2017, MAPIR group, University of Malaga						**
**	Copyright(c) 2017, Computer Vision group, TU Munich							**
**																				**
**  This program is free software: you can redistribute it and/or modify		**
**  it under the terms of the GNU General Public License (version 3) as			**
**	published by the Free Software Foundation.									**
**																				**
**  This program is distributed in the hope that it will be useful, but			**
**	WITHOUT ANY WARRANTY; without even the implied warranty of					**
**  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the				**
**  GNU General Public License for more details.								**
**																				**
**  You should have received a copy of the GNU General Public License			**
**  along with this program. If not, see <http://www.gnu.org/licenses/>.		**
**																				**
*********************************************************************************/

#include <camera_rs.h>

const int CAMERA_RESOLUTION_WIDTH = 640;
const int CAMERA_RESOLUTION_HEIGHT = 480;

RGBD_Camera::RGBD_Camera(unsigned int res_factor)
{
    cam_mode = res_factor; // (1 - 640 x 480, 2 - 320 x 240)
	max_distance = 4.5f;
        e = 0;

}

bool RGBD_Camera::openCamera()
{

  // Create a context object. This object owns the handles to all connected realsense devices.
  // The returned object should be released with rs2_delete_context(...)
  ctx = rs2_create_context(RS2_API_VERSION, &e);
  rs_print_error(e);


  // Get a list of all the connected devices.
  // The returned object should be released with rs2_delete_device_list(...)
  device_list = rs2_query_devices(ctx, &e);
  rs_print_error(e);

  int dev_count = rs2_get_device_count(device_list, &e);
  rs_print_error(e);

  dev = rs2_create_device(device_list, 0, &e);
  rs_print_error(e);

  rs2_sensor_list* slist = rs2_query_sensors(dev, &e);
  rs_print_error(e);

  int sensor_list_count = rs2_get_sensors_count(slist, &e);
  rs_print_error(e);

  rs2_sensor* depth_sensor = rs2_create_sensor(slist, 0, &e);
  rs_print_error(e);

  rs2_sensor* color_sensor = rs2_create_sensor(slist, 1, &e);
  rs_print_error(e);


  //float min,max,step,def;
  //rs2_get_option_range((rs2_options*)sensor, RS2_OPTION_LASER_POWER, &min, &max, &step, &def, &e);

  //set depth sensor options
  rs2_set_option((rs2_options*)depth_sensor, RS2_OPTION_LASER_POWER, 360, &e);
  rs2_set_option((rs2_options*)depth_sensor, RS2_OPTION_EMITTER_ENABLED, 1, &e);
  //rs2_set_option((rs2_options*)depth_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0, &e);

  //set color sensor options
  //rs2_set_option((rs2_options*)color_sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0, &e);
  //rs2_set_option((rs2_options*)color_sensor, RS2_OPTION_EXPOSURE, 50.f, &e);


  if (e)
  {
    return false;
  }


  // Create a pipeline to configure, start and stop camera streaming
  // The returned object should be released with rs2_delete_pipeline(...)
  pipeline = rs2_create_pipeline(ctx, &e);
  rs_print_error(e);

  // Create a config instance, used to specify hardware configuration
  // The returned object should be released with rs2_delete_config(...)
  rs2config = rs2_create_config(&e);
  rs_print_error(e);

  if (e)
  {

    return false;
  }

  const int CAMERA_FPS = 30;


  //qDebug() << "Configuring data streams.";

  // Request a specific configuration
  rs2_config_enable_stream(rs2config, RS2_STREAM_DEPTH, 0, CAMERA_RESOLUTION_WIDTH, CAMERA_RESOLUTION_HEIGHT, RS2_FORMAT_Z16, CAMERA_FPS, &e);
  if (e)
  {
    rs_print_error(e);
    return false;
  }

  rs2_config_enable_stream(rs2config, RS2_STREAM_COLOR, 0, CAMERA_RESOLUTION_WIDTH, CAMERA_RESOLUTION_HEIGHT, RS2_FORMAT_RGB8, CAMERA_FPS, &e);
  if (e)
  {
    rs_print_error(e);
    return false;
  }


  //
  // Depth & Color alignment
  //
  // init alignment process
  m_alignQueue = rs2_create_frame_queue(1, &e);
  m_alignProcessor = rs2_create_align(RS2_STREAM_COLOR, &e);

  // Start the pipeline streaming
  // The returned object should be released with rs2_delete_pipeline_profile(...)
  pipeline_profile = rs2_pipeline_start_with_config(pipeline, rs2config, &e);
  if (e)
  {
    rs_print_error(e);
    return false;
  }

  rs2_start_processing_queue(m_alignProcessor, m_alignQueue, &e);
  if (e)
  {
   rs_print_error(e);
    return false;
  }

    //rs2_frame *frames = rs2_pipeline_wait_for_frames(pipeline, 2000, &e);




  return 1;
}

void RGBD_Camera::closeCamera()
{
//  // Stop the pipeline streaming
//  //rs2_pipeline_stop(pipe, &e);
//  rs2::check_error(e);
//
//  // Release resources
//  rs2_delete_pipeline_profile(pipeline_profile);
//  rs2_delete_config(config);
//  rs2_delete_pipeline(pipeline);
//  rs2_delete_device(dev);
//  rs2_delete_device_list(device_list);
//  rs2_delete_context(ctx);
//
//  return EXIT_SUCCESS;
}

void RGBD_Camera::loadFrame(Eigen::MatrixXf &depth_wf, Eigen::MatrixXf &color_wf)
{

  rs2_frame* frames;


  // Blocking call to fetch a frame.
  frames = rs2_pipeline_wait_for_frames(pipeline, 2000, &e);

  // Returns the number of frames embedded within the composite frame
  int num_of_frames = rs2_embedded_frames_count(frames, &e);
  rs_print_error(e);


  rs2_frame *depth_final_frame = nullptr;

  // align depth and color data
  rs2_frame_add_ref(frames, &e);
  rs_print_error(e);

  rs2_process_frame(m_alignProcessor, frames, &e);
  rs_print_error(e);
  if (e)
     std::cerr << "rs2_process_frame failed!";


  rs2_frame *frames_aligned = rs2_wait_for_frame(m_alignQueue, 100, &e);
  rs_print_error(e);
  if (e)
    std::cerr << "rs2_wait_for_frame frames_aligned failed!";


  // Get depth and color frames
  rs2_frame *frame_depth = rs2_extract_frame(frames_aligned, 0, &e);
  rs_print_error(e);
  rs2_frame *frame_color = rs2_extract_frame(frames_aligned, 1, &e);
  rs_print_error(e);

  depth_final_frame = frame_depth;

  const float norm_factor = 1.f/255.f;
  const float max_dist_mm = 1000.f*max_distance;

  const uint16_t *depth_frame_data = (const uint16_t *)(rs2_get_frame_data(depth_final_frame, &e));
  rs_print_error(e);
  const uint8_t *rgb_frame_data = (const uint8_t *)(rs2_get_frame_data(frame_color, &e));
  rs_print_error(e);

  for (int y = 0; y < CAMERA_RESOLUTION_HEIGHT; ++y)
  {
    for (int x = 0; x < CAMERA_RESOLUTION_WIDTH; ++x)
    {
      uint16_t depth = *depth_frame_data++;
      uint8_t r = *rgb_frame_data++;
      uint8_t g = *rgb_frame_data++;
      uint8_t b = *rgb_frame_data++;

      color_wf(y,x) = norm_factor*(0.299f*r + 0.587f*g+ 0.114f*b);
      depth_wf(y,x) = 0.001f*(depth)*(depth < max_dist_mm);

  }
}

  rs2_release_frame(frame_depth);
  rs2_release_frame(depth_final_frame);

  rs2_release_frame(frame_color);
  rs2_release_frame(frames_aligned);
  rs2_release_frame(frames);
}

void RGBD_Camera::disableAutoExposureAndWhiteBalance()
{
//    Eigen::MatrixXf aux_depth, aux_color;
//	if (cam_mode == 1)	{aux_depth.resize(480,640); aux_color.resize(480,640);}
//	else				{aux_depth.resize(240,320); aux_color.resize(240,320);}
//
//	for (unsigned int i=1; i<=10; i++)
//        loadFrame(aux_depth, aux_color);
//
//    //Turn off autoExposure
//    rgb.getCameraSettings()->setAutoExposureEnabled(false);
//    printf("Auto Exposure: %s \n", rgb.getCameraSettings()->getAutoExposureEnabled() ? "ON" : "OFF");
//
//    //Turn off White balance
//    rgb.getCameraSettings()->setAutoWhiteBalanceEnabled(false);
//    printf("Auto White balance: %s \n", rgb.getCameraSettings()->getAutoWhiteBalanceEnabled() ? "ON" : "OFF");
}

void RGBD_Camera::rs_print_error(rs2_error *e)
{
  if (e > 0)
  {
    std::cout << "rs_error was raised when calling "
             << rs2_get_failed_function(e) << "(" << rs2_get_failed_args(e) << "):";
    std::cout << "    " << rs2_get_error_message(e) << std::endl;
  }
}

