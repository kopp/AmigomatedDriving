/// @file
/// \brief Build a node that publishes images from a web cam as ros topic.
///
/// Further information about techniques used in this node:
///
/// * http://answers.ros.org/question/99831/publish-file-to-image-topic/
/// * https://curl.haxx.se/libcurl/c/example.html
/// * http://stackoverflow.com/questions/14727267/opencv-read-jpeg-image-from-buffer



#include <ros/ros.h>
#include <web_cam_receiver/web_cam_receiver.h>


/// \brief helper to load parameters
#define getParamFatal(param_name, description) \
    std::string param_name; \
    if (! nh.getParam(#param_name, param_name)) \
    { \
      ROS_ERROR("please specify " description " as parameter " #param_name); \
      all_params_available = false; \
    } \
    else \
    { \
      ROS_INFO_STREAM("Using " #param_name " = " << param_name); \
    }


int main(int argc, char** argv)
{
  try
  {
    // setup ros
    ros::init(argc, argv, "web_cam_receiver");
    ros::NodeHandle nh;

    // get parameters for settings
    bool all_params_available = true;
    getParamFatal(frame_id, "frame id");
    getParamFatal(camera_name, "name of the camera model");
    getParamFatal(calibration_url, "url to a file with calibration for the given camera (${NAME} expands to camera model name");
    getParamFatal(image_url, "url from which a single image can be retrieved");
    getParamFatal(login_info, "login information for the camera in the form user:password");
    if (! all_params_available)
    {
      ROS_FATAL("Unable to start web_cam_receiver because parameters are missing.  "
                "Make sure that the required paramteres are set in the namespace of this node.");
      return 1;
    }
    float loop_rate_Hz;
    if (! nh.getParam("loop_rate_Hz", loop_rate_Hz))
    {
      loop_rate_Hz = 5;
      ROS_WARN_STREAM("param loop_rate_Hz not specified; assuming " << loop_rate_Hz);
    }
    std::cout << "loop rate: " << loop_rate_Hz << std::endl;

    WebCamReceiver receiver(nh,
                            frame_id,
                            camera_name,
                            calibration_url,
                            image_url,
                            login_info);


    // main loop
    ros::Rate loop_rate(loop_rate_Hz);
    while (nh.ok())
    {
      // timeout a little bit shorter than loop rate
      int timeout_ms = loop_rate.expectedCycleTime().toNSec() / 1100000;
      receiver.processImage(timeout_ms);

      loop_rate.sleep();
    }
  } catch (std::runtime_error& e)
  {
    std::string msg(e.what());
    ROS_FATAL_STREAM("WebCamReceiver failed with " << msg);
    return 2;
  }

  return 0;
}
