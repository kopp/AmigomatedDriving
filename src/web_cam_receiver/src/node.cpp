#include <ros/ros.h>

// handle image
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h> // convert opencv to ros message

// ros messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <curl/curl.h> // download data from camera

#include <cassert>
#include <cstdint>
#include <sstream> // debug output
#include <algorithm> // min
#include <vector> // buffer

#include <camera_info_manager/camera_info_manager.h> // camere interface


/// Further information about techniques used in this node:
///
/// * http://answers.ros.org/question/99831/publish-file-to-image-topic/
/// * https://curl.haxx.se/libcurl/c/example.html
/// * http://stackoverflow.com/questions/14727267/opencv-read-jpeg-image-from-buffer



/// \brief Class that handles connection to the camera, retrival of images
///        and publishing to ros topics.
class WebCamReceiver
{
private:
  // ros
  /// distorted camera image
  ros::Publisher image_pub_;
  /// information about this camera
  /// send data for uncalibrated camera (D, K, R, P all 0)
  ros::Publisher info_pub_;

  /// frame id of the camera
  std::string frame_id_;

  /// provide ros access to camera info stuff
 camera_info_manager::CameraInfoManager camera_manager_;

  // network
  /// buffer that will be used to collect data from multiple callbacks for one GET request
  std::vector<std::uint8_t> current_message_buffer_;

  /// handle used for all curl operations
  CURL* curl_handle_;

protected: // methods

  /// \brief callback for web request by curl
  /// This callback collects data from multiple calls in a buffer.
  /// \param p_data       pointer to the data that is new in this call
  /// \param size         see nmemb
  /// \param nmemb        the product size * nmemb is the number of bytes to handle
  /// \param p_userdata   pointer to a buffer that collects data; actually vector<uint8_t>
  /// \return number of bytes handled
  static size_t handle_image_from_web(void* p_data, size_t size, size_t nmemb, void* p_userdata)
  {
    assert(p_data != 0 && "invalid data passed into handle_image_from_web");
    assert(p_userdata != 0 && "invalid user data passed into handle_image_from_web");

    size_t num_bytes = size * nmemb;
    ROS_DEBUG_STREAM("Received " << num_bytes << " bytes (" << size << ", " << nmemb << ")");

    // user data is a buffer
    std::vector<std::uint8_t>& buffer = *static_cast<std::vector<std::uint8_t>*>(p_userdata);
    // size of the buffer before and after appending the data
    size_t size_before = buffer.size();
    size_t expected_size = size_before + num_bytes;
    // make sure, that we have enough space in the buffer
    buffer.reserve(expected_size);
    // now copy the data into the buffer (append after last data)
    buffer.insert(buffer.end(),
                  static_cast<std::uint8_t*>(p_data),
                  static_cast<std::uint8_t*>(p_data) + num_bytes);

    // make sure that all data was copied
    assert(buffer.size() == expected_size && "data was not correctly inserted into buffer");

    // success: return that all bytes were processed
    return num_bytes;
  }


  /// \brief publish image and camera info
  void publishImage(cv::Mat const & decoded_image)
  {
    // header for messages
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = frame_id_;
    // Note: header.seq is set when publish is called

    // use cv_bridge to convert to ros msg
    cv_bridge::CvImage cv_image(header, "bgr8", decoded_image);
    auto image_msg = cv_image.toImageMsg();
    image_pub_.publish(image_msg);

    // camera info
    // Note: This will load the configuration from file if that was not done before.
    auto info_msg = camera_manager_.getCameraInfo();
    info_msg.header = header;

    // check, whether basic image properties are set correctly
    if (info_msg.height != image_msg->height || info_msg.width != image_msg->width)
    {
      // set them in any case, but warn when the camera was assumed to be calibrated
      if (camera_manager_.isCalibrated())
      {
        ROS_ERROR("camera calibration does not seem to match:"
                  "different height/width than image from camera.");
      }
      info_msg.height = image_msg->height;
      info_msg.width = image_msg->width;
    }

    info_pub_.publish(info_msg);
  }


public:
  /// Set up the web cam receiver
  /// \param image_url complete url to the image to get
  /// \param login_info Login required to get to the camera images as username:password
  WebCamReceiver(ros::NodeHandle& nh,
                 std::string frame_id,
                 std::string camera_name,
                 std::string calibration_url,
                 std::string image_url,
                 std::string login_info)
    : image_pub_(nh.advertise<sensor_msgs::Image>("image_raw", 1))
    , info_pub_(nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1))
    , frame_id_(frame_id)
    , camera_manager_(nh,  camera_name, calibration_url)
  {
    // // setup curl
    curl_handle_ = curl_easy_init();
    if (! curl_handle_)
    {
      throw std::runtime_error("Unable to start curl, thus unable to get images.");
    }
    // // configure curl
    // set address
    curl_easy_setopt(curl_handle_, CURLOPT_URL, image_url.c_str());
    // set login information
    curl_easy_setopt(curl_handle_, CURLOPT_USERPWD, login_info.c_str());
    // follow redirect if necessary
    curl_easy_setopt(curl_handle_, CURLOPT_FOLLOWLOCATION, 1L);
    // set callback to handle data
    curl_easy_setopt(curl_handle_, CURLOPT_WRITEFUNCTION, WebCamReceiver::handle_image_from_web);
    // set p_userdata to the cv image that will transport the data
    curl_easy_setopt(curl_handle_, CURLOPT_WRITEDATA, static_cast<void*>(&current_message_buffer_));
  }

  /// \brief Try to get a new image from the webcam and publish it.
  /// \param timeout_ms set a timeout in ms for the operation; <= 0 for no timeout
  /// \return false in case of error
  bool processImage(int timeout_ms)
  {
    // set timeout if necessary
    long actual_timeout_ms = 0;
    if (timeout_ms > 0)
    {
      actual_timeout_ms = static_cast<long>(timeout_ms);
    }
    curl_easy_setopt(curl_handle_, CURLOPT_TIMEOUT_MS, actual_timeout_ms);

    // perform web request
    CURLcode res = curl_easy_perform(curl_handle_);
    // check for errors
    if(res == CURLE_OK)
    {
      ROS_DEBUG("curl GET performed sucessfully");

      // convert to image; data to interpret is in buffer
      // interpret data as image
      cv::Mat rawData(1, current_message_buffer_.size(), CV_8UC1, current_message_buffer_.data());
      cv::Mat decodedImage = cv::imdecode(rawData, CV_LOAD_IMAGE_COLOR);
      // check, whether interpretation worked
      if (decodedImage.data == 0)
      {
        // it did not work -- print error message
        std::stringstream s;
        for (size_t i = 0; i < std::min(current_message_buffer_.size(), static_cast<size_t>(50)); ++i)
        {
          s << std::hex << static_cast<int>(current_message_buffer_[i]);
        }
        ROS_ERROR_STREAM("Unable to interpret data from web as image: " << s.str());
      }
      else
      {
        // it worked; publish image
        ROS_DEBUG("Received data sucessfully converted to image; going to publish as ros image.");
        publishImage(decodedImage);
      }
      // in any case, clear the buffer
      current_message_buffer_.clear();

      return true;
    }
    else
    {
      ROS_ERROR_STREAM("Error '" << curl_easy_strerror(res) << "' when trying to get an image");
      return false;
    }
  }

  ~WebCamReceiver()
  {
    // tear down curl
    curl_easy_cleanup(curl_handle_);
  }
};


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
