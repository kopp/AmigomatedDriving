#include <ros/ros.h> // Publisher

// handle image
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <curl/curl.h> // download data from camera

#include <cstdint>

#include <vector> // buffer

#include <camera_info_manager/camera_info_manager.h> // camere interface


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
  static size_t handle_image_from_web(void* p_data, size_t size, size_t nmemb, void* p_userdata);


  /// \brief publish image and camera info
  void publishImage(cv::Mat const & decoded_image);


public:
  /// Set up the web cam receiver
  /// \param image_url complete url to the image to get
  /// \param login_info Login required to get to the camera images as username:password
  WebCamReceiver(ros::NodeHandle& nh,
                 std::string frame_id,
                 std::string camera_name,
                 std::string calibration_url,
                 std::string image_url,
                 std::string login_info);

  /// \brief Try to get a new image from the webcam and publish it.
  /// \param timeout_ms set a timeout in ms for the operation; <= 0 for no timeout
  /// \return false in case of error
  bool processImage(int timeout_ms);

  ~WebCamReceiver();
};
