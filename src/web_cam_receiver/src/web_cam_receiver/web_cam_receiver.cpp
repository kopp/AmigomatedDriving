#include <web_cam_receiver/web_cam_receiver.h> // this

#include <cassert>
#include <sstream> // debug output
#include <algorithm> // min

#include <cv_bridge/cv_bridge.h> // convert opencv to ros message

// ros messages
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>


size_t WebCamReceiver::handle_image_from_web(void* p_data, size_t size, size_t nmemb, void* p_userdata)
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



void WebCamReceiver::publishImage(const cv::Mat& decoded_image)
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



WebCamReceiver::WebCamReceiver(ros::NodeHandle& nh, std::string frame_id, std::string camera_name, std::string calibration_url, std::string image_url, std::string login_info)
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



bool WebCamReceiver::processImage(int timeout_ms)
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



WebCamReceiver::~WebCamReceiver()
{
  // tear down curl
  curl_easy_cleanup(curl_handle_);
}
