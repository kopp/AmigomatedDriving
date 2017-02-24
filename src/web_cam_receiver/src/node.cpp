#include <ros/ros.h>

// handle image
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h> // convert opencv to ros message

#include <sensor_msgs/image_encodings.h> // ros message

#include <curl/curl.h> // download data from camera

#include <cassert>
#include <cstdint>
#include <sstream> // debug output
#include <algorithm> // min
#include <vector> // buffer


// Further information about techniques used in this node:
// http://answers.ros.org/question/99831/publish-file-to-image-topic/
// https://curl.haxx.se/libcurl/c/example.html
// http://stackoverflow.com/questions/14727267/opencv-read-jpeg-image-from-buffer


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


int main(int argc, char** argv)
{
  // setup ros
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<sensor_msgs::Image>("/web_cam_image", 1);

  // buffer that will be used to collect data from multiple callbacks for one GET request
  std::vector<std::uint8_t> current_message_buffer;

  // // setup curl
  CURL* curl = curl_easy_init();
  if (! curl)
  {
    ROS_FATAL("Unable to start curl, thus unable to get images.");
    return 1;
  }
  // // configure curl
  // set address
  curl_easy_setopt(curl, CURLOPT_URL, "http://10.0.126.9/snapshot.jpg?account=admin&password=1234");
  // set login information
  curl_easy_setopt(curl, CURLOPT_USERPWD, "admin:1234");
  // follow redirect if necessary
  curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
  // set callback to handle data
  curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, handle_image_from_web);
  // set p_userdata to the cv image that will transport the data
  curl_easy_setopt(curl, CURLOPT_WRITEDATA, static_cast<void*>(&current_message_buffer));


  // ros message that will get sent
  sensor_msgs::Image ros_image;
  ros_image.header.frame_id = "/odom";

  // setup cv bridge data structure which will be filled by handle_image_from_web
  cv_bridge::CvImage cv_image;
  // Note: to read an image from file cv_image.image = cv::imread("/tmp/snapshot.jpg", CV_LOAD_IMAGE_COLOR);
  cv_image.encoding = "bgr8";


  // main loop
  ros::Rate loop_rate(5);
  while (nh.ok())
  {
    // perform web request
    CURLcode res = curl_easy_perform(curl);
    // check for errors
    if(res == CURLE_OK)
    {
      ROS_DEBUG("curl GET performed sucessfully");

      // convert to image; data to interpret is in buffer
      // interpret data as image
      cv::Mat rawData(1, current_message_buffer.size(), CV_8UC1, current_message_buffer.data());
      cv::Mat decodedImage = cv::imdecode(rawData, CV_LOAD_IMAGE_COLOR);
      // check, whether interpretation worked
      if (decodedImage.data == 0)
      {
        // it did not work -- print error message
        std::stringstream s;
        for (size_t i = 0; i < std::min(current_message_buffer.size(), static_cast<size_t>(50)); ++i)
        {
          s << std::hex << static_cast<int>(current_message_buffer[i]);
        }
        ROS_ERROR_STREAM("Unable to interpret data from web as image: " << s.str());
      }
      else
      {
        // it worked; publish image
        ROS_DEBUG("Received data sucessfully converted to image; going to publish as ros image.");
        cv_image.image = decodedImage;
        cv_image.toImageMsg(ros_image);
        ros_image.header.stamp = ros::Time::now();
        pub.publish(ros_image);
      }
      // in any case, clear the buffer
      current_message_buffer.clear();

    }
    else
    {
      ROS_ERROR_STREAM("Error '" << curl_easy_strerror(res) << "' when trying to get an image");
    }

    loop_rate.sleep();
  }

  // tear down curl
  curl_easy_cleanup(curl);
}
