#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "geometry_msgs/Vector3.h"
#include "image_provider/interface_msg.h"


namespace enc = sensor_msgs::image_encodings;

//static const char WINDOW[] = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::Subscriber command_sub_;
  ros::Subscriber objects_sub_;

  ros::Publisher object_id_pub_;

  //Init Number of Object in Camera Image
  int num_objects_;

  //ID of currently selected object
  int curr_object_id_;

  //Coordinates of currently selected object
  float u_, v_;

  //ID of Object used for planning (published for planning node)
  int planning_id_;

  //Flag to check whether Confirm Command has been received
  bool confirm_triggered_;

  //Size of Image
  int im_width_, im_height_;

  
public:
  ImageConverter()
    : it_(nh_)
  {

    //Subscriber to get Objects in Camera Image and the BCI-command for object selection
    image_sub_ = it_.subscribe("/xtion/rgb/image_color", 1, &ImageConverter::imageCb,this);
    image_pub_ = it_.advertise("/image_provider/scene_visualization", 1);

    //Subscriber to get Objects in Camera Image and the BCI-command for object selection
    command_sub_ = nh_.subscribe("/command_publisher/bci_command", 1, &ImageConverter::receivedCommand,this);
    objects_sub_ = nh_.subscribe("/objects_2D", 1, &ImageConverter::receivedObjects,this);

    //Publisher to send the ID of the selected Object
    object_id_pub_ = nh_.advertise<std_msgs::Int32>("/object_ID", 10);


    //Init Object ID
    curr_object_id_ = 0;

    //Init Number of Object in Camera Image
    num_objects_ = 0;

    //Init Coordinates of currently selected object
    u_ = 0.0;
    v_ = 0.0;

    //Init ID of Object used for planning
    planning_id_ = 0;

    //Flag to check whether Confirm Command has been received
    confirm_triggered_ = false;

    //Init Image Size (only used to trigger initialization)
    im_height_ = 0;
    im_width_ = 0;


    //cv::namedWindow(WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(WINDOW);
  }
  
  void imageCb(const sensor_msgs::ImageConstPtr& msg)
   {
     cv_bridge::CvImagePtr cv_ptr;
     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }

     //ROS_INFO("%s", "CAM Alive");

     //Get Size of Image
     //cv::Size im_size = cv_ptr->image.size();
     if (im_height_ == 0 && im_width_ == 0)
     {
         im_height_ =  cv_ptr->image.rows;
         im_width_ = cv_ptr->image.cols;
         ROS_INFO("Image width [%i]",  im_height_);
         ROS_INFO("Image heigth [%i]",  im_height_);
     }

     //If Confirm hasn't been send so far
     if (confirm_triggered_ == false)
     {
         //Draw a circle at the location of the currently selected Object
         if (im_height_ > 60 && im_width_ > 60)
         {
           cv::circle(cv_ptr->image, cv::Point(u_, v_), 10, CV_RGB(0,255,0),10);
         }
     }

     //cv::imshow(WINDOW, cv_ptr->image);
     cv::waitKey(3);

     //Publish new image
     image_pub_.publish(cv_ptr->toImageMsg());
   }



    //Commands received from BCF Node
    void receivedCommand(const std_msgs::String::ConstPtr& msg)
    {
     std::string command =  msg->data.c_str();
     if (command == "Left")
     {
         if(curr_object_id_ > 0)
            curr_object_id_= curr_object_id_-1;
     }
     else if (command == "Right")
     {
         if(curr_object_id_ < (num_objects_-1))
            curr_object_id_= curr_object_id_+1;
     }
     else if (command == "Confirm")
     {
         //Set Confirm triggered flag to true
         confirm_triggered_ = true;

         std_msgs::Int32 object_id_msg;
         object_id_msg.data = planning_id_;
         ROS_INFO("Confirm selection of object with ID: [%d]", object_id_msg.data);
         object_id_pub_.publish(object_id_msg);
     }
     else
     {
       ROS_INFO("%s", "Command not valid");
     }

    }


    //Position and ID of Objects in Camera Image
    //TODO -> exchange message type to vector or whatever
    void receivedObjects(const image_provider::interface_msg& msg)
    {
        //Reset Confirm trigger flag
        confirm_triggered_ = false;

        //Set the number of objects received (num_objects_)
        num_objects_ = msg.objects.size();

        //ROS_INFO("%f", msg.objects.at(0).x);
        //ROS_INFO("Objects in camera image: [%i]", num_objects_);

        //Check if object with curr_object_id_ exists in received message (Object with ID = 1 should be always in message otherwise set the
        //coordinates u_,v_ to zero)
        if (num_objects_ == 0) //if camera image does not contain any objects
        {
            u_ = 0.0;
            v_ = 0.0;
        }
        else if(curr_object_id_ > (num_objects_-1)) //If last object dissappears, we need to set the ID to the new last object
        {
            curr_object_id_ = num_objects_-1;
        }
        else
        {
            //Compute image coordinates of currently selected object (x,y vary between -1 and 1 from left to right / top to buttom)
            u_ = im_width_/2 + (msg.objects.at(curr_object_id_).x * im_width_/2);    //horizontal coordinate
            v_ = im_height_/2 + (msg.objects.at(curr_object_id_).y * im_height_/2);  //vertical coordinate
            //Set ID used for planning (not identical with curr_object_id_!!!)
            planning_id_ = msg.objects.at(curr_object_id_).z;
        }

    }

 
}; 
