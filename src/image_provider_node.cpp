

#include <image_provider/image_converter.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_provider_node");
  ImageConverter ic;
  ros::spin();
  return 0;
}
 
