#include <cstdio>
#include <iostream>
#include <tuw_object_map/object_map.hpp>

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world tuw_object_map package\n");


  cv::Vec3d lla(46.8015409, 15.8382641, 338.917);
  cv::Vec3d utm; 
  int zone = tuw_object_map::ObjectMap::find_utm_zone(lla);
  tuw_object_map::ObjectMap::convert_LLA_to_UTM(lla, utm, zone);

  std::cout << "lla: " << lla << std::endl;
  std::cout << "utm: " << utm << std::endl;
  std::cout << "zone: " << zone << std::endl;
  return 0;
}
