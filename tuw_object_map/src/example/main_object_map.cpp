#include <cstdio>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <GeographicLib/UTMUPS.hpp>

int find_utm_zone(const cv::Vec3d &lla)
{
  int zone;
  bool northp;
  double x, y, gamma, k;
  GeographicLib::UTMUPS::Forward(lla[0], lla[1], zone, northp, x, y, gamma, k);
  return zone;
}
cv::Vec3d &convert_LLA_to_UTM(const cv::Vec3d &lla, cv::Vec3d &utm, int setzone)
{
  int zone;
  bool northp;
  double x, y, gamma, k;
  GeographicLib::UTMUPS::Forward(lla[0], lla[1], zone, northp, x, y, gamma, k, setzone);
  utm[0] = x, utm[1] = y;
  return utm;
}

int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world tuw_object_map package\n");


  cv::Vec3d lla(46.8015409, 15.8382641, 338.917);
  cv::Vec3d utm; 
  int zone = find_utm_zone(lla);
  convert_LLA_to_UTM(lla, utm, zone);

  std::cout << "lla: " << lla << std::endl;
  std::cout << "utm: " << utm << std::endl;
  std::cout << "zone: " << zone << std::endl;
  return 0;
}
