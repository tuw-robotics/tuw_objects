#include <cstdio>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tuw_geometry/geo_map.hpp>

void onMouse(int event, int x, int y, int, void *userdata)
{
  tuw::GeoMapMetaData *info = (tuw::GeoMapMetaData *)userdata;
  cv::Vec3d lla = info->m2g(cv::Point(x,y));
  if (event == cv::EVENT_LBUTTONDOWN)
  {
    std::cout << "Left button clicked at position: (" << x << ", " << y << ") > " << lla << std::endl;
  }
  else if (event == cv::EVENT_RBUTTONDOWN)
  {
    std::cout << "Right button clicked at position: (" << x << ", " << y << ") > " << lla << std::endl;
  }
}

int main(int argc, char **argv)
{
  (void)argc;
  (void)argv;

  printf("hello world tuw_object_map package\n");
  printf("example: geo_map\n");
  tuw::GeoMapMetaData info;

  info.size = cv::Size(600, 400);
  info.resolution = 1. / 5.0;
  info.init(46.8015409, 15.8382641, 338.917);

  std::cout << info.info_map() << std::endl;
  std::cout << info.info_geo() << std::endl;

  cv::Vec3d geo_lla_r0_0(46.8015409, 15.8382641, 338.917);
  cv::Vec3d geo_lla_r0_1(46.8015662, 15.8379026, 343.067);

  cv::Vec3d geo_lla_r1_0(46.8015624, 15.8382788, 338.668);
  cv::Vec3d geo_lla_r1_1(46.8016036, 15.8377093, 345.751);

  cv::Vec3d geo_lla_r2_0(46.8015856, 15.8382938, 338.730);
  cv::Vec3d geo_lla_r2_1(46.8016587, 15.8372159, 351.361);

  std::vector<cv::Vec3d> geo_points({geo_lla_r0_0, geo_lla_r0_1, geo_lla_r1_0, geo_lla_r1_1, geo_lla_r2_0, geo_lla_r2_1});

  std::vector<cv::Vec3d> utm_points;
  for (auto &p_lla : geo_points)
  {
    utm_points.push_back(info.lla2utm(p_lla));
    std::cout << "utm: " << utm_points.back() << std::endl;
  }
  std::vector<cv::Vec3d> world_points;
  for (auto &p_utm : utm_points)
  {
    world_points.push_back(info.utm2world(p_utm));
    std::cout << "world: " << world_points.back() << std::endl;
  }
  std::vector<cv::Vec2d> map_points;
  for (auto &p_world : world_points)
  {
    map_points.push_back(info.world2map(p_world));
    std::cout << map_points.back() << std::endl;
  }
  for (size_t i = 0; i < geo_points.size(); i++)
  {
    printf("geo: [%10.8f, %10.8f, %10.8f] > utm: [%10.8f, %10.8f, %10.8f] m > world: [% 10.8f, % 10.8f, % 10.8f] m > map: [%5.1f, %5.1f] px\n",
           geo_points[i][0], geo_points[i][1], geo_points[i][2], utm_points[i][0], utm_points[i][1], utm_points[i][2],
           map_points[i][0], map_points[i][1], map_points[i][2], map_points[i][0], map_points[i][1]);
  }
  for (size_t i = 0; i < geo_points.size(); i++)
  {
    cv::Point pm = info.g2m(geo_points[i]);
    cv::Vec3d pg = info.m2g(pm);
    printf("geo: [%10.8f, %10.8f, %10.8f] > map: [%5d, %5d] px > geo: [%10.8f, %10.8f, %10.8f] > err: [%10.8f, %10.8f, %10.8f]\n", 
      geo_points[i][0], geo_points[i][1], geo_points[i][2], 
      pm.x, pm.y, 
      pg[0], pg[1], pg[2],
      pg[0] - geo_points[i][0], pg[1] - geo_points[i][1], pg[2] - geo_points[i][2]);
  }

  cv::Mat map(info.size, CV_8UC1, cv::Scalar(0x80));
  for (size_t i = 0; i < geo_points.size(); i += 2)
  {
    cv::line(map, info.g2m(geo_points[i]), info.g2m(geo_points[i + 1]), cv::Scalar(0xFF), 1);
  }

  cv::imshow("map", map);
  cv::setMouseCallback("map", onMouse, &info);
  cv::waitKey(10000);

  return 0;
}
