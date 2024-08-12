#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

void readImage(const char *filename, raisim::DepthCamera *depthSensor) {
  auto im = depthSensor->getDepthArray();
  std::vector<uint8_t> im_u8(im.size());
  for (int i = 0; i < im.size(); i++) {
    float depth = im[i];
    depth *= 10;
    if (std::isnan(depth))
      depth = 255.0;
    im_u8[i] = (uint8_t)depth;
  }
  stbi_write_png(filename, 64, 64, 1, im_u8.data(), 64);
}

int main(int argc, char *argv[]) {
  raisim::World world;
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  uint nGrid = 128;
  std::vector<double> heightVec(nGrid * nGrid, -1.0);
  world.addHeightMap(nGrid, nGrid, 10.0, 10.0, 0.0, 0.0, heightVec);
  // world.addGround(-2);
  world.setTimeStep(0.002);
  auto robot = world.addArticulatedSystem(
      binaryPath.getDirectory() + "/rsc/anymal/urdf/anymal_depth_sensor.urdf");

  Eigen::VectorXd gc;
  gc.setZero(robot->getGeneralizedCoordinateDim());
  gc << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03,
      -0.4, 0.8, -0.03, -0.4, 0.8;
  robot->setGeneralizedCoordinate(gc);

  auto depthSensor = robot->getSensorSet("depth_camera")
                         ->getSensor<raisim::DepthCamera>("depth");
  depthSensor->setMeasurementSource(raisim::Sensor::MeasurementSource::RAISIM);
  depthSensor->update(world);

  std::string fileName = binaryPath.getDirectory().getString() + "/depth_x.png";
  readImage(fileName.c_str(), depthSensor);

  for (int i = 0; i < 20; i++) {
    // rotate robot
    raisim::Mat<3, 3> rot;
    raisim::angleAxisToRotMat({0, 0, 1}, 0.2 * i, rot);
    raisim::Vec<4> quat;
    raisim::rotMatToQuat(rot, quat);
    gc.segment(3, 4) = quat.e();

    robot->setGeneralizedCoordinate(gc);
    depthSensor->update(world);

    fileName = binaryPath.getDirectory().getString() + "/depth_rotated_" +
               std::to_string(i) + ".png";
    readImage(fileName.c_str(), depthSensor);
  }
}
