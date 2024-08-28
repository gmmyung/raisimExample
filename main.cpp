#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb/stb_image_write.h"

void readImage(const char *filename, raisim::DepthCamera *depthSensor,
               int width, int height) {
  auto im = depthSensor->getDepthArray();
  std::vector<uint8_t> im_u8(im.size());
  for (int i = 0; i < im.size(); i++) {
    float depth = im[i];
    depth *= 20;
    if (std::isnan(depth))
      depth = 255.0;
    im_u8[i] = (uint8_t)depth;
  }
  stbi_write_png(filename, width, height, 1, im_u8.data(), width);
}

int main(int argc, char *argv[]) {
  raisim::World world;
  auto binaryPath = raisim::Path::setFromArgv(argv[0]);
  uint nGrid = 128;
  std::vector<double> heightVec(nGrid * nGrid, -0.5);
  world.addHeightMap(nGrid, nGrid, 10.0, 10.0, 0.0, 0.0, heightVec);
  auto robot = world.addArticulatedSystem(binaryPath.getDirectory() +
                                          "/rsc/raibo2/urdf/raibo2.urdf");

  Eigen::VectorXd gc;
  gc.setZero(robot->getGeneralizedCoordinateDim());
  gc << 0, 0, 0.50, 1.0, 0.0, 0.0, 0.0, 0, 0.580099, -1.195, 0, 0.580099,
      -1.195, 0, 0.580099, -1.195, 0, 0.580099, -1.195;

  robot->setGeneralizedCoordinate(gc);

  auto depthSensor = robot->getSensorSet("d430_front")
                         ->getSensor<raisim::DepthCamera>("depth");
  auto badDepthSensor = robot->getSensorSet("d430_front_bad")
                            ->getSensor<raisim::DepthCamera>("depth");

  depthSensor->setMeasurementSource(raisim::Sensor::MeasurementSource::RAISIM);
  badDepthSensor->setMeasurementSource(
      raisim::Sensor::MeasurementSource::RAISIM);
  world.setTimeStep(0.002);
  depthSensor->update(world);
  badDepthSensor->update(world);

  std::string fileName = binaryPath.getDirectory().getString() + "/x.png";
  readImage(fileName.c_str(), depthSensor, 64, 64);

  fileName = binaryPath.getDirectory().getString() + "/x_bad.png";
  readImage(fileName.c_str(), badDepthSensor, 64, 36);
}
