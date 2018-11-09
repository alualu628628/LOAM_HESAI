// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//
// and 
// To suit for Hesai Lidar Pandar 40 , this code is modified and refreshed by Pengdi Huang
// October 2018

#include "loam_velodyne/MultiScanRegistration.h"
#include "math_utils.h"

#include <pcl_conversions/pcl_conversions.h>


namespace loam {

MultiScanMapper::MultiScanMapper(const float& lowerBound,
                                 const float& upperBound,
                                 const uint16_t& nScanRings)
    : _lowerBound(lowerBound),
      _upperBound(upperBound),
      _nScanRings(nScanRings),
      _factor((nScanRings - 1) / (upperBound - lowerBound))
{
	HesaiAngleLookupTable();
}

void MultiScanMapper::set(const float &lowerBound,
                          const float &upperBound,
                          const uint16_t &nScanRings)
{
  _lowerBound = lowerBound;
  _upperBound = upperBound;
  _nScanRings = nScanRings;
  _factor = (nScanRings - 1) / (upperBound - lowerBound);
}

void MultiScanMapper::HesaiAngleLookupTable()
{

	_IDLineIndex.clear();
	_HesaiVerticalAngls.clear();

	pcl::PointXY oneVerAngle;
	oneVerAngle.x = 0.0;
	oneVerAngle.y = 0.0;
	////-16.00 - line 40 
	oneVerAngle.x = -16.00;
	_IDLineIndex.push_back(15);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-15.00 - line 39 
	oneVerAngle.x = -15.00;
	_IDLineIndex.push_back(14);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-14.00 - line 38 
	oneVerAngle.x = -14.00;
	_IDLineIndex.push_back(13);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-13.00 - line 37 
	oneVerAngle.x = -13.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-12.00 - line 36 
	oneVerAngle.x = -12.00;
	_IDLineIndex.push_back(12);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-11.00 - line 35 
	oneVerAngle.x = -11.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-10.00 - line 34 
	oneVerAngle.x = -10.00;
	_IDLineIndex.push_back(11);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-9.00 - line 33 
	oneVerAngle.x = -9.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-8.00 - line 32 
	oneVerAngle.x = -8.00;
	_IDLineIndex.push_back(10);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-7.00 - line 31 
	oneVerAngle.x = -7.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-6.00 - line 30 
	oneVerAngle.x = -6.00;
	_IDLineIndex.push_back(9);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-5.67 - line 29 
	oneVerAngle.x = -5.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-5.33 - line 28 
	oneVerAngle.x = -5.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-5.00 - line 27 
	oneVerAngle.x = -5.00;
	_IDLineIndex.push_back(8);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-4.67 - line 26 
	oneVerAngle.x = -4.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-4.33 - line 25 
	oneVerAngle.x = -4.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-4.00 - line 24 
	oneVerAngle.x = -4.00;
	_IDLineIndex.push_back(7);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-3.67 - line 23 
	oneVerAngle.x = -3.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-3.33 - line 22
	oneVerAngle.x = -3.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-3.00 - line 21 
	oneVerAngle.x = -3.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-2.67 - line 20 
	oneVerAngle.x = -2.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-2.33 - line 19 
	oneVerAngle.x = -2.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-2.00 - line 18 
	oneVerAngle.x = -2.00;
	_IDLineIndex.push_back(6);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-1.67 - line 17 
	oneVerAngle.x = -1.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-1.33 - line 16 
	oneVerAngle.x = -1.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-1.00 - line 15 
	oneVerAngle.x = -1.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-0.67 - line 14 
	oneVerAngle.x = -0.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////-0.33 - line 13 
	oneVerAngle.x = -0.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////0.00 - line 12 
	oneVerAngle.x = 0.00;
	_IDLineIndex.push_back(5);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////0.33 - line 11 
	oneVerAngle.x = 0.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////0.67 - line 10 
	oneVerAngle.x = 0.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////1.00 - line 09 
	oneVerAngle.x = 1.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////1.33 - line 08
	oneVerAngle.x = 1.33;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////1.67 - line 07 
	oneVerAngle.x = 1.67;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////2.00 - line 06 
	oneVerAngle.x = 2.00;
	_IDLineIndex.push_back(4);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////3.00 - line 05 
	oneVerAngle.x = 3.00;
	_IDLineIndex.push_back(-1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////4.00 - line 04 
	oneVerAngle.x = 4.00;
	_IDLineIndex.push_back(3);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////5.00 - line 03 
	oneVerAngle.x = 5.00;
	_IDLineIndex.push_back(2);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////6.00 - line 02 
	oneVerAngle.x = 6.00;
	_IDLineIndex.push_back(1);
	_HesaiVerticalAngls.push_back(oneVerAngle);
	////7.00 - line 01 
	oneVerAngle.x = 7.00;
	_IDLineIndex.push_back(0);
	_HesaiVerticalAngls.push_back(oneVerAngle);

	//generate a common kdtree for lookup table
	pcl::PointCloud<pcl::PointXY>::Ptr pHesaiVerticalAngls(new pcl::PointCloud<pcl::PointXY>);
	for (int i = 0; i != _HesaiVerticalAngls.size(); ++i)
		pHesaiVerticalAngls->points.push_back(_HesaiVerticalAngls[i]);

	HesaiAnglekdtree.setInputCloud(pHesaiVerticalAngls);

}


int MultiScanMapper::getRingForAngle(const float& angle) {
  return int(((angle * 180 / M_PI) - _lowerBound) * _factor + 0.5);
}



int MultiScanMapper::getRingForHesaiAngle(const float& angle) {

	//prepare searching
	pcl::PointXY searchAngle;
	searchAngle.x = angle * 180.0 / M_PI;
	//std::cout << "  angle:" << searchAngle.x << "  ";
	searchAngle.y = 0.0;
	std::vector<int> nearAnglIdx;
	std::vector<float> nearAnglDis;

	//search the ID of given angle value
	HesaiAnglekdtree.nearestKSearch(searchAngle, 1, nearAnglIdx, nearAnglDis);

	//find the corresponding index from lookup table
	int queryID = _IDLineIndex[nearAnglIdx[0]];
	return queryID;
}


MultiScanRegistration::MultiScanRegistration(const MultiScanMapper& scanMapper,
                                             const RegistrationParams& config)
    : ScanRegistration(config),
      _systemDelay(SYSTEM_DELAY),
      _scanMapper(scanMapper)
{

};



bool MultiScanRegistration::setup(ros::NodeHandle& node,
                                  ros::NodeHandle& privateNode)
{
  if (!ScanRegistration::setup(node, privateNode)) {
    return false;
  }

  // fetch scan mapping params
  std::string lidarName;

  if (privateNode.getParam("lidar", lidarName)) {
    if (lidarName == "VLP-16") {
      _scanMapper = MultiScanMapper::Velodyne_VLP_16();
    } else if (lidarName == "HDL-32") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_32();
    } else if (lidarName == "HDL-64E") {
      _scanMapper = MultiScanMapper::Velodyne_HDL_64E();
    } else {
      ROS_ERROR("Invalid lidar parameter: %s (only \"VLP-16\", \"HDL-32\" and \"HDL-64E\" are supported)", lidarName.c_str());
      return false;
    }

    ROS_INFO("Set  %s  scan mapper.", lidarName.c_str());
    if (!privateNode.hasParam("scanPeriod")) {
      _config.scanPeriod = 0.1;
      ROS_INFO("Set scanPeriod: %f", _config.scanPeriod);
    }
  } else {
    float vAngleMin, vAngleMax;
    int nScanRings;

    if (privateNode.getParam("minVerticalAngle", vAngleMin) &&
        privateNode.getParam("maxVerticalAngle", vAngleMax) &&
        privateNode.getParam("nScanRings", nScanRings)) {
      if (vAngleMin >= vAngleMax) {
        ROS_ERROR("Invalid vertical range (min >= max)");
        return false;
      } else if (nScanRings < 2) {
        ROS_ERROR("Invalid number of scan rings (n < 2)");
        return false;
      }

      _scanMapper.set(vAngleMin, vAngleMax, nScanRings);
      ROS_INFO("Set linear scan mapper from %g to %g degrees with %d scan rings.", vAngleMin, vAngleMax, nScanRings);
    }
  }


  // subscribe to input cloud topic
  _subLaserCloud = node.subscribe<sensor_msgs::PointCloud2>
      ("/multi_scan_points", 2, &MultiScanRegistration::handleCloudMessage, this);

  return true;
}



void MultiScanRegistration::handleCloudMessage(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
{
  if (_systemDelay > 0) {
    _systemDelay--;
    return;
  }

  // fetch new input cloud
  pcl::PointCloud<pcl::PointXYZ> laserCloudIn;
  pcl::fromROSMsg(*laserCloudMsg, laserCloudIn);

  process(laserCloudIn, laserCloudMsg->header.stamp);
}



void MultiScanRegistration::process(const pcl::PointCloud<pcl::PointXYZ>& laserCloudIn,
                                    const ros::Time& scanTime)
{
  size_t cloudSize = laserCloudIn.size();

  // reset internal buffers and set IMU start state based on current scan time
  reset(scanTime);

  // determine scan start and end orientations
  float startOri = -std::atan2(laserCloudIn[0].y, laserCloudIn[0].x);
  float endOri = -std::atan2(laserCloudIn[cloudSize - 1].y,
                             laserCloudIn[cloudSize - 1].x) + 2 * float(M_PI);
  if (endOri - startOri > 3 * M_PI) {
    endOri -= 2 * M_PI;
  } else if (endOri - startOri < M_PI) {
    endOri += 2 * M_PI;
  }

  bool halfPassed = false;
  pcl::PointXYZI point;
  std::vector<pcl::PointCloud<pcl::PointXYZI> > laserCloudScans(_scanMapper.getNumberOfScanRings());

  // extract valid points from input cloud
  for (int i = 0; i < cloudSize; i++) {
    point.x = laserCloudIn[i].y;
    point.y = laserCloudIn[i].z;
    point.z = laserCloudIn[i].x;

    // skip NaN and INF valued points
    if (!pcl_isfinite(point.x) ||
        !pcl_isfinite(point.y) ||
        !pcl_isfinite(point.z)) {
      continue;
    }

    // skip zero valued points
    if (point.x * point.x + point.y * point.y + point.z * point.z < 0.0001) {
      continue;
    }

    // calculate vertical point angle and scan ID
    float angle = std::atan(point.y / std::sqrt(point.x * point.x + point.z * point.z));
    int scanID = _scanMapper.getRingForHesaiAngle(angle);
    if (scanID == -1 ){
      continue;
    }

    // calculate horizontal point angle
    float ori = -std::atan2(point.x, point.z);
    if (!halfPassed) {
      if (ori < startOri - M_PI / 2) {
        ori += 2 * M_PI;
      } else if (ori > startOri + M_PI * 3 / 2) {
        ori -= 2 * M_PI;
      }

      if (ori - startOri > M_PI) {
        halfPassed = true;
      }
    } else {
      ori += 2 * M_PI;

      if (ori < endOri - M_PI * 3 / 2) {
        ori += 2 * M_PI;
      } else if (ori > endOri + M_PI / 2) {
        ori -= 2 * M_PI;
      }
    }

    // calculate relative scan time based on point orientation
    float relTime = _config.scanPeriod * (ori - startOri) / (endOri - startOri);
    point.intensity = scanID + relTime;

    // project point to the start of the sweep using corresponding IMU data
    if (hasIMUData()) {
      setIMUTransformFor(relTime);
      transformToStartIMU(point);
    }

    laserCloudScans[scanID].push_back(point);
  }

  // construct sorted full resolution cloud
  cloudSize = 0;
  for (int i = 0; i < _scanMapper.getNumberOfScanRings(); i++) {
    _laserCloud += laserCloudScans[i];

    IndexRange range(cloudSize, 0);
    cloudSize += laserCloudScans[i].size();
    range.second = cloudSize > 0 ? cloudSize - 1 : 0;
    _scanIndices.push_back(range);
  }

  // extract features
  extractFeatures();

  // publish result
  publishResult();
}

} // end namespace loam
