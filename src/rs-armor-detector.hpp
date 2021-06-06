#pragma once

#include "seu-detect/Armor/ArmorDetector.h"
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <optional>

class Armor {
  public:
	enum class Color { RED, BLUE };

	/** 装甲板四个顶点的坐标 */
	std::vector<cv::Point2i> vertices;

	/** 装甲板中心在相机坐标系中的三维坐标 */
	cv::Point3f position;

	/** 装甲板中心到相机的距离 */
	double distance;

	/** 装甲板颜色 */
	Color color;
};

class DetectResult {
  public:
	rs2::frameset frameset;
	std::optional<Armor> detected_armor;
};

class RSArmorDetector {
  public:
	/**
	 * @param color_resolution RGB 帧的分辨率
	 * @param depth_resolution 深度帧的分辨率
	 */
	RSArmorDetector(cv::Size2i color_resolution, cv::Size2i depth_resolution);

	/** 拉取一帧并识别装甲板 */
	DetectResult poll_and_detect();

	/** 要识别的装甲板的颜色，默认红色 */
	Armor::Color color = Armor::Color::RED;

  private:
	/** RealSense 管线 */
	rs2::pipeline pipeline;

	/** 用于将深度帧对齐到 RGB 帧 */
	rs2::align align_to_color;

	/** RGB 相机内参 */
	rs2_intrinsics color_intrinsics;

	/** 装甲板识别实现，这里使用的是东南大学的开源装甲板识别方案 */
	rm::ArmorDetector armor_detector_impl;
};
