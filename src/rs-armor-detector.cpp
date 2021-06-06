#include "rs-armor-detector.hpp"
#include "cv-helpers.hpp"
#include <librealsense2/rsutil.h>

RSArmorDetector::RSArmorDetector(cv::Size2i color_resolution,
                                 cv::Size2i depth_resolution)
    : align_to_color(RS2_STREAM_COLOR) {

	rs2::config rs_config;
	// 启用 RGB 流
	rs_config.enable_stream(RS2_STREAM_COLOR, color_resolution.width,
	                        color_resolution.height, RS2_FORMAT_BGR8);
	// 启用深度流
	rs_config.enable_stream(RS2_STREAM_DEPTH, depth_resolution.width,
	                        depth_resolution.height, RS2_FORMAT_Z16);

	auto pipeline_profile = pipeline.start(rs_config); // 启动管线

	// 获取 RGB 相机内参
	auto color_stream = pipeline_profile.get_stream(RS2_STREAM_COLOR)
	                        .as<rs2::video_stream_profile>();
	color_intrinsics = color_stream.get_intrinsics();

	// 初始化装甲板识别
	rm::ArmorParam armor_param;
	armor_detector_impl.init(armor_param);
}

/** 计算多边形中心点 */
cv::Point2f center_point(const std::vector<cv::Point2f> &polygon) {
	cv::Point2f sum;
	for (auto &p : polygon) {
		sum += p;
	}
	return sum / static_cast<float>(polygon.size());
}

/** 将多边形缩小一定比例，保持中心点不变 */
std::vector<cv::Point2f> shrink_polygon(const std::vector<cv::Point2f> &in,
                                        float shrink_ratio) {
	auto center = center_point(in);
	auto result = in;
	for (auto &p : result) {
		p = (p - center) * (1 - shrink_ratio) + center;
	}
	return result;
}

/** 计算四边形区域内的平均深度值 */
uint16_t average_depth_in_quad(const cv::Mat &frame,
                               const std::vector<cv::Point2f> &quad) {
	std::vector<cv::Point> pts;
	for (auto &p : quad) {
		pts.push_back({static_cast<int>(std::round(p.x)),
		               static_cast<int>(std::round(p.y))});
	}

	int min_x = std::max(std::min({pts[0].x, pts[1].x, pts[2].x, pts[3].x}), 0);
	int max_x = std::min(std::max({pts[0].x, pts[1].x, pts[2].x, pts[3].x}),
	                     frame.cols - 1);
	int min_y = std::max(std::min({pts[0].y, pts[1].y, pts[2].y, pts[3].y}), 0);
	int max_y = std::min(std::max({pts[0].y, pts[1].y, pts[2].y, pts[3].y}),
	                     frame.rows - 1);

	cv::Mat1b mask(frame.rows, frame.cols, uchar(0));
	cv::fillConvexPoly(mask, pts, cv::Scalar(0xff));

	uint64_t depth_sum = 0;
	size_t samples = 0;
	for (int y = min_y; y <= max_y; y++) {
		for (int x = min_x; x <= max_x; x++) {
			if (mask.at<uchar>(y, x) == 0)
				continue;
			auto depth = frame.at<uint16_t>(y, x);
			if (depth == 0)
				continue;
			samples++;
			depth_sum += depth;
		}
	}
	return samples == 0 ? 0 : depth_sum / samples;
}

DetectResult RSArmorDetector::poll_and_detect() {
	auto frameset = pipeline.wait_for_frames();  // 拉取 RGB 及深度帧
	frameset = align_to_color.process(frameset); // 将深度帧对齐到 RGB 帧

	auto color_frame = frameset.get_color_frame();
	auto depth_frame = frameset.get_depth_frame();

	DetectResult result;
	result.frameset = frameset;

	// 识别装甲板
	if (color == Armor::Color::RED) {
		armor_detector_impl.setEnemyColor(rm::RED);
	} else {
		armor_detector_impl.setEnemyColor(rm::BLUE);
	}
	armor_detector_impl.loadImg(frame_to_mat(color_frame));
	bool detected =
	    armor_detector_impl.detect() == rm::ArmorDetector::ARMOR_LOCAL;

	if (detected) {
		auto vertices = armor_detector_impl.getArmorVertex(); // 装甲板四个顶点

		// 将装甲板四边形缩小一定比例，并计算该区域内的平均深度
		auto shrinked_armor_quad = shrink_polygon(vertices, 0.3);
		auto raw_depth = average_depth_in_quad(frame_to_mat(depth_frame),
		                                       shrinked_armor_quad);

		if (raw_depth != 0) {
			// 深度值存在
			float distance = raw_depth * depth_frame.get_units(); // 实际距离

			// 根据深度值将二维点投射为三维点
			auto center = center_point(vertices);
			float pos_2d[] = {center.x, center.y};
			float pos_3d[3];
			rs2_deproject_pixel_to_point(pos_3d, &color_intrinsics, pos_2d,
			                             distance);
			cv::Point3f position{pos_3d[0], pos_3d[1], pos_3d[2]}; // 三维坐标

			Armor armor;
			armor.color = color;
			for (auto &vertex : vertices) {
				armor.vertices.push_back(
				    {static_cast<int>(std::round(vertex.x)),
				     static_cast<int>(std::round(vertex.y))});
			}
			armor.distance = distance;
			armor.position = position;
			result.detected_armor = armor;
		}
	}

	return result;
}
