#include "cv-helpers.hpp"
#include "rs-armor-detector.hpp"

/** 绘制带背景的多行文本 */
void draw_text_with_background(cv::Mat display, const std::string &text,
                               cv::Point position,
                               cv::Scalar background_color = {0x00, 0xa5, 0xff},
                               cv::Scalar text_color = {0xff, 0xff, 0xff},
                               int fontface = cv::FONT_HERSHEY_SIMPLEX,
                               double scale = .5, int thickness = 1) {
	std::string::size_type pos = 0;
	std::string::size_type pos_end;
	do {
		pos_end = text.find('\n', pos + 1);
		if (pos_end == std::string::npos) {
			pos_end = text.size();
		}
		auto line_text = text.substr(pos, pos_end - pos);

		int baseline = 0;
		cv::Size text_size =
		    cv::getTextSize(line_text, fontface, scale, thickness, &baseline);
		cv::rectangle(
		    display, position,
		    position + cv::Point{text_size.width, text_size.height + baseline},
		    background_color, cv::FILLED);
		cv::putText(display, line_text,
		            position + cv::Point{0, text_size.height}, fontface, scale,
		            text_color, thickness, 8);

		position.y += text_size.height + baseline;

		pos = pos_end + 1;
	} while (pos_end < text.size());
}

int main() {
	RSArmorDetector armor_detector{
	    {1280, 720}, // RGB 分辨率
	    {1280, 720}  // 深度分辨率
	};
	armor_detector.color = Armor::Color::RED; // 识别红色装甲板

	for (;;) {
		auto result = armor_detector.poll_and_detect();
		auto img = frame_to_mat(result.frameset.get_color_frame());

		if (result.detected_armor.has_value()) {
			auto &armor = result.detected_armor.value();

			// 绘制四边形线框
			cv::Scalar line_color;
			if (armor.color == Armor::Color::RED) {
				line_color = {0, 0, 0xff};
			} else {
				line_color = {0xff, 0, 0};
			}
			cv::polylines(img, armor.vertices, true, line_color, 2);

			// 显示距离及坐标
			std::stringstream caption;
			caption << std::setprecision(2) << std::fixed
			        << "distance: " << armor.distance << "m\n"
			        << "(" << armor.position.x << "," << armor.position.y << ","
			        << armor.position.z << ")";
			int caption_x =
			    std::max({armor.vertices[0].x, armor.vertices[1].x,
			              armor.vertices[2].x, armor.vertices[3].x});
			int caption_y =
			    std::max({armor.vertices[0].y, armor.vertices[1].y,
			              armor.vertices[2].y, armor.vertices[3].y});
			draw_text_with_background(img, caption.str(),
			                          {caption_x, caption_y}, line_color);
		}

		cv::imshow("image", img);
		cv::waitKey(1);
	}
}
