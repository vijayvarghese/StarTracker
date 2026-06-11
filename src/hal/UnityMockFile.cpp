#include "startracker/hal/UnityMock.hpp"

namespace ST {

cv::Mat UnityMock::grabFrame() {

  cv::Mat img = cv::imread(file_path_, cv::IMREAD_COLOR);

  return img;
}

bool UnityMock::isOpen() const { return true; }

void UnityMock::close() {}

} // namespace ST