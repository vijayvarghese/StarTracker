#include "startracker/hal/UnityMockFile.hpp"

namespace ST {

cv::Mat UnityMockFile::grabFrame() {

  cv::Mat img = cv::imread(file_path_, cv::IMREAD_COLOR);

  return img;
}

bool UnityMockFile::isOpen() const { return true; }

void UnityMockFile::close() {}

} // namespace ST