#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

#include "ICamera.hpp"

namespace ST {

class UnityMockFile : public ICamera {

private:
  std::string file_path_;
  int frame_index = 0;

public:
  explicit UnityMockFile(const std::string &file_path)
      : file_path_(file_path) {}

  cv::Mat grabFrame() override;

  bool isOpen() const override;

  void close() override;
};

} // namespace ST