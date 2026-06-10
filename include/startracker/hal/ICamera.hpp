#pragma once

#include <opencv2/core/core.hpp>

namespace ST {

class ICamera {
public:
  virtual ~ICamera() = default;

  virtual cv::Mat grabFrame() = 0;

  virtual bool isOpen() const = 0;

  virtual void close() = 0;
};

} // namespace ST