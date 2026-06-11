#pragma once

#include "ICamera.hpp"
#include <atomic>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>

namespace ST {
class UnityMockTCP : public ICamera {
public:
  explicit UnityMockTCP(u_int16_t port = 5000);
  ~UnityMockTCP() override;
  cv::Mat grabFrame() override;
  bool isOpen() const override;
  void close() override;
  const std::string &frameMetadata() const;
  void requestShutdown();

private:
  bool recvAll(void *buffer, size_t size);
  bool waitForConnection();

private:
  int m_serverFd{-1};
  int m_clientFd{-1};
  bool m_connected{false};
  std::string m_lastMetadata;
  bool waitReadable(int fd);
  int m_wakePipe[2] = {-1, -1};
  std::atomic<bool> m_shutdown{false};
};

} // namespace ST