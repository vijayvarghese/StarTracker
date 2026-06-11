#include "startracker/hal/UnityMockTCP.hpp"

#include <opencv2/imgcodecs.hpp>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <iostream>
#include <vector>

namespace ST {

namespace {

struct PacketHeader {
  uint32_t frameId;
  uint32_t jsonSize;
  uint32_t imageSize;
};

} // namespace

UnityMockTCP::UnityMockTCP(uint16_t port) {
  m_serverFd = socket(AF_INET, SOCK_STREAM, 0);
  if (pipe(m_wakePipe) < 0) {
    throw std::runtime_error("Failed to create wake pipe");
  }

  if (m_serverFd < 0) {
    throw std::runtime_error("Failed to create socket");
  }

  int opt = 1;

  setsockopt(m_serverFd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

  sockaddr_in addr{};

  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = INADDR_ANY;
  addr.sin_port = htons(port);

  if (bind(m_serverFd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    throw std::runtime_error("Failed to bind");
  }

  if (listen(m_serverFd, 1) < 0) {
    throw std::runtime_error("Failed to listen");
  }

  std::cout << "Waiting for Unity connection..." << std::endl;
}

UnityMockTCP::~UnityMockTCP() { close(); }

bool UnityMockTCP::waitForConnection() {
  if (m_connected)
    return true;
  std::cout << "Waiting for Unity connection..." << std::endl;
  if (!waitReadable(m_serverFd)) {
    std::cout << "Shutdown requested" << std::endl;
    return false;
  }
  sockaddr_in clientAddr{};

  socklen_t len = sizeof(clientAddr);

  m_clientFd =
      accept(m_serverFd, reinterpret_cast<sockaddr *>(&clientAddr), &len);
  std::cout << "accept returned" << std::endl;
  std::cout << "client fd=" << m_clientFd << std::endl;
  if (m_clientFd < 0)
    return false;

  m_connected = true;

  std::cout << "Unity connected" << std::endl;

  return true;
}

bool UnityMockTCP::recvAll(void *buffer, size_t size) {
  auto *ptr = static_cast<uint8_t *>(buffer);

  size_t received = 0;

  while (received < size) {
    ssize_t n = recv(m_clientFd, ptr + received, size - received, 0);

    if (n <= 0) {
      m_connected = false;
      return false;
    }

    received += n;
  }

  return true;
}

cv::Mat UnityMockTCP::grabFrame() {
  if (!waitForConnection()) {
    return {};
  }

  PacketHeader hdr{};
  std::cout << "waiting for header" << std::endl;
  if (!recvAll(&hdr, sizeof(hdr))) {
    std::cout << "header failed" << std::endl;
    return {};
  }
  std::cout << "frame=" << hdr.frameId << " json=" << hdr.jsonSize
            << " img=" << hdr.imageSize << std::endl;
  std::vector<char> json(hdr.jsonSize);

  if (!recvAll(json.data(), hdr.jsonSize)) {
    return {};
  }

  m_lastMetadata.assign(json.begin(), json.end());

  std::vector<uint8_t> image(hdr.imageSize);

  if (!recvAll(image.data(), hdr.imageSize)) {
    return {};
  }

  cv::Mat encoded(1, static_cast<int>(image.size()), CV_8UC1, image.data());
  cv::Mat decoded = cv::imdecode(encoded, cv::IMREAD_COLOR);
  return decoded;
}

bool UnityMockTCP::isOpen() const { return m_connected; }

void UnityMockTCP::close() {
  if (m_clientFd >= 0) {
    ::close(m_clientFd);
    m_clientFd = -1;
  }
  if (m_serverFd >= 0) {
    ::close(m_serverFd);
    m_serverFd = -1;
  }
  if (m_wakePipe[0] >= 0) {
    ::close(m_wakePipe[0]);
    m_wakePipe[0] = -1;
  }
  if (m_wakePipe[1] >= 0) {
    ::close(m_wakePipe[1]);
    m_wakePipe[1] = -1;
  }
  m_connected = false;
}

const std::string &UnityMockTCP::frameMetadata() const {
  return m_lastMetadata;
}

void UnityMockTCP::requestShutdown() {
  m_shutdown.store(true);
  uint8_t byte = 1;
  ::write(m_wakePipe[1], &byte, 1); // wakes select() immediately
}

bool UnityMockTCP::waitReadable(int fd) {
  while (true) {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(fd, &readfds);
    FD_SET(m_wakePipe[0], &readfds);

    int maxFd = std::max(fd, m_wakePipe[0]) + 1;
    int ret = select(maxFd, &readfds, nullptr, nullptr, nullptr);

    if (ret < 0) {
      if (errno == EINTR)
        continue; // signal interrupted, retry
      return false;
    }

    if (FD_ISSET(m_wakePipe[0], &readfds))
      return false; // shutdown
    if (FD_ISSET(fd, &readfds))
      return true; // ready
  }
}

} // namespace ST