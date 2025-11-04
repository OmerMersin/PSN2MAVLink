// Cross-platform PSN -> MAVLink bridge

#include <chrono>
#include <cstdint>
#include <cstring>
#include <cerrno>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <vector>
#include <cmath> 
#include <system_error>
#include <algorithm>
#include <cctype>
#include <limits>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>

#ifdef _WIN32
  #ifndef WIN32_LEAN_AND_MEAN
    #define WIN32_LEAN_AND_MEAN
  #endif
  #ifndef NOMINMAX
    #define NOMINMAX
  #endif
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  #include <windows.h>
  using socklen_t = int;
  using ssize_t   = long long;
  #define CLOSESOCK closesocket
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <unistd.h>
  #include <fcntl.h>
  #include <termios.h>
  #include <sys/types.h>
  #include <sys/stat.h>
  #define CLOSESOCK close
#endif

extern "C" {
  #include <common/mavlink.h>
}


// ---------- PSN constants ----------
namespace psn {
  static constexpr const char* DEFAULT_GROUP = "236.10.10.10";
  static constexpr uint16_t    DEFAULT_PORT  = 56565;

  static constexpr uint16_t PSN_DATA_PACKET        = 0x6755;
  static constexpr uint16_t PSN_DATA_PACKET_HEADER = 0x0000;
  static constexpr uint16_t PSN_DATA_TRACKER_LIST  = 0x0001;
  static constexpr uint16_t PSN_DATA_TRACKER_POS   = 0x0000; // inside tracker
  static constexpr uint16_t PSN_DATA_TRACKER_SPEED = 0x0001;

#pragma pack(push,1)
  struct ChunkHeader {
    uint16_t id;
    uint16_t data_len_and_flag; // low 15 = len, high bit = has_subchunks
    bool has_subchunks() const { return (data_len_and_flag & 0x8000u) != 0; }
    uint16_t data_len()   const { return (uint16_t)(data_len_and_flag & 0x7FFFu); }
  };
  static_assert(sizeof(ChunkHeader)==4, "ChunkHeader must be 4 bytes");

  struct DataPacketHeader {
    uint64_t packet_timestamp_us;
    uint8_t  version_high;
    uint8_t  version_low;
    uint8_t  frame_id;
    uint8_t  frame_packet_count;
  };
#pragma pack(pop)
}

// -------- byte cursor --------
class Cursor {
public:
  Cursor(const uint8_t* data, size_t len): p_(data), end_(data+len) {}
  bool ok(size_t n=0) const { return p_+n <= end_; }
  size_t remain() const { return (size_t)(end_-p_); }
  const uint8_t* ptr() const { return p_; }
  template<typename T> bool readLE(T& out) {
    if (!ok(sizeof(T))) return false;
    std::memcpy(&out, p_, sizeof(T)); // little-endian host assumed
    p_ += sizeof(T);
    return true;
  }
  bool skip(size_t n){ if(!ok(n)) return false; p_+=n; return true; }
private:
  const uint8_t* p_;
  const uint8_t* end_;
};

struct TrackerPose {
  uint16_t id{};
  float x{},y{},z{}; bool has_pos{false};
  float vx{},vy{},vz{}; bool has_vel{false};
};

static bool readChunkHeader(Cursor& c, psn::ChunkHeader& ch){ return c.readLE(ch); }

static void parseTrackerChildren(uint16_t tracker_id, Cursor& c, size_t data_len,
                                 std::map<uint16_t, TrackerPose>& out){
  const uint8_t* start = c.ptr();
  TrackerPose& tp = out[tracker_id];
  tp.id = tracker_id;
  while ((size_t)(c.ptr()-start) < data_len && c.ok()){
    psn::ChunkHeader ch{};
    if (!readChunkHeader(c, ch)) return;
    switch(ch.id){
      case psn::PSN_DATA_TRACKER_POS: {
        float x,y,z;
        Cursor c2(c.ptr(), ch.data_len());
        if (c2.readLE(x) && c2.readLE(y) && c2.readLE(z)){
          tp.x=x; tp.y=y; tp.z=z; tp.has_pos=true;
        }
        c.skip(ch.data_len());
      } break;
      case psn::PSN_DATA_TRACKER_SPEED: {
        float vx,vy,vz;
        Cursor c2(c.ptr(), ch.data_len());
        if (c2.readLE(vx)&&c2.readLE(vy)&&c2.readLE(vz)){
          tp.vx=vx; tp.vy=vy; tp.vz=vz; tp.has_vel=true;
        }
        c.skip(ch.data_len());
      } break;
      default: c.skip(ch.data_len()); break;
    }
  }
}

static void parseTrackerList(Cursor& c, size_t data_len,
                             std::map<uint16_t, TrackerPose>& out){
  const uint8_t* start = c.ptr();
  while ((size_t)(c.ptr()-start) < data_len && c.ok()){
    psn::ChunkHeader tr{};
    if (!readChunkHeader(c, tr)) return;
    Cursor child(c.ptr(), tr.data_len());
    parseTrackerChildren(tr.id, child, tr.data_len(), out);
    c.skip(tr.data_len());
  }
}

static bool parsePSNPacket(const uint8_t* buf, size_t len,
                           std::map<uint16_t, TrackerPose>& poses){
  Cursor c(buf, len);
  psn::ChunkHeader root{};
  if (!readChunkHeader(c, root)) return false;
  if (root.id != psn::PSN_DATA_PACKET) return false;

  psn::ChunkHeader hdr{};
  if (!readChunkHeader(c, hdr)) return false;
  if (hdr.id != psn::PSN_DATA_PACKET_HEADER) return false;

  psn::DataPacketHeader dph{};
  if (hdr.data_len() < sizeof(psn::DataPacketHeader)) return false;
  Cursor hcur(c.ptr(), hdr.data_len());
  if (!hcur.readLE(dph)) return false;
  c.skip(hdr.data_len());

  const size_t bytes_to_read = root.data_len();
  const uint8_t* start = c.ptr();
  while ((size_t)(c.ptr()-start) < bytes_to_read && c.ok()){
    psn::ChunkHeader ch{};
    if (!readChunkHeader(c, ch)) break;
    if (ch.id == psn::PSN_DATA_TRACKER_LIST){
      parseTrackerList(c, ch.data_len(), poses);
    } else {
      c.skip(ch.data_len());
    }
  }
  return true;
}

static std::optional<in_addr> parse_ipv4_addr(const std::string& ip){
  in_addr addr{};
#ifdef _WIN32
  if (InetPtonA(AF_INET, ip.c_str(), &addr) != 1) return std::nullopt;
#else
  if (inet_pton(AF_INET, ip.c_str(), &addr) != 1) return std::nullopt;
#endif
  return addr;
}

static bool is_multicast_ipv4(const std::string& ip){
  auto addr = parse_ipv4_addr(ip);
  if (!addr) return false;
  uint32_t host_order = ntohl(addr->s_addr);
  return (host_order >= 0xE0000000u) && (host_order <= 0xEFFFFFFFu);
}

static int last_socket_error(){
#ifdef _WIN32
  return WSAGetLastError();
#else
  return errno;
#endif
}

static std::string format_socket_error(int err){
  std::error_code ec(err, std::system_category());
  std::string msg = ec.message();
  if (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r')){
    while (!msg.empty() && (msg.back() == '\n' || msg.back() == '\r')) msg.pop_back();
  }
  return std::to_string(err) + (msg.empty() ? std::string() : (" (" + msg + ")"));
}

static int last_os_error(){
#ifdef _WIN32
  return (int)GetLastError();
#else
  return errno;
#endif
}

static bool looks_like_serial_target(const std::string& target){
#ifdef _WIN32
  if (target.rfind("\\\\.\\", 0) == 0) return true;
  if (target.size() >= 3 && (target.rfind("COM", 0) == 0 || target.rfind("com", 0) == 0)) return true;
  return false;
#else
  return target.rfind("/dev/", 0) == 0;
#endif
}

#ifndef _WIN32
static std::optional<speed_t> baud_to_speed(int baud){
  switch (baud){
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
#ifdef B921600
    case 921600: return B921600;
#endif
    default: return std::nullopt;
  }
}
#endif

#ifdef _WIN32
static bool write_serial_blocking(HANDLE handle, const uint8_t* data, size_t len){
  if (handle == INVALID_HANDLE_VALUE) return false;
  size_t total = 0;
  while (total < len){
    DWORD chunk = (DWORD)std::min<size_t>(len - total, std::numeric_limits<DWORD>::max());
    DWORD written = 0;
    if (!WriteFile(handle, data + total, chunk, &written, nullptr)) return false;
    if (written == 0) return false;
    total += static_cast<size_t>(written);
  }
  return true;
}
#else
static bool write_serial_blocking(int fd, const uint8_t* data, size_t len){
  if (fd < 0) return false;
  size_t total = 0;
  while (total < len){
    ssize_t w = write(fd, data + total, len - total);
    if (w < 0){
      if (errno == EINTR) continue;
      if (errno == EAGAIN || errno == EWOULDBLOCK){
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      return false;
    }
    if (w == 0){
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    total += (size_t)w;
  }
  return true;
}
#endif

enum class MavlinkTransport {
  UDP,
  TCP,
  SERIAL
};

class SerialAsyncWriter {
public:
  SerialAsyncWriter() = default;
  SerialAsyncWriter(const SerialAsyncWriter&) = delete;
  SerialAsyncWriter& operator=(const SerialAsyncWriter&) = delete;

#ifdef _WIN32
  bool start(HANDLE handle){
    if (handle == INVALID_HANDLE_VALUE) return false;
    if (running_.load()) return false;
    handle_ = handle;
    running_.store(true);
    worker_ = std::thread(&SerialAsyncWriter::run, this);
    return true;
  }
#else
  bool start(int fd){
    if (fd < 0) return false;
    if (running_.load()) return false;
    fd_ = fd;
    running_.store(true);
    worker_ = std::thread(&SerialAsyncWriter::run, this);
    return true;
  }
#endif

  void stop(){
    if (!running_.exchange(false)) return;
    cv_.notify_all();
    if (worker_.joinable()) worker_.join();
    std::lock_guard<std::mutex> lock(mtx_);
    queue_.clear();
    queued_bytes_ = 0;
  }

  ~SerialAsyncWriter(){ stop(); }

  bool enqueue(const uint8_t* data, size_t len){
    if (len == 0) return true;
    if (!running_.load()) return false;
    std::vector<uint8_t> buffer(data, data + len);
    bool dropped = false;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!running_.load()) return false;
      while (queued_bytes_ + buffer.size() > max_queue_bytes_ && !queue_.empty()){
        queued_bytes_ -= queue_.front().size();
        queue_.pop_front();
        dropped = true;
      }
      queue_.push_back(std::move(buffer));
      queued_bytes_ += len;
    }
    if (dropped) maybe_log_drop();
    cv_.notify_one();
    return true;
  }

private:
  void run(){
    while (true){
      std::vector<uint8_t> payload;
      {
        std::unique_lock<std::mutex> lock(mtx_);
        cv_.wait(lock, [&]{ return !queue_.empty() || !running_.load(); });
        if (queue_.empty()){
          if (!running_.load()) break;
          continue;
        }
        payload = std::move(queue_.front());
        queue_.pop_front();
        queued_bytes_ -= payload.size();
      }
#ifdef _WIN32
      if (!write_serial_blocking(handle_, payload.data(), payload.size())){
        report_serial_error();
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.clear();
        queued_bytes_ = 0;
      }
#else
      if (!write_serial_blocking(fd_, payload.data(), payload.size())){
        report_serial_error();
        std::lock_guard<std::mutex> lock(mtx_);
        queue_.clear();
        queued_bytes_ = 0;
      }
#endif
    }
  }

  void maybe_log_drop(){
    auto now = std::chrono::steady_clock::now();
    if (now - last_drop_log_ < std::chrono::seconds(1)) return;
    last_drop_log_ = now;
    std::cerr << "Serial send queue full; dropping oldest MAVLink frame. Consider using higher baud or reducing rate.\n";
  }

  void report_serial_error(){
    auto now = std::chrono::steady_clock::now();
    if (now - last_error_log_ < std::chrono::seconds(1)) return;
    last_error_log_ = now;
#ifdef _WIN32
    int err = last_os_error();
#else
    int err = errno;
#endif
    std::cerr << "Serial write failed: " << format_socket_error(err) << "\n";
  }

#ifdef _WIN32
  HANDLE handle_{INVALID_HANDLE_VALUE};
#else
  int fd_{-1};
#endif
  std::mutex mtx_;
  std::condition_variable cv_;
  std::deque<std::vector<uint8_t>> queue_;
  std::thread worker_;
  std::atomic<bool> running_{false};
  size_t queued_bytes_{0};
  const size_t max_queue_bytes_{32 * 1024};
  std::chrono::steady_clock::time_point last_drop_log_{};
  std::chrono::steady_clock::time_point last_error_log_{};
};

struct MavlinkEndpoint {
  MavlinkTransport type{MavlinkTransport::UDP};
  int socket_fd{-1};
  sockaddr_in udp_to{};
#ifdef _WIN32
  HANDLE serial_handle{INVALID_HANDLE_VALUE};
#else
  int serial_fd{-1};
#endif
  std::unique_ptr<SerialAsyncWriter> serial_writer;
  // udpin/listen support
  bool udpin_listener{false};
  std::atomic<bool> udpin_running{false};
  std::thread udpin_thread;
  std::mutex udp_mtx; // protects udp_to and has_udp_remote
  bool has_udp_remote{false};

  // Default constructor
  MavlinkEndpoint() = default;
  
  // Delete copy operations (non-copyable due to mutex, atomic, thread)
  MavlinkEndpoint(const MavlinkEndpoint&) = delete;
  MavlinkEndpoint& operator=(const MavlinkEndpoint&) = delete;
  
  // Move constructor
  MavlinkEndpoint(MavlinkEndpoint&& other) noexcept
    : type(other.type), socket_fd(other.socket_fd), udp_to(other.udp_to),
#ifdef _WIN32
      serial_handle(other.serial_handle),
#else
      serial_fd(other.serial_fd),
#endif
      serial_writer(std::move(other.serial_writer)),
      udpin_listener(other.udpin_listener),
      udpin_running(other.udpin_running.load()),
      udpin_thread(std::move(other.udpin_thread)),
      has_udp_remote(other.has_udp_remote)
  {
    other.socket_fd = -1;
#ifdef _WIN32
    other.serial_handle = INVALID_HANDLE_VALUE;
#else
    other.serial_fd = -1;
#endif
  }
  
  // Move assignment
  MavlinkEndpoint& operator=(MavlinkEndpoint&& other) noexcept {
    if (this != &other) {
      type = other.type;
      socket_fd = other.socket_fd;
      udp_to = other.udp_to;
#ifdef _WIN32
      serial_handle = other.serial_handle;
      other.serial_handle = INVALID_HANDLE_VALUE;
#else
      serial_fd = other.serial_fd;
      other.serial_fd = -1;
#endif
      serial_writer = std::move(other.serial_writer);
      udpin_listener = other.udpin_listener;
      udpin_running.store(other.udpin_running.load());
      udpin_thread = std::move(other.udpin_thread);
      has_udp_remote = other.has_udp_remote;
      other.socket_fd = -1;
    }
    return *this;
  }
};

static bool send_bytes(MavlinkEndpoint& endpoint, const uint8_t* data, size_t len){
  switch (endpoint.type){
    case MavlinkTransport::UDP: {
      if (endpoint.socket_fd < 0) return false;
      // If this endpoint is a udpin listener, only send once we've discovered a remote
      if (endpoint.udpin_listener){
        std::lock_guard<std::mutex> lk(endpoint.udp_mtx);
        if (!endpoint.has_udp_remote) return false;
        int sent = (int)sendto(endpoint.socket_fd, (const char*)data, (int)len, 0,
                               (const sockaddr*)&endpoint.udp_to, sizeof(endpoint.udp_to));
        return sent == (int)len;
      } else {
        int sent = (int)sendto(endpoint.socket_fd, (const char*)data, (int)len, 0,
                               (const sockaddr*)&endpoint.udp_to, sizeof(endpoint.udp_to));
        return sent == (int)len;
      }
    }
    case MavlinkTransport::TCP: {
      if (endpoint.socket_fd < 0) return false;
      size_t total = 0;
      while (total < len){
        int s = (int)send(endpoint.socket_fd, (const char*)data + total, (int)(len - total), 0);
        if (s <= 0) return false;
        total += (size_t)s;
      }
      return true;
    }
    case MavlinkTransport::SERIAL: {
      if (!endpoint.serial_writer) return false;
      return endpoint.serial_writer->enqueue(data, len);
    }
  }
  return false;
}

static std::optional<MavlinkEndpoint> open_mavlink_udp_endpoint(const std::string& target_ip, uint16_t port){
  int fd = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0){ std::perror("socket"); return std::nullopt; }

  auto addr = parse_ipv4_addr(target_ip);
  if (!addr){
    std::cerr << "Invalid MAVLink UDP target IP: " << target_ip << "\n";
    CLOSESOCK(fd);
    return std::nullopt;
  }

  MavlinkEndpoint ep{};
  ep.type = MavlinkTransport::UDP;
  ep.socket_fd = fd;
  std::memset(&ep.udp_to, 0, sizeof(ep.udp_to));
  ep.udp_to.sin_family = AF_INET;
  ep.udp_to.sin_port = htons(port);
  ep.udp_to.sin_addr = *addr;
  return std::move(ep);
}

// Create a UDP socket bound to 'port' and operate in "udpin" mode: listen for incoming
// MAVLink packets (heartbeats) and learn the remote address to send responses to.
static std::optional<MavlinkEndpoint> open_mavlink_udpin_endpoint(const std::string& /*listen_ip*/, uint16_t port){
  int fd = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0){ std::perror("socket"); return std::nullopt; }

  // Allow reuse
  int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_res = bind(fd, (sockaddr*)&addr, sizeof(addr));
#ifdef _WIN32
  if (bind_res == SOCKET_ERROR){
#else
  if (bind_res < 0){
#endif
    int err = last_socket_error();
    std::cerr << "bind failed for udpin: " << format_socket_error(err) << "\n";
    CLOSESOCK(fd); return std::nullopt;
  }

  // Bigger recv buffer (optional)
  int rcvbuf = 1<<20;
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvbuf, sizeof(rcvbuf));

  MavlinkEndpoint ep{};
  ep.type = MavlinkTransport::UDP;
  ep.socket_fd = fd;
  ep.udpin_listener = true;
  ep.has_udp_remote = false;
  std::memset(&ep.udp_to, 0, sizeof(ep.udp_to));
  return std::move(ep);
}

static std::optional<MavlinkEndpoint> open_mavlink_tcp_endpoint(const std::string& target_ip, uint16_t port){
  int fd = (int)socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (fd < 0){ std::perror("socket"); return std::nullopt; }

  auto addr = parse_ipv4_addr(target_ip);
  if (!addr){
    std::cerr << "Invalid MAVLink TCP target IP: " << target_ip << "\n";
    CLOSESOCK(fd);
    return std::nullopt;
  }

  sockaddr_in to{};
  to.sin_family = AF_INET;
  to.sin_port = htons(port);
  to.sin_addr = *addr;

  int res = connect(fd, (sockaddr*)&to, sizeof(to));
#ifdef _WIN32
  if (res == SOCKET_ERROR){
#else
  if (res < 0){
#endif
    int err = last_socket_error();
    std::cerr << "TCP connect failed: " << format_socket_error(err) << "\n";
    CLOSESOCK(fd);
    return std::nullopt;
  }

  MavlinkEndpoint ep{};
  ep.type = MavlinkTransport::TCP;
  ep.socket_fd = fd;
  return std::move(ep);
}

static std::optional<MavlinkEndpoint> open_mavlink_serial_endpoint(const std::string& device, int baud){
#ifdef _WIN32
  std::string path = device;
  if (path.rfind("\\\\.\\", 0) != 0 && (path.rfind("COM", 0) == 0 || path.rfind("com", 0) == 0)){
    path = "\\\\.\\" + path;
  }

  HANDLE h = CreateFileA(path.c_str(), GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, 0, nullptr);
  if (h == INVALID_HANDLE_VALUE){
    int err = last_os_error();
    std::cerr << "Failed to open serial port " << device << ": " << format_socket_error(err) << "\n";
    return std::nullopt;
  }

  DCB dcb;
  std::memset(&dcb, 0, sizeof(dcb));
  dcb.DCBlength = sizeof(dcb);
  if (!GetCommState(h, &dcb)){
    int err = last_os_error();
    std::cerr << "GetCommState failed for " << device << ": " << format_socket_error(err) << "\n";
    CloseHandle(h);
    return std::nullopt;
  }

  dcb.BaudRate = baud;
  dcb.ByteSize = 8;
  dcb.Parity = NOPARITY;
  dcb.StopBits = ONESTOPBIT;
  dcb.fOutxCtsFlow = FALSE;
  dcb.fOutxDsrFlow = FALSE;
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fOutX = FALSE;
  dcb.fInX = FALSE;

  if (!SetCommState(h, &dcb)){
    int err = last_os_error();
    std::cerr << "SetCommState failed for " << device << ": " << format_socket_error(err) << "\n";
    CloseHandle(h);
    return std::nullopt;
  }

  COMMTIMEOUTS timeouts{};
  timeouts.ReadIntervalTimeout = 50;
  timeouts.ReadTotalTimeoutMultiplier = 10;
  timeouts.ReadTotalTimeoutConstant = 50;
  timeouts.WriteTotalTimeoutMultiplier = 10;
  timeouts.WriteTotalTimeoutConstant = 50;
  if (!SetCommTimeouts(h, &timeouts)){
    int err = last_os_error();
    std::cerr << "SetCommTimeouts failed for " << device << ": " << format_socket_error(err) << "\n";
    CloseHandle(h);
    return std::nullopt;
  }

  PurgeComm(h, PURGE_RXCLEAR | PURGE_TXCLEAR);

  MavlinkEndpoint ep{};
  ep.type = MavlinkTransport::SERIAL;
  ep.serial_handle = h;
  ep.serial_writer = std::make_unique<SerialAsyncWriter>();
  if (!ep.serial_writer->start(ep.serial_handle)){
    std::cerr << "Failed to start serial writer thread for " << device << "\n";
    ep.serial_writer.reset();
    CloseHandle(h);
    return std::nullopt;
  }
  return std::move(ep);
#else
  int fd = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0){
    std::cerr << "Failed to open serial port " << device << ": " << format_socket_error(errno) << "\n";
    return std::nullopt;
  }

  termios tty{};
  if (tcgetattr(fd, &tty) != 0){
    std::cerr << "tcgetattr failed for " << device << ": " << format_socket_error(errno) << "\n";
    close(fd);
    return std::nullopt;
  }

  cfmakeraw(&tty);
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1; // 100 ms write timeout

  auto spd = baud_to_speed(baud);
  if (!spd){
    std::cerr << "Unsupported baud rate " << baud << ", falling back to 57600.\n";
    spd = baud_to_speed(57600);
  }
  if (spd){
    cfsetispeed(&tty, *spd);
    cfsetospeed(&tty, *spd);
  }

  if (tcsetattr(fd, TCSANOW, &tty) != 0){
    std::cerr << "tcsetattr failed for " << device << ": " << format_socket_error(errno) << "\n";
    close(fd);
    return std::nullopt;
  }
  tcflush(fd, TCIOFLUSH);

  MavlinkEndpoint ep{};
  ep.type = MavlinkTransport::SERIAL;
  ep.serial_fd = fd;
  ep.serial_writer = std::make_unique<SerialAsyncWriter>();
  if (!ep.serial_writer->start(ep.serial_fd)){
    std::cerr << "Failed to start serial writer thread for " << device << "\n";
    ep.serial_writer.reset();
    close(fd);
    return std::nullopt;
  }
  return std::move(ep);
#endif
}

static void close_mavlink_endpoint(MavlinkEndpoint& endpoint){
  // If this was a udpin listener, stop the receive thread first by closing the socket
  if (endpoint.udpin_listener){
    endpoint.udpin_running.store(false);
    if (endpoint.socket_fd >= 0){
      CLOSESOCK(endpoint.socket_fd);
      endpoint.socket_fd = -1;
    }
    if (endpoint.udpin_thread.joinable()) endpoint.udpin_thread.join();
    endpoint.udpin_listener = false;
  }

  switch (endpoint.type){
    case MavlinkTransport::UDP:
    case MavlinkTransport::TCP:
      if (endpoint.socket_fd >= 0){
        CLOSESOCK(endpoint.socket_fd);
        endpoint.socket_fd = -1;
      }
      break;
    case MavlinkTransport::SERIAL:
      if (endpoint.serial_writer){
        endpoint.serial_writer->stop();
        endpoint.serial_writer.reset();
      }
#ifdef _WIN32
      if (endpoint.serial_handle != INVALID_HANDLE_VALUE){
        CloseHandle(endpoint.serial_handle);
        endpoint.serial_handle = INVALID_HANDLE_VALUE;
      }
#else
      if (endpoint.serial_fd >= 0){
        close(endpoint.serial_fd);
        endpoint.serial_fd = -1;
      }
#endif
      break;
  }
}


// ---------- networking ----------
static int open_psn_socket(const std::string& group, uint16_t port,
                           const std::optional<std::string>& iface_ip,
                           bool join_multicast){
#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2,2), &wsa) != 0){
    std::cerr << "WSAStartup failed\n"; return -1;
  }
#endif
  int fd = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0){ std::perror("socket"); return -1; }

  // Allow multiple listeners on the same port
  int reuse = 1;
  setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (const char*)&reuse, sizeof(reuse));

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(port);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  int bind_res = bind(fd, (sockaddr*)&addr, sizeof(addr));
#ifdef _WIN32
  if (bind_res == SOCKET_ERROR){
#else
  if (bind_res < 0){
#endif
    int err = last_socket_error();
    std::cerr << "bind failed: " << format_socket_error(err) << "\n";
#ifdef _WIN32
    if (err == WSAEACCES){
      std::cerr << "Hint: Windows reserves some UDP port ranges. Run `netsh interface ipv4 show excludedportrange protocol=udp` "
                   "to check, or pick a different port.\n";
    }
#endif
    CLOSESOCK(fd); return -1;
  }

  if (join_multicast){
    auto group_addr = parse_ipv4_addr(group);
    if (!group_addr){
      std::cerr << "Invalid multicast group address: " << group << "\n";
      CLOSESOCK(fd); return -1;
    }

    ip_mreq mreq{};
    mreq.imr_multiaddr = *group_addr;
    if (iface_ip){
      auto iface_addr = parse_ipv4_addr(*iface_ip);
      if (!iface_addr){
        std::cerr << "Invalid interface IP: " << *iface_ip << "\n";
        CLOSESOCK(fd); return -1;
      }
      mreq.imr_interface = *iface_addr;
    } else {
      mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    }

    if (setsockopt(fd, IPPROTO_IP, IP_ADD_MEMBERSHIP,
       (const char*)&mreq, sizeof(mreq)) < 0){
  int err = last_socket_error();
  std::cerr << "IP_ADD_MEMBERSHIP failed: " << format_socket_error(err) << "\n";
      CLOSESOCK(fd); return -1;
    }
  }

  // Bigger recv buffer (optional)
  int rcvbuf = 1<<20;
  setsockopt(fd, SOL_SOCKET, SO_RCVBUF, (const char*)&rcvbuf, sizeof(rcvbuf));
  return fd;
}

// ---------- MAVLink helpers ----------
static bool send_vision_position_estimate(MavlinkEndpoint& endpoint,
                                          uint8_t sysid, uint8_t compid,
                                          double stamp_s,
                                          float x_ned, float y_ned, float z_ned,
                                          std::optional<float> yaw_rad = std::nullopt)
{
  mavlink_message_t msg;
  uint8_t buf[512];

  float roll = 0.0f, pitch = 0.0f, yaw = yaw_rad.value_or(NAN);
  uint64_t usec = static_cast<uint64_t>(stamp_s * 1e6);

  // Covariance (row-major upper triangle of 6x6 pose covariance -> 21 elements).
  // ArduPilot GPS-denied mode requires proper covariance values, not NANs
  float cov[21];
  for (int i = 0; i < 21; ++i) cov[i] = 0.0f;
  
  // Set position variances (diagonal elements of upper triangle)
  cov[0]  = 0.01f; // var(x) - 0.1m std deviation
  cov[2]  = 0.01f; // var(y) - 0.1m std deviation  
  cov[5]  = 0.04f; // var(z) - 0.2m std deviation (altitude less certain)
  // Rotation variances (for roll, pitch, yaw)
  cov[9]  = 0.01f; // var(roll)
  cov[14] = 0.01f; // var(pitch)
  cov[20] = 0.04f; // var(yaw) - 0.2 rad std deviation

  uint8_t reset_counter = 0;

  mavlink_msg_vision_position_estimate_pack(
      sysid, compid, &msg,
      usec, x_ned, y_ned, z_ned,
      roll, pitch, yaw,
      cov, reset_counter);

  const uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  return send_bytes(endpoint, buf, len);
}


int main(int argc, char** argv){
  // Defaults
  std::string psn_group = psn::DEFAULT_GROUP;
  uint16_t    psn_port  = psn::DEFAULT_PORT;
  std::optional<std::string> iface_ip; // e.g. "192.168.0.10"

  std::string mav_ip = "127.0.0.1";
  uint16_t    mav_port = 14550;
  std::string mav_transport = "udp";
  int         serial_baud = 57600;
  uint8_t     sysid = 245;
  uint8_t     compid = 200;
  int         target_tracker = -1;
  bool        convert_to_ned = true;
  bool        force_unicast = false;

  for (int i=1; i<argc; ++i){
    std::string a = argv[i];
    auto next = [&](std::string def="")->std::string{ return (i+1<argc)? std::string(argv[++i]) : def; };
    if (a=="--group") psn_group = next(psn_group);
    else if (a=="--port") psn_port = (uint16_t)std::stoi(next());
    else if (a=="--iface") iface_ip = next();
    else if (a=="--mav-ip") mav_ip = next(mav_ip);
    else if (a=="--mav-port") mav_port = (uint16_t)std::stoi(next());
    else if (a=="--mav-transport") mav_transport = next(mav_transport);
    else if (a=="--mav-serial-baud" || a=="--serial-baud") serial_baud = std::stoi(next());
    else if (a=="--sysid") sysid = (uint8_t)std::stoi(next());
    else if (a=="--compid") compid = (uint8_t)std::stoi(next());
    else if (a=="--tracker") target_tracker = std::stoi(next());
    else if (a=="--no-ned") convert_to_ned = false;
    else if (a=="--unicast") force_unicast = true;
    else if (a=="-h" || a=="--help"){
      std::cout <<
        "Usage: psn_to_mavlink [--group 236.10.10.10] [--port 56565]\n"
        "                      [--iface 192.168.0.X] [--mav-ip 127.0.0.1] [--mav-port 14550]\n"
        "                      [--mav-transport udp|tcp|serial] [--mav-serial-baud 57600]\n"
        "                      [--sysid 245] [--compid 200] [--tracker <id>] [--no-ned] [--unicast]\n";
      return 0;
    }
  }

  if (serial_baud <= 0){
    std::cerr << "Serial baud rate must be positive\n";
    return 1;
  }

  std::string transport_lower = mav_transport;
  std::transform(transport_lower.begin(), transport_lower.end(), transport_lower.begin(),
                 [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

  bool use_tcp = false;
  bool use_serial = false;
  bool use_udpin = false;
  if (transport_lower == "udp"){
    use_tcp = false;
  } else if (transport_lower == "tcp"){
    use_tcp = true;
  } else if (transport_lower == "serial"){
    use_serial = true;
  } else if (transport_lower == "udpin"){
    use_udpin = true;
  } else {
    std::cerr << "Unknown MAVLink transport '" << mav_transport << "'. Use udp, udpin, tcp, or serial.\n";
    return 1;
  }

  bool auto_serial = false;
  if (!use_tcp && !use_serial && looks_like_serial_target(mav_ip)){
    use_serial = true;
    auto_serial = true;
  }

  if (auto_serial){
    std::cout << "Detected serial destination '" << mav_ip << "', using serial transport.\n";
  }

  bool want_multicast = !force_unicast && is_multicast_ipv4(psn_group);
  if (force_unicast){
    std::cout << "Unicast mode forced; skipping multicast join.\n";
  } else if (!want_multicast){
    std::cout << "Group " << psn_group << " is not in the multicast range; using unicast receive.\n";
  }

  int psn_fd = open_psn_socket(psn_group, psn_port, iface_ip, want_multicast);
  if (psn_fd < 0) return 1;

  std::optional<MavlinkEndpoint> mav_endpoint_opt;
  if (use_serial){
    mav_endpoint_opt = open_mavlink_serial_endpoint(mav_ip, serial_baud);
  } else if (use_tcp){
    mav_endpoint_opt = open_mavlink_tcp_endpoint(mav_ip, mav_port);
  } else if (use_udpin){
    mav_endpoint_opt = open_mavlink_udpin_endpoint(mav_ip, mav_port);
  } else {
    mav_endpoint_opt = open_mavlink_udp_endpoint(mav_ip, mav_port);
  }
  if (!mav_endpoint_opt) {
    CLOSESOCK(psn_fd);
#ifdef _WIN32
    WSACleanup();
#endif
    return 1;
  }
  MavlinkEndpoint mav_endpoint = std::move(*mav_endpoint_opt);

  // If udpin mode, start a receiver thread to learn the peer address (heartbeats)
  if (use_udpin){
    mav_endpoint.udpin_listener = true;
    mav_endpoint.udpin_running.store(true);
    mav_endpoint.udpin_thread = std::thread([&mav_endpoint]{
      uint8_t buf[2048];
      mavlink_message_t msg;
      mavlink_status_t status{};
      while (mav_endpoint.udpin_running.load()){
        sockaddr_in from{};
        socklen_t flen = sizeof(from);
        ssize_t n = recvfrom(mav_endpoint.socket_fd, (char*)buf, (int)sizeof(buf), 0,
                             (sockaddr*)&from, &flen);
        if (n <= 0){ std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        for (ssize_t i = 0; i < n; ++i){
          if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)){
            if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT){
              std::lock_guard<std::mutex> lk(mav_endpoint.udp_mtx);
              mav_endpoint.udp_to = from;
              mav_endpoint.has_udp_remote = true;
              const char* ip = inet_ntoa(from.sin_addr);
              std::cout << "Discovered MAVLink UDP peer " << ip << ":" << ntohs(from.sin_port) << "\n";
            }
          }
        }
      }
    });
  }

  std::cout << "PSN listen " << psn_group << ":" << psn_port << " -> MAVLink ";
  switch (mav_endpoint.type){
    case MavlinkTransport::UDP:
      if (mav_endpoint.udpin_listener){
        std::cout << "(udpin/listen) :" << mav_port;
      } else {
        std::cout << "(udp) " << mav_ip << ":" << mav_port;
      }
      break;
    case MavlinkTransport::TCP:
      std::cout << "(tcp) " << mav_ip << ":" << mav_port;
      break;
    case MavlinkTransport::SERIAL:
      std::cout << "(serial) " << mav_ip << " @" << serial_baud << " baud";
      break;
  }
  std::cout << '\n';

  std::vector<uint8_t> rx(2048);
  while (true){
    ssize_t n = recv(psn_fd, (char*)rx.data(), (int)rx.size(), 0);
    if (n <= 0) continue;

    std::map<uint16_t, TrackerPose> poses;
    if (!parsePSNPacket(rx.data(), (size_t)n, poses)) continue;

    using clk = std::chrono::steady_clock;
    double t_s = std::chrono::duration<double>(clk::now().time_since_epoch()).count();

    for (const auto& kv : poses){
      uint16_t id = kv.first;
      if (target_tracker >= 0 && id != (uint16_t)target_tracker) continue;
      const TrackerPose& tp = kv.second;
      if (!tp.has_pos) continue;

      float x = tp.x, y = tp.y, z = tp.z;
      float xn=x, yn=y, zn=z;
      if (convert_to_ned){ xn = x; yn = y; zn = -z; } // Z up -> Z down

      if (!send_vision_position_estimate(mav_endpoint, sysid, compid, t_s, xn, yn, zn)){
        int err = (mav_endpoint.type == MavlinkTransport::SERIAL)
                    ? last_os_error()
                    : last_socket_error();
        std::cerr << "Failed to send MAVLink vision_position_estimate: "
                  << format_socket_error(err) << "\n";
        if (mav_endpoint.type != MavlinkTransport::UDP){
          CLOSESOCK(psn_fd);
          close_mavlink_endpoint(mav_endpoint);
#ifdef _WIN32
          WSACleanup();
#endif
          return 1;
        }
      }

      // Uncomment for debugging (slows down significantly):
      // std::cout << "Tracker " << id << " PSN:"
      //       << x << "," << y << "," << z
      //       << "  NED:" << xn << "," << yn << "," << zn << '\n';
    }
  }

  CLOSESOCK(psn_fd);
  close_mavlink_endpoint(mav_endpoint);
#ifdef _WIN32
  WSACleanup();
#endif
  return 0;
}
