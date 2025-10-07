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

#ifdef _WIN32
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  using socklen_t = int;
  using ssize_t   = long long;
  #define CLOSESOCK closesocket
#else
  #include <arpa/inet.h>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <unistd.h>
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

struct MavlinkEndpoint {
  int fd{-1};
  bool is_udp{true};
  sockaddr_in udp_to{};
};

static bool send_bytes(const MavlinkEndpoint& endpoint, const uint8_t* data, size_t len){
  if (endpoint.fd < 0) return false;
  if (endpoint.is_udp){
    int sent = (int)sendto(endpoint.fd, (const char*)data, (int)len, 0,
                            (const sockaddr*)&endpoint.udp_to, sizeof(endpoint.udp_to));
    return sent == (int)len;
  } else {
    size_t total = 0;
    while (total < len){
      int s = (int)send(endpoint.fd, (const char*)data + total, (int)(len - total), 0);
      if (s <= 0) return false;
      total += (size_t)s;
    }
    return true;
  }
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
  ep.fd = fd;
  ep.is_udp = true;
  std::memset(&ep.udp_to, 0, sizeof(ep.udp_to));
  ep.udp_to.sin_family = AF_INET;
  ep.udp_to.sin_port = htons(port);
  ep.udp_to.sin_addr = *addr;
  return ep;
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
  ep.fd = fd;
  ep.is_udp = false;
  return ep;
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
static bool send_vision_position_estimate(const MavlinkEndpoint& endpoint,
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
  // If unknown, use NANs. If you prefer reasonable defaults, set e.g. 0.04f (0.2 m std^2) for pos.
  float cov[21];
  for (int i = 0; i < 21; ++i) cov[i] = NAN;

  // Example (optional): give modest variances for position and yaw
  // cov[0]  = 0.04f; // var(x)
  // cov[2]  = 0.04f; // var(y)
  // cov[5]  = 0.04f; // var(z)
  // cov[20] = 0.01f; // var(yaw) ~ (≈ 0.1 rad std)^2

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
    else if (a=="--sysid") sysid = (uint8_t)std::stoi(next());
    else if (a=="--compid") compid = (uint8_t)std::stoi(next());
    else if (a=="--tracker") target_tracker = std::stoi(next());
    else if (a=="--no-ned") convert_to_ned = false;
    else if (a=="--unicast") force_unicast = true;
    else if (a=="-h" || a=="--help"){
      std::cout <<
        "Usage: psn_to_mavlink [--group 236.10.10.10] [--port 56565]\n"
        "                      [--iface 192.168.0.X] [--mav-ip 127.0.0.1] [--mav-port 14550]\n"
        "                      [--mav-transport udp] [--sysid 245] [--compid 200] [--tracker <id>] [--no-ned]\n"
        "                      [--unicast]\n";
      return 0;
    }
  }

  std::string transport_lower = mav_transport;
  std::transform(transport_lower.begin(), transport_lower.end(), transport_lower.begin(),
                 [](unsigned char c){ return static_cast<char>(std::tolower(c)); });

  bool use_tcp = false;
  if (transport_lower == "udp"){
    use_tcp = false;
  } else if (transport_lower == "tcp"){
    use_tcp = true;
  } else {
    std::cerr << "Unknown MAVLink transport '" << mav_transport << "'. Use udp or tcp." << std::endl;
    return 1;
  }

  bool want_multicast = !force_unicast && is_multicast_ipv4(psn_group);
  if (force_unicast){
    std::cout << "Unicast mode forced; skipping multicast join." << std::endl;
  } else if (!want_multicast){
    std::cout << "Group " << psn_group << " is not in the multicast range; using unicast receive." << std::endl;
  }

  int psn_fd = open_psn_socket(psn_group, psn_port, iface_ip, want_multicast);
  if (psn_fd < 0) return 1;

  std::optional<MavlinkEndpoint> mav_endpoint_opt = use_tcp
    ? open_mavlink_tcp_endpoint(mav_ip, mav_port)
    : open_mavlink_udp_endpoint(mav_ip, mav_port);
  if (!mav_endpoint_opt) { 
  CLOSESOCK(psn_fd);
#ifdef _WIN32
  WSACleanup();
#endif
  return 1; 
  }
  MavlinkEndpoint mav_endpoint = *mav_endpoint_opt;

  std::cout << "PSN listen " << psn_group << ":" << psn_port
            << " -> MAVLink (" << (use_tcp ? "tcp" : "udp") << ") "
            << mav_ip << ":" << mav_port << std::endl;

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
        int err = last_socket_error();
        std::cerr << "Failed to send MAVLink vision_position_estimate: "
                  << format_socket_error(err) << "\n";
        if (!mav_endpoint.is_udp){
          CLOSESOCK(psn_fd);
          CLOSESOCK(mav_endpoint.fd);
#ifdef _WIN32
          WSACleanup();
#endif
          return 1;
        }
      }

      std::cout << "Tracker " << id << " PSN:"
                << x << "," << y << "," << z
                << "  NED:" << xn << "," << yn << "," << zn << std::endl;
    }
  }

  CLOSESOCK(psn_fd);
  CLOSESOCK(mav_endpoint.fd);
#ifdef _WIN32
  WSACleanup();
#endif
  return 0;
}
