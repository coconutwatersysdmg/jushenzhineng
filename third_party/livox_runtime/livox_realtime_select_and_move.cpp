// Livox SDK2 realtime point receive -> pick/recognize point -> world coordinate -> optional PLC move.
// Copy this file into Livox-SDK2/samples/livox_realtime_select_and_move/ or build it with Livox SDK2.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

struct Point {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  uint8_t reflectivity = 0;
};

struct Roi {
  double xmin = -std::numeric_limits<double>::infinity();
  double xmax = std::numeric_limits<double>::infinity();
  double ymin = -std::numeric_limits<double>::infinity();
  double ymax = std::numeric_limits<double>::infinity();
  double zmin = -std::numeric_limits<double>::infinity();
  double zmax = std::numeric_limits<double>::infinity();

  bool contains(const Point& p) const {
    return xmin <= p.x && p.x <= xmax && ymin <= p.y && p.y <= ymax && zmin <= p.z && p.z <= zmax;
  }
};

struct Command {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
};

static std::mutex g_points_mutex;
static std::vector<Point> g_points;
static std::atomic<bool> g_running{true};
static size_t g_max_buffered_points = 600000;

static const double R[3][3] = {
    {-0.009681797288, -0.026598874563, -0.999599301057},
    {0.004599733696, 0.999634393434, -0.026644359896},
    {0.999942550975, -0.004855855879, -0.009555909820},
};

static const double T[3] = {617.848664784362, 958.313720699084, 322.667147954541};

std::array<double, 3> LidarToWorld(const Point& p) {
  const double lidar[3] = {p.x, p.y, p.z};
  std::array<double, 3> world{};
  for (int row = 0; row < 3; ++row) {
    world[row] = T[row];
    for (int col = 0; col < 3; ++col) {
      world[row] += R[row][col] * lidar[col];
    }
  }
  return world;
}

bool ValidateXySoftLimits(const Command& cmd, std::string* reason) {
  auto check = [&](const char* axis, double value, double lower, double upper) {
    if (lower <= value && value <= upper) {
      return true;
    }
    std::ostringstream out;
    out << axis << "=" << std::fixed << std::setprecision(2) << value
        << " out of soft limit " << lower << "-" << upper;
    *reason = out.str();
    return false;
  };
  return check("X", cmd.x, 0.0, 500.0) && check("Y", cmd.y, 0.0, 1300.0);
}

void PushPoint(const Point& point) {
  std::lock_guard<std::mutex> lock(g_points_mutex);
  g_points.push_back(point);
  if (g_points.size() > g_max_buffered_points) {
    const size_t drop = g_points.size() - g_max_buffered_points;
    g_points.erase(g_points.begin(), g_points.begin() + static_cast<std::ptrdiff_t>(drop));
  }
}

void PointCloudCallback(uint32_t handle, const uint8_t dev_type, LivoxLidarEthernetPacket* data, void* client_data) {
  (void)handle;
  (void)dev_type;
  (void)client_data;
  if (data == nullptr || data->data == nullptr || data->dot_num == 0) {
    return;
  }

  if (data->data_type != kLivoxLidarCartesianCoordinateHighData) {
    return;
  }

  const LivoxLidarCartesianHighRawPoint* points =
      reinterpret_cast<const LivoxLidarCartesianHighRawPoint*>(data->data);
  for (uint32_t i = 0; i < data->dot_num; ++i) {
    Point p;
    p.x = static_cast<double>(points[i].x);
    p.y = static_cast<double>(points[i].y);
    p.z = static_cast<double>(points[i].z);
    p.reflectivity = points[i].reflectivity;
    PushPoint(p);
  }
}

std::vector<Point> Snapshot(const Roi& roi) {
  std::vector<Point> snapshot;
  {
    std::lock_guard<std::mutex> lock(g_points_mutex);
    snapshot = g_points;
  }
  snapshot.erase(
      std::remove_if(snapshot.begin(), snapshot.end(), [&](const Point& p) { return !roi.contains(p); }),
      snapshot.end());
  return snapshot;
}

std::vector<Point> SnapshotAll() {
  std::lock_guard<std::mutex> lock(g_points_mutex);
  return g_points;
}

void ClearBufferedPoints() {
  std::lock_guard<std::mutex> lock(g_points_mutex);
  g_points.clear();
}

bool WritePcd(const std::string& path, const std::vector<Point>& points, std::string* reason) {
  std::ofstream file(path.c_str(), std::ios::out | std::ios::trunc);
  if (!file.is_open()) {
    *reason = "cannot open output file";
    return false;
  }

  file << "# .PCD v0.7 - Point Cloud Data file format\n";
  file << "VERSION 0.7\n";
  file << "FIELDS x y z\n";
  file << "SIZE 4 4 4\n";
  file << "TYPE F F F\n";
  file << "COUNT 1 1 1\n";
  file << "WIDTH " << points.size() << "\n";
  file << "HEIGHT 1\n";
  file << "VIEWPOINT 0 0 0 1 0 0 0\n";
  file << "POINTS " << points.size() << "\n";
  file << "DATA ascii\n";
  file << std::fixed << std::setprecision(3);
  for (const auto& p : points) {
    file << p.x << " " << p.y << " " << p.z << "\n";
  }
  return true;
}

bool PickCentroid(const std::vector<Point>& points, Point* selected) {
  if (points.empty()) {
    return false;
  }
  double sx = 0.0;
  double sy = 0.0;
  double sz = 0.0;
  for (const auto& p : points) {
    sx += p.x;
    sy += p.y;
    sz += p.z;
  }
  selected->x = sx / points.size();
  selected->y = sy / points.size();
  selected->z = sz / points.size();
  selected->reflectivity = 0;
  return true;
}

std::string Quote(const std::string& text) {
  return "\"" + text + "\"";
}

int RunMoveScript(const std::string& python,
                  const std::string& script,
                  const std::string& plc_app_dir,
                  const Command& command,
                  bool execute) {
  std::ostringstream cmd;
  cmd << Quote(python) << " " << Quote(script)
      << " --world-point " << std::fixed << std::setprecision(3) << command.x << " " << command.y << " "
      << command.z << " --z-offset 0 --xy-only";
  if (!plc_app_dir.empty()) {
    cmd << " --plc-app-dir " << Quote(plc_app_dir);
  }
  if (execute) {
    cmd << " --execute --yes";
  }
  return std::system(cmd.str().c_str());
}

void PrintHelp() {
  std::cout << "\nCommands:\n"
            << "  status                         show buffered point count\n"
            << "  capture output.pcd [ms]        clear buffer, capture ms, save PCD\n"
            << "  roi xmin xmax ymin ymax zmin zmax   set ROI in lidar coordinate, mm\n"
            << "  pick                           pick centroid point from latest ROI\n"
            << "  manual x y z                   use manual lidar point, mm\n"
            << "  dry                            calculate XY target and prepare confirmation\n"
            << "  yes                            execute the last dry XY target\n"
            << "  quit                           exit\n\n";
}

int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "Usage:\n"
      << "  livox_realtime_select_and_move.exe livox_lidar_config.json "
                 "[plc_app_dir] [python_exe] [--capture-pcd output.pcd] [--capture-ms 3000] "
                 "[--max-points 600000]\n\n";
    std::cout << "Example:\n"
              << "  livox_realtime_select_and_move.exe ..\\..\\livox_lidar_config.json "
                 "D:\\research_code\\plc_finished_app python\n";
    return 1;
  }

  const std::string sdk_config_path = argv[1];
  std::string plc_app_dir;
  std::string python = "python";
  bool python_set = false;
  std::string capture_pcd_path;
  int capture_ms = 3000;
  for (int i = 2; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--capture-pcd") {
      if (i + 1 >= argc) {
        std::cerr << "--capture-pcd requires output path.\n";
        return 1;
      }
      capture_pcd_path = argv[++i];
    } else if (arg == "--capture-ms") {
      if (i + 1 >= argc) {
        std::cerr << "--capture-ms requires milliseconds.\n";
        return 1;
      }
      capture_ms = std::atoi(argv[++i]);
      if (capture_ms <= 0) {
        std::cerr << "--capture-ms must be positive.\n";
        return 1;
      }
    } else if (arg == "--max-points") {
      if (i + 1 >= argc) {
        std::cerr << "--max-points requires point count.\n";
        return 1;
      }
      const long long parsed_max_points = std::atoll(argv[++i]);
      if (parsed_max_points <= 0) {
        std::cerr << "--max-points must be positive.\n";
        return 1;
      }
      g_max_buffered_points = static_cast<size_t>(parsed_max_points);
    } else if (arg.compare(0, 2, "--") == 0) {
      std::cerr << "Unknown option: " << arg << "\n";
      return 1;
    } else if (plc_app_dir.empty()) {
      plc_app_dir = arg;
    } else if (!python_set) {
      python = arg;
      python_set = true;
    } else {
      std::cerr << "Unexpected argument: " << arg << "\n";
      return 1;
    }
  }
  const std::string move_script = "plc_move_xyz_once.py";

  SetLivoxLidarPointCloudCallBack(PointCloudCallback, nullptr);
  if (!LivoxLidarSdkInit(sdk_config_path.c_str())) {
    std::cerr << "LivoxLidarSdkInit failed. Check config path and lidar network.\n";
    return 2;
  }

  if (!capture_pcd_path.empty()) {
    std::cout << "Capturing fresh point cloud for " << capture_ms << " ms...\n";
    std::cout << "Max buffered points: " << g_max_buffered_points << "\n";
    ClearBufferedPoints();
    std::this_thread::sleep_for(std::chrono::milliseconds(capture_ms));
    auto cloud = SnapshotAll();
    std::string reason;
    if (!WritePcd(capture_pcd_path, cloud, &reason)) {
      std::cerr << "Write PCD failed: " << reason << "\n";
      LivoxLidarSdkUninit();
      return 3;
    }
    std::cout << "Saved PCD: " << capture_pcd_path << "\n";
    std::cout << "Captured points: " << cloud.size() << "\n";
    LivoxLidarSdkUninit();
    return 0;
  }

  Roi roi;
  Point selected;
  bool has_selected = false;
  Command pending_command;
  bool has_pending_command = false;
  double x_offset = 0.0;
  double y_offset = 0.0;

  std::cout << "Livox realtime receive started.\n";
  std::cout << "Use 'roi' then 'pick'. Type 'help' for commands.\n";

  std::string line;
  while (g_running.load()) {
    std::cout << "> ";
    if (!std::getline(std::cin, line)) {
      break;
    }
    std::istringstream in(line);
    std::string op;
    in >> op;

    if (op == "help") {
      PrintHelp();
    } else if (op == "quit" || op == "q") {
      break;
    } else if (op == "status") {
      std::lock_guard<std::mutex> lock(g_points_mutex);
      std::cout << "buffered points: " << g_points.size() << "\n";
    } else if (op == "capture") {
      std::string output_path;
      int ms = 1000;
      if (!(in >> output_path)) {
        std::cout << "Bad capture. Example: capture current_scan.pcd 1000\n";
        continue;
      }
      if (in >> ms && ms <= 0) {
        std::cout << "Capture milliseconds must be positive.\n";
        continue;
      }
      std::cout << "Capturing fresh point cloud for " << ms << " ms...\n";
      ClearBufferedPoints();
      std::this_thread::sleep_for(std::chrono::milliseconds(ms));
      auto cloud = SnapshotAll();
      std::string reason;
      if (!WritePcd(output_path, cloud, &reason)) {
        std::cout << "Write PCD failed: " << reason << "\n";
        continue;
      }
      has_pending_command = false;
      std::cout << "Saved PCD: " << output_path << "\n";
      std::cout << "Captured points: " << cloud.size() << "\n";
    } else if (op == "roi") {
      Roi new_roi;
      if (!(in >> new_roi.xmin >> new_roi.xmax >> new_roi.ymin >> new_roi.ymax >> new_roi.zmin >> new_roi.zmax)) {
        std::cout << "Bad roi. Example: roi -500 500 -500 500 300 2000\n";
        continue;
      }
      roi = new_roi;
      has_pending_command = false;
      std::cout << "ROI set.\n";
    } else if (op == "manual") {
      if (!(in >> selected.x >> selected.y >> selected.z)) {
        std::cout << "Bad point. Example: manual 120 30 900\n";
        continue;
      }
      has_selected = true;
      has_pending_command = false;
      std::cout << "selected lidar point: " << selected.x << ", " << selected.y << ", " << selected.z << "\n";
    } else if (op == "pick") {
      auto cloud = Snapshot(roi);
      if (!PickCentroid(cloud, &selected)) {
        std::cout << "No points in ROI. Adjust ROI or check lidar data.\n";
        continue;
      }
      has_selected = true;
      has_pending_command = false;
      std::cout << "ROI points: " << cloud.size() << "\n";
      std::cout << "selected lidar centroid: " << std::fixed << std::setprecision(2) << selected.x << ", "
                << selected.y << ", " << selected.z << "\n";
    } else if (op == "dry") {
      if (!has_selected) {
        std::cout << "No selected point. Use pick or manual first.\n";
        continue;
      }
      auto world = LidarToWorld(selected);
      Command cmd{world[0] + x_offset, world[1] + y_offset, world[2]};
      std::cout << "world point: X=" << world[0] << " Y=" << world[1] << " Z=" << world[2] << "\n";
      std::cout << "XY move target: X=" << cmd.x << " Y=" << cmd.y << "\n";
      std::cout << "Z display only, not sent to PLC: Z=" << world[2] << "\n";
      std::string reason;
      if (!ValidateXySoftLimits(cmd, &reason)) {
        std::cout << "Blocked: " << reason << "\n";
        has_pending_command = false;
        continue;
      }
      RunMoveScript(python, move_script, plc_app_dir, cmd, false);
      pending_command = cmd;
      has_pending_command = true;
      std::cout << "Dry OK. Type yes to execute XY motion. Any other command will not move.\n";
    } else if (op == "yes") {
      if (!has_pending_command) {
        std::cout << "No pending dry target. Use dry first.\n";
        continue;
      }
      std::string reason;
      if (!ValidateXySoftLimits(pending_command, &reason)) {
        std::cout << "Blocked: " << reason << "\n";
        has_pending_command = false;
        continue;
      }
      RunMoveScript(python, move_script, plc_app_dir, pending_command, true);
      has_pending_command = false;
    } else if (op == "move") {
      std::cout << "The move command is disabled. Use dry, then type yes to execute XY only.\n";
    } else if (op.empty()) {
      continue;
    } else {
      std::cout << "Unknown command. Type help.\n";
    }
  }

  g_running.store(false);
  LivoxLidarSdkUninit();
  return 0;
}
