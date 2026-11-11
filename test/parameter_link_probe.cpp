#include <holydrone_autotune/parameter_link.hpp>
#include <iostream>

// Intentionally never spin the default callback queue: parameter replies must be independent.
// Read-only probe, also usable against a ground MAVROS instance. No setpoints or writes.
int main(int argc, char** argv) {
  ros::init(argc, argv, "parameter_link_probe");
  ros::NodeHandle nh;
  ros::NodeHandle priv("~");
  std::string name; priv.param<std::string>("name", name, "MC_PITCHRATE_D");
  brct::ParameterLink link; link.setup(nh, "/mavros");
  try {
    for (int i=0; i<5; ++i) {
      const auto started = ros::WallTime::now();
      const double value = link.exchange(name);
      std::cout << name << "=" << value << " elapsed_ms="
                << (ros::WallTime::now()-started).toSec()*1000 << std::endl;
    }
  } catch (const std::exception& e) { std::cerr << e.what() << std::endl; return 1; }
  return 0;
}
