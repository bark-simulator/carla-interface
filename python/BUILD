package(default_visibility = ["//visibility:public"])

cc_binary(
  name = "carla-interface.so",
  srcs = glob([
    "*.cpp", "*.hpp",
    "mpc/*.cpp", "mpc/*.hpp"]),
  deps = [
    "@python_linux//:python-lib",
    "@pybind11//:pybind11",
    "@com_github_eigen_eigen//:eigen", 
    "//cosimulation_modules/mpc:mpc",
  ],
  linkshared = 1,
  linkstatic = 1
)