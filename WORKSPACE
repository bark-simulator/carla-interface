workspace(name = "carla_interface")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
  name = "com_github_bark_simulator_bark",
  remote = "https://github.com/tin1254/bark",
  commit = "29c826561e007a9f18ccedc21a60d034c9f27427",
)

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "8a084196b14a396b6d4ff7c928ffbb6621f0d32c",
    remote = "https://github.com/patrickhart/rules_boost",
)

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()


http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.7.0.zip",
    sha256 = "b58cb7547a28b2c718d1e38aee18a3659c9e3ff52440297e965f5edffe34b6d0",
    build_file = "@com_github_bark_simulator_bark//tools:gtest.BUILD",
    strip_prefix = "googletest-release-1.7.0",
)


http_archive(
    name = "com_google_protobuf",
    sha256 = "cef7f1b5a7c5fba672bec2a319246e8feba471f04dcebfe362d55930ee7c1c30",
    strip_prefix = "protobuf-3.5.0",
    urls = ["https://github.com/google/protobuf/archive/v3.5.0.zip"],
)

http_archive(
    name = "pybind11",
    strip_prefix = "pybind11-2.3.0",
    urls = ["https://github.com/pybind/pybind11/archive/v2.3.0.zip"],
    build_file = "@com_github_bark_simulator_bark//tools/pybind11:pybind.BUILD",
)

# External dependency: Eigen; has no Bazel build.
http_archive(
    name = "com_github_eigen_eigen",
    build_file = "@com_github_bark_simulator_bark//tools/eigen:eigen.BUILD",
    sha256 = "dd254beb0bafc695d0f62ae1a222ff85b52dbaa3a16f76e781dce22d0d20a4a6",
    strip_prefix = "eigen-eigen-5a0156e40feb",
    urls = [
        "http://bitbucket.org/eigen/eigen/get/3.3.4.tar.bz2",
    ],
)

http_archive(
    name = "com_google_ceres_solver", 
    strip_prefix = "ceres-solver-1.14.0", 
    sha256 = "1296330fcf1e09e6c2f926301916f64d4a4c5c0ff12d460a9bc5d4c48411518f",
    build_file = "@com_github_bark_simulator_bark//tools/ceres:ceres.BUILD",
    urls = ["https://github.com/ceres-solver/ceres-solver/archive/1.14.0.tar.gz"],
)

# External dependency: Google Log; has Bazel build already.
http_archive(
    name = "com_github_google_glog",
    sha256 = "7083af285bed3995b5dc2c982f7de39bced9f0e6fd78d631f3285490922a0c3d",
    strip_prefix = "glog-3106945d8d3322e5cbd5658d482c9ffed2d892c0",
    urls = [
        "https://github.com/drigz/glog/archive/3106945d8d3322e5cbd5658d482c9ffed2d892c0.tar.gz",
    ],
)

# External dependency: Google Flags; has Bazel build already.
http_archive(
    name = "com_github_gflags_gflags",
    sha256 = "6e16c8bc91b1310a44f3965e616383dbda48f83e8c1eaa2370a215057b00cabe",
    strip_prefix = "gflags-77592648e3f3be87d6c7123eb81cbad75f9aef5a",
    urls = [
        "https://mirror.bazel.build/github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
        "https://github.com/gflags/gflags/archive/77592648e3f3be87d6c7123eb81cbad75f9aef5a.tar.gz",
    ],
)

new_local_repository(
    name = "python_linux",
    path = "./python/venv/",
    build_file_content = """
cc_library(
    name = "python-lib",
    srcs = glob(["lib/libpython3.*", "libs/python3.lib", "libs/python36.lib"]),
    hdrs = glob(["include/**/*.h", "include/*.h"]),
    includes = ["include/python3.6m", "include", "include/python3.7m", "include/python3.5m"], 
    visibility = ["//visibility:public"],
)
    """
)

http_archive(
  name = "carla",
  url= "http://carla-assets-internal.s3.amazonaws.com/Releases/Linux/CARLA_0.9.6.tar.gz",
  sha256 = "ddccb35682e8387f4f413f69d1577c1cc012e0e1474e69e20d0daf7f826a673f",
  build_file_content = """
filegroup(
    name="carla_python_lib",
    srcs= select({
        "@bazel_tools//src/conditions:linux_x86_64": glob(["PythonAPI/carla/dist/carla-*-py3.5-linux-x86_64.egg"]),
        "@bazel_tools//src/conditions:windows": glob(["PythonAPI/carla/dist/carla-*-py3.5-win-amd64.egg"])
    }),
    visibility = ["//visibility:public"],
)
filegroup(
    name="carla_server",
    srcs= ["CarlaUE4.sh"],
    visibility = ["//visibility:public"],
)
  """
)

