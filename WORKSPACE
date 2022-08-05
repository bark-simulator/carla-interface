workspace(name = "carla_interface")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive", "http_file")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

git_repository(
  name = "bark_project",
  branch = "carla_adaptation",
  remote = "https://github.com/bark-simulator/bark",
#   commit = "0ab20e65aa6f328d61f7aab56e407ebdb4c7857e",
)
# local_repository(
#   name = "bark_project",
#   path = "/home/xliu/fortiss_bark/bark",
# )
load("@bark_project//tools:deps.bzl", "bark_dependencies")
bark_dependencies()

load("@com_github_nelhage_rules_boost//:boost/boost.bzl", "boost_deps")
boost_deps()

load("@pybind11_bazel//:python_configure.bzl", "python_configure")
python_configure(name = "local_config_python") 

git_repository(
    name = "com_github_nelhage_rules_boost",
    commit = "8a084196b14a396b6d4ff7c928ffbb6621f0d32c",
    remote = "https://github.com/patrickhart/rules_boost",
)


http_archive(
    name = "com_google_ceres_solver", 
    strip_prefix = "ceres-solver-1.14.0", 
    sha256 = "1296330fcf1e09e6c2f926301916f64d4a4c5c0ff12d460a9bc5d4c48411518f",
    build_file = "@bark_project//tools/ceres:ceres.BUILD",
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
new_local_repository(
    name = "carla",
    path = "/opt/carla-simulator",
    build_file_content = """
filegroup(
    name="carla_python_lib",
    srcs= select({
        "@bazel_tools//src/conditions:linux_x86_64": glob(["PythonAPI/carla/dist/carla-*-py3.7-linux-x86_64.egg"])
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

