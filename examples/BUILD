py_binary(
  name = "test",
  srcs = ["test.py"],
  data = [
            "@com_github_bark-simulator_bark//python:bark.so",
            "@com_github_bark-simulator_bark//modules/runtime/tests:xodr_data",
            "@com_github_bark-simulator_bark//examples:params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
          ],
  imports = ["@com_github_bark-simulator_bark//python/"],
  deps = [
            "@com_github_bark-simulator_bark//modules/runtime/commons:parameters",
            "@com_github_bark-simulator_bark//modules/runtime/commons:xodr_parser",
            "@com_github_bark-simulator_bark//modules/runtime/viewer:pygame_viewer",
            "@com_github_bark-simulator_bark//modules/runtime/viewer:matplotlib_viewer",
            "//client:carla_client",
          ],
)