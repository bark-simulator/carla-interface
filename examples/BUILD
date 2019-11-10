py_binary(
  name = "test",
  srcs = ["test.py"],
  data = [
            "@com_github_bark_simulator_bark//python:bark.so",
            "@com_github_bark_simulator_bark//modules/runtime/tests:xodr_data",
            "@com_github_bark_simulator_bark//modules/commons/params:params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
          ],
  deps = [
            "@com_github_bark_simulator_bark//modules/runtime/commons:xodr_parser",
          ],
)