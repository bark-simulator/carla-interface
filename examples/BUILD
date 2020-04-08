py_binary(
  name = "fill_world_with_npc",
  srcs = ["fill_world_with_npc.py"],
  data = [
            "@com_github_bark_simulator_bark//python:bark.so",
            "@com_github_bark_simulator_bark//modules/runtime/tests:xodr_data",
            "@com_github_bark_simulator_bark//examples:params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
          ],
  imports = ["@com_github_bark_simulator_bark//python/"],
  deps = [
            "@com_github_bark_simulator_bark//modules/runtime/commons:parameters",
            "@com_github_bark_simulator_bark//modules/runtime/commons:xodr_parser",
            "@com_github_bark_simulator_bark//modules/runtime/viewer:pygame_viewer",
            "//cosimulation_modules/client:carla_client",
            "//cosimulation_modules/client:sensors",
            "//cosimulation_modules/client:viewer",
          ],
)

py_binary(
  name = "fill_world_with_bark_ego",
  srcs = ["fill_world_with_bark_ego.py"],
  data = [
            "@com_github_bark_simulator_bark//python:bark.so",
            "@com_github_bark_simulator_bark//modules/runtime/tests:xodr_data",
            "@com_github_bark_simulator_bark//examples:params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
            "//python:carla-interface.so",
          ],
  imports = ["@com_github_bark_simulator_bark//python/"],
  deps = [
            "@com_github_bark_simulator_bark//modules/runtime/commons:parameters",
            "@com_github_bark_simulator_bark//modules/runtime/commons:xodr_parser",
            "@com_github_bark_simulator_bark//modules/runtime/viewer:pygame_viewer",
            "//cosimulation_modules/client:carla_client",
            "//cosimulation_modules/client:controller",
            "//cosimulation_modules/client:sensors",
            "//cosimulation_modules/client:viewer"
          ],
)

py_binary(
  name = "simulate_on_city_highway_straight",
  srcs = ["simulate_on_city_highway_straight.py"],
  data = [
            "@com_github_bark_simulator_bark//python:bark.so",
            "@com_github_bark_simulator_bark//modules/runtime/tests:xodr_data",
            "@com_github_bark_simulator_bark//examples:params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
            "//python:carla-interface.so",
            "//cosimulation_modules/maps:maps"
          ],
  imports = ["@com_github_bark_simulator_bark//python/"],
  deps = [
            "@com_github_bark_simulator_bark//modules/runtime/commons:parameters",
            "@com_github_bark_simulator_bark//modules/runtime/commons:xodr_parser",
            "@com_github_bark_simulator_bark//modules/runtime/viewer:pygame_viewer",
            "//cosimulation_modules/client:carla_client",
            "//cosimulation_modules/client:controller",
            "//cosimulation_modules/client:sensors",
            "//cosimulation_modules/client:viewer"
          ],
)

py_binary(
  name = "simulate_on_Crossing8Course",
  srcs = ["simulate_on_Crossing8Course.py"],
  data = [
            "@com_github_bark_simulator_bark//python:bark.so",
            "@com_github_bark_simulator_bark//modules/runtime/tests:xodr_data",
            "@com_github_bark_simulator_bark//examples:params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
            "//python:carla-interface.so",
            "//cosimulation_modules/maps:maps"
          ],
  imports = ["@com_github_bark_simulator_bark//python/"],
  deps = [
            "@com_github_bark_simulator_bark//modules/runtime/commons:parameters",
            "@com_github_bark_simulator_bark//modules/runtime/commons:xodr_parser",
            "@com_github_bark_simulator_bark//modules/runtime/viewer:pygame_viewer",
            "//cosimulation_modules/client:carla_client",
            "//cosimulation_modules/client:controller",
            "//cosimulation_modules/client:sensors",
            "//cosimulation_modules/client:viewer"
          ],
)