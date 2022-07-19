py_binary(
  name = "fill_world_with_npc",
  srcs = ["fill_world_with_npc.py"],
  data = [
            "@bark_project//bark/python_wrapper:core.so",
            "@bark_project//bark/runtime/tests:xodr_data",
            "//examples/eg_params:eg_params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
          ],
  imports = ["../external/bark_project/bark/python_wrapper/"],
  deps = [
            "@bark_project//bark/runtime/commons:parameters",
            "@bark_project//bark/runtime/commons:xodr_parser",
            "@bark_project//bark/runtime/viewer:pygame_viewer",
            "@bark_project//bark/runtime:runtime",
            "//cosimulation_modules/client:carla_client",
            "//cosimulation_modules/client:sensors",
            "//cosimulation_modules/client:viewer",
          ],
)

py_binary(
  name = "fill_world_with_bark_ego",
  srcs = ["fill_world_with_bark_ego.py"],
  data = [
            "@bark_project//bark/python_wrapper:core.so",
            "@bark_project//bark/runtime/tests:xodr_data",
            "//examples/eg_params:eg_params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
            "//python:carla-interface.so",
          ],
  imports = ["../external/bark_project/bark/python_wrapper/"],
  deps = [
            "@bark_project//bark/runtime/commons:parameters",
            "@bark_project//bark/runtime/commons:xodr_parser",
            "@bark_project//bark/runtime/viewer:pygame_viewer",
            "@bark_project//bark/runtime:runtime",
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
            "@bark_project//bark/python_wrapper:core.so",
            "@bark_project//bark/runtime/tests:xodr_data",
            "//examples/eg_params:eg_params",
            "@bark_project//bark/runtime:runtime",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
            "//python:carla-interface.so",
            "//cosimulation_modules/maps:maps"
          ],
  imports = ["../external/bark_project/bark/python_wrapper/"],
  deps = [
            "@bark_project//bark/runtime/commons:parameters",
            "@bark_project//bark/runtime/commons:xodr_parser",
            "@bark_project//bark/runtime:runtime",
            "@bark_project//bark/runtime/viewer:pygame_viewer",
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
            "@bark_project//bark/python_wrapper:core.so",
            "@bark_project//bark/runtime/tests:xodr_data",
            "@bark_project//bark/runtime:runtime",
            "//examples/eg_params:eg_params",
            "@carla//:carla_python_lib",
            "@carla//:carla_server",
            "//python:carla-interface.so",
            "//cosimulation_modules/maps:maps"
          ],
  imports = ["../external/bark_project/bark/python_wrapper/"],
  deps = [
            "@bark_project//bark/runtime/commons:parameters",
            "@bark_project//bark/runtime/commons:xodr_parser",
            "@bark_project//bark/runtime/viewer:pygame_viewer",
            "@bark_project//bark/runtime:runtime",
            "//cosimulation_modules/client:carla_client",
            "//cosimulation_modules/client:controller",
            "//cosimulation_modules/client:sensors",
            "//cosimulation_modules/client:viewer"
          ],
)