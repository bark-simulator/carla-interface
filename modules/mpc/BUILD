cc_library(
  name = "cost_function",
  srcs = ["cost_function.cpp"],
  hdrs = ["cost_function.hpp"],
  deps = [
    "@com_github_eigen_eigen//:eigen",
    "@com_google_ceres_solver//:ceres",
  ],
  visibility = ["//visibility:public"],
)

cc_library(
  name = "mpc",
  srcs = ["mpc.cpp"],
  hdrs = ["mpc.hpp"],
  deps = [
    "@com_github_eigen_eigen//:eigen",
    "@com_google_ceres_solver//:ceres",
    ":cost_function",
  ],
  visibility = ["//visibility:public"],
)