// Copyright (c) 2019 fortiss GmbH, Julian Bernhard, Klemens Esterle, Patrick Hart, Tobias Kessler
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include "cosimulation_modules/mpc/mpc.hpp"
#include "mpc.hpp"

namespace py = pybind11;

void python_mpc(py::module m) {
  py::class_<MPC, std::shared_ptr<MPC>>(m, "MPC")
    .def(py::init<>())
    .def("__repr__", [](const MPC &a) {
      return "carla-interface.mpc.MPC";
    })
    .def("solve",&MPC::solve);
}