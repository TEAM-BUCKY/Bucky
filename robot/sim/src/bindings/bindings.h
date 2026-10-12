#pragma once

#include <pybind11/pybind11.h>

namespace bucky_bindings {
void bind_board(pybind11::module_& m);
void bind_firmware(pybind11::module_& m);
}  // namespace bucky_bindings
