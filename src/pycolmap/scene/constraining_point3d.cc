#include "colmap/scene/constraining_point3d.h"

#include "pycolmap/helpers.h"
#include "pycolmap/pybind11_extension.h"
#include "pycolmap/scene/types.h"

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

using namespace colmap;
using namespace pybind11::literals;
namespace py = pybind11;

void BindConstrainingPoint3D(py::module& m) {
  py::classh_ext<ConstrainingPoint3D> PyConstrainingPoint3D(
      m, "ConstrainingPoint3D");
  PyConstrainingPoint3D.def(py::init<>())
      .def(py::init<const Eigen::Vector3d&>(), "xyz"_a)
      .def_readwrite("xyz", &ConstrainingPoint3D::xyz);
  MakeDataclass(PyConstrainingPoint3D);

  py::bind_map<ConstrainingPoint3DMap>(m, "ConstrainingPoint3DMap");
}
