// -*- c++ -*- //
// Copyright (c) Microsoft Corporation.
// Licensed under the GNU General Public License v3.0 or later.
// See License.txt in the project root for license information.
//

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <difi/difi_source_cpp.h>
// pydoc.h is automatically generated in the build directory
#include <difi_source_cpp_pydoc.h>


template <typename T, typename S>
void bind_difi_source_cpp_template(py::module& m, const char* classname)
{

    using difi_source_cpp    = ::gr::difi::difi_source_cpp<T, S>;


    py::class_<difi_source_cpp, gr::sync_block, gr::block, gr::basic_block,
        std::shared_ptr<difi_source_cpp>>(m, classname,  D(difi_source_cpp))

        .def(py::init(&difi_source_cpp::make),
           py::arg("ip_addr"),
           py::arg("port"),
           py::arg("protocol"),
           py::arg("stream_number"),
           py::arg("context_pkt_behavior"),
           py::arg("timestamp_source"),
           D(difi_source_cpp,make)
        );
}

void bind_difi_source_cpp(py::module& m)
{
    bind_difi_source_cpp_template<gr_complex, std::complex<int16_t>>(m, "difi_source_cpp_sc16_fc32");
    bind_difi_source_cpp_template<gr_complex, std::complex<int8_t>>(m, "difi_source_cpp_sc8_fc32");
    bind_difi_source_cpp_template<std::complex<int8_t>, std::complex<int16_t>>(m, "difi_source_cpp_sc16_sc8");
    bind_difi_source_cpp_template<std::complex<int8_t>, std::complex<int8_t>>(m, "difi_source_cpp_sc8_sc8");
}
