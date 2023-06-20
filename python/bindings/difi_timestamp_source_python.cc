/*
 * Copyright 2023 Free Software Foundation, Inc.
 *
 * This file is part of GNU Radio
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 *
 */

/***********************************************************************************/
/* This file is automatically generated using bindtool and can be manually
 * edited  */
/* The following lines can be configured to regenerate this file during cmake */
/* If manual edits are made, the following tags should be modified accordingly.
 */
/* BINDTOOL_GEN_AUTOMATIC(0) */
/* BINDTOOL_USE_PYGCCXML(0) */
/* BINDTOOL_HEADER_FILE(difi_timestamp_source.h) */
/* BINDTOOL_HEADER_FILE_HASH(0e6ec89145d7c6118863b748a9e2f03b) */
/***********************************************************************************/

#include <pybind11/complex.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

#include <difi/difi_timestamp_source.h>
// pydoc.h is automatically generated in the build directory
#include <difi_timestamp_source_pydoc.h>

void bind_difi_timestamp_source(py::module &m) {

  py::enum_<::gr::difi::difi_timestamp_source>(m, "difi_timestamp_source")
      .value(
          "DIFI_TIMESTAMP_SIGNAL_PACKET",
          ::gr::difi::difi_timestamp_source::DIFI_TIMESTAMP_SIGNAL_PACKET) // 0
      .value(
          "DIFI_TIMESTAMP_SYSTEM_CLOCK",
          ::gr::difi::difi_timestamp_source::DIFI_TIMESTAMP_SYSTEM_CLOCK) // 1
      .value("DIFI_TIMESTAMP_MIXED",
             ::gr::difi::difi_timestamp_source::DIFI_TIMESTAMP_MIXED) // 2
      .export_values();

  py::implicitly_convertible<int, ::gr::difi::difi_timestamp_source>();
}