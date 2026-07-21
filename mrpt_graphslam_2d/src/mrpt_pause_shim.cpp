// Copyright (c) 2024-2026, Jose Luis Blanco-Claraco.
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

/**
 * @file mrpt_pause_shim.cpp
 * @brief No-op override of mrpt::system::pause() for headless ROS nodes.
 *
 * The MRPT CGraphSlamEngine constructor calls mrpt::system::pause() when the
 * constraint-type's fully-qualified class name (e.g.
 * "mrpt::poses::CPosePDFGaussianInf") does not match its internal short-name
 * whitelist ("CPosePDFGaussianInf").  This is a latent MRPT 2.15 bug: the
 * whitelist uses bare class names while GetRuntimeClass()->className returns
 * namespace-qualified names.
 *
 * Because mrpt::system::pause() is an ELF global symbol exported from
 * libmrpt-system.so, defining it here causes the dynamic linker to prefer the
 * executable's definition, making the interactive pause a no-op for all three
 * graphslam executables that link this translation unit.
 *
 * When the upstream MRPT library is fixed (whitelist updated to use
 * fully-qualified names), this file can safely be removed.
 */

#include <mrpt/system/os.h>

#include <rclcpp/rclcpp.hpp>

namespace mrpt
{
namespace system
{

// Interpose mrpt::system::pause() – replace the blocking keypress wait with a
// ROS WARN log so operators know the constraint-type check was skipped, then
// return immediately.
void pause(const std::string & msg) noexcept
{
  RCLCPP_WARN(
    rclcpp::get_logger("mrpt_graphslam"),
    "[mrpt::system::pause() suppressed for headless execution] %s",
    msg.c_str());
}

}  // namespace system
}  // namespace mrpt
