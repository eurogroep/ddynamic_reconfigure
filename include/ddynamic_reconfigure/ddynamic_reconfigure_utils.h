/**
 * Copyright 2019 PAL Robotics S.L.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <limits>
#include <map>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <stdexcept>
#include <string>
#include <vector>

template <typename T>
inline T getMin()
{
  if (std::is_same<T, double>::value)
  {
    return -1e9;
  }
  return std::numeric_limits<T>::min();
}

template <>
inline bool getMin()
{
  return false;
}

template <>
inline std::string getMin<std::string>()
{
  return "";
}

template <typename T>
inline T getMax()
{
  if (std::is_same<T, double>::value)
  {
    return 1e9;
  }
  return std::numeric_limits<T>::max();
}

template <>
inline bool getMax()
{
  return true;
}

template <>
inline std::string getMax()
{
  return "";
}

//!
//! \brief declare_parameter_if_not_declared
//! \tparam NodeT Type of the node
//! \param node Node
//! \param param_name Parameter name
//! \param default_value Default value
//! \param parameter_descriptor Descriptor of the parameter
//! \return parameter value
//!
template <typename NodeT, typename ParamT>
ParamT declare_parameter_if_not_declared(
    NodeT node, const std::string& param_name, const ParamT& default_value,
    const rcl_interfaces::msg::ParameterDescriptor& parameter_descriptor = rcl_interfaces::msg::ParameterDescriptor())
{
  if (!node->has_parameter(param_name))
  {
    std::cout << "declare" << std::endl;
    return node->declare_parameter(param_name, default_value, parameter_descriptor);
  }
  else
  {
    ParamT param_value;
    node->get_parameter(param_name, param_value);
    return param_value;
  }
}