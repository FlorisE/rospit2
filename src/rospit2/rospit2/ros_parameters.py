# Copyright (c) 2020 AIST.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""ROS parameters."""

import asyncio
from collections import namedtuple

from lxml import etree

from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters

from ros2param.api import get_parameter_value

from rospit_msgs.msg import Parameter as ROSPITParameterMsg


ROSPITParameter = namedtuple(
    'ROSPITParameter', 'node_name name value store_as')


def map_param_to_msg(parameter):
    """Map a ROSPIT Parameter to a ROSPIT Parameter message."""
    param_msg = ROSPITParameterMsg()
    param_msg.node_name = parameter.node_name
    param_msg.parameter_name = parameter.name
    param_msg.parameter_value = parameter.value
    param_msg.store_as = parameter.store_as
    return param_msg


def map_msg_to_param(param_msg):
    """Map a ROSPIT Parameter message to a ROSPIT Parameter."""
    return ROSPITParameter(param_msg.node_name,
                           param_msg.parameter_name,
                           param_msg.parameter_value,
                           param_msg.store_as)


async def store_parameters(
        node,
        parameters,
        stored_parameters):
    """Store the parameters."""
    tasks = []
    for parameter in parameters:
        tasks.append(
            asyncio.create_task(
                set_parameter(
                    node,
                    parameter.node_name,
                    parameter.name,
                    parameter.value,
                    parameter.store_as,
                    stored_parameters)))
    await asyncio.wait(tasks)


def parse_csv_parameters(path, separator):
    """Parse a ROSPIT CSV Parameter file."""
    param_file = open(path)

    parameters = []
    for param_line in param_file.read().splitlines():
        param_parts = param_line.split(separator)
        num_param_parts = len(param_parts)

        if num_param_parts < 3 or num_param_parts > 4:
            raise RuntimeError(
                f'Expected 4 or 5 elements in the parameter line but '
                f'got {num_param_parts}')

        if num_param_parts >= 3:
            node_name = param_parts[0]
            param_name = param_parts[1]
            param_value = param_parts[2]

        if num_param_parts == 4:
            store_as = param_parts[3]
        else:
            store_as = None

        parameters.append(
            ROSPITParameter(
                node_name,
                param_name,
                param_value,
                store_as))
    return parameters


def parse_and_store_csv_parameters(
        node,
        path,
        separator,
        stored_parameters):
    """Parse a ROSPIT CSV Parameter file and store the parameters."""
    parameters = parse_csv_parameters(path, separator)
    asyncio.run(store_parameters(node, parameters, stored_parameters))


def parse_xml_parameter_element(element):
    """Parse a ROSPIT XML Parameter element."""
    if element.tag == 'Parameter':
        return ROSPITParameter(
            element.attrib['node_name'],
            element.attrib['parameter_name'],
            element.attrib['parameter_value'],
            element.attrib.get('store_as', None))
    else:
        raise RuntimeError(
            f'Expected Parameter element but got {element.tag}')


def parse_xml_parameters_element(element):
    """Parse a ROSPIT XML Parameters element."""
    parameters = []
    if element.tag == 'Parameters':
        for child in element.getchildren():
            parameter = parse_xml_parameter_element(child)
            parameters.append(parameter)
    else:
        raise RuntimeError(
            f'Expected Parameters element but got {element.tag}')
    return parameters


def parse_xml_parameters(path):
    """Parse a ROSPIT XML Parameter file."""
    param_file = open(path)
    content = param_file.read()
    tree = etree.fromstring(content)
    return parse_xml_parameters_element(tree)


def parse_and_store_xml_parameters(
        node,
        path,
        stored_parameters):
    """Parse a ROSPIT XML Parameter file and store the parameters."""
    parameters = parse_xml_parameters(path)
    asyncio.run(store_parameters(node, parameters, stored_parameters))


def get_parameter(
        node,
        node_name,
        parameter_name,
        store_as,
        stored_parameters):
    """Get a parameter from the ROS Parameter Server."""
    if store_as:
        node.get_logger().info(
            'Getting parameter {} of node {}, storing as {}'.format(
                parameter_name, node_name, store_as))
    else:
        node.get_logger().info(
            'Getting parameter {} of node {}'.format(
                parameter_name, node_name))

    client = node.create_client(
        GetParameters, f'{node_name}/get_parameters')
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    request = GetParameters.Request()
    request.names = [parameter_name]
    result = client.call(request)
    key = store_as or f'/{node_name}/{parameter_name}'
    pvalue = result.values[0]
    stored_parameters[key] = pvalue
    if pvalue.type == ParameterType.PARAMETER_BOOL:
        label = 'Boolean value is:'
        value = pvalue.bool_value
    elif pvalue.type == ParameterType.PARAMETER_INTEGER:
        label = 'Integer value is:'
        value = pvalue.integer_value
    elif pvalue.type == ParameterType.PARAMETER_DOUBLE:
        label = 'Double value is:'
        value = pvalue.double_value
    elif pvalue.type == ParameterType.PARAMETER_STRING:
        label = 'String value is:'
        value = pvalue.string_value
    elif pvalue.type == ParameterType.PARAMETER_BYTE_ARRAY:
        label = 'Byte values are:'
        value = pvalue.byte_array_value
    elif pvalue.type == ParameterType.PARAMETER_BOOL_ARRAY:
        label = 'Boolean values are:'
        value = pvalue.bool_array_value
    elif pvalue.type == ParameterType.PARAMETER_INTEGER_ARRAY:
        label = 'Integer values are:'
        value = pvalue.integer_array_value
    elif pvalue.type == ParameterType.PARAMETER_DOUBLE_ARRAY:
        label = 'Double values are:'
        value = pvalue.double_array_value
    elif pvalue.type == ParameterType.PARAMETER_STRING_ARRAY:
        label = 'String values are:'
        value = pvalue.string_array_value
    elif pvalue.type == ParameterType.PARAMETER_NOT_SET:
        label = 'Parameter not set.'
        value = None
    else:
        raise ValueError(
            f"Unknown parameter type '{pvalue.type}' for key {key}")
    node.get_logger().info(f'{label} {value}')


async def set_parameter(
        node,
        node_name,
        parameter_name,
        parameter_value,
        store_as,
        stored_parameters):
    """Set a parameter on the ROS Parameter Server."""
    if store_as:
        node.get_logger().info(
            'Setting parameter {} of node {} to {}, storing as {}'.format(
                parameter_name, node_name,
                parameter_value, store_as))
    else:
        node.get_logger().info(
            'Setting parameter {} of node {} to {}'.format(
                parameter_name, node_name, parameter_value))

    client = node.create_client(
        SetParameters, f'{node_name}/set_parameters')
    ready = client.wait_for_service(timeout_sec=5.0)
    if not ready:
        raise RuntimeError('Wait for service timed out')

    parameter_encoded = get_parameter_value(
        string_value=parameter_value)

    parameter = Parameter()
    parameter.name = parameter_name
    parameter.value = parameter_encoded

    request = SetParameters.Request()
    request.parameters = [parameter]
    await client.call_async(request)

    key = store_as or f'/{node_name}/{parameter_name}'
    stored_parameters[key] = parameter_encoded
