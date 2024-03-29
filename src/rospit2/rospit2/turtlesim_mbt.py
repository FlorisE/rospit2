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

"""Example of Model-Based Testing using turtlesim."""

import rclpy
from rclpy.node import Node
from rospit_msgs.srv import MBTInitializer, MBTIterator
from rospit_msgs.msg import Parameter


def make_parameter(parameter_name, parameter_value, store_as):
    """Make a single parameter."""
    p = Parameter()
    p.node_name = 'test_runner'
    p.parameter_name = parameter_name
    p.parameter_value = parameter_value
    p.store_as = store_as
    return p


def make_parameters(llx_pre, ulx_pre, lly_pre, uly_pre,
                    llx_post, ulx_post, lly_post, uly_post,
                    llx_inv, ulx_inv, lly_inv, uly_inv,
                    lin_x, lin_y, lin_z,
                    ang_x, ang_y, ang_z):
    """Make parameters for one run."""
    llx_pre = make_parameter('lower_limit_x_pre', str(llx_pre), 'llx_pre')
    ulx_pre = make_parameter('upper_limit_x_pre', str(ulx_pre), 'ulx_pre')
    lly_pre = make_parameter('lower_limit_y_pre', str(lly_pre), 'lly_pre')
    uly_pre = make_parameter('upper_limit_y_pre', str(uly_pre), 'uly_pre')
    llx_post = make_parameter('lower_limit_x_post', str(llx_post), 'llx_post')
    ulx_post = make_parameter('upper_limit_x_post', str(ulx_post), 'ulx_post')
    lly_post = make_parameter('lower_limit_y_post', str(lly_post), 'lly_post')
    uly_post = make_parameter('upper_limit_y_post', str(uly_post), 'uly_post')
    llx_inv = make_parameter('lower_limit_x_inv', str(llx_inv), 'llx')
    ulx_inv = make_parameter('upper_limit_x_inv', str(ulx_inv), 'ulx')
    lly_inv = make_parameter('lower_limit_y_inv', str(lly_inv), 'lly')
    uly_inv = make_parameter('upper_limit_y_inv', str(uly_inv), 'uly')
    lin_x = make_parameter('lin_x', str(lin_x), 'lin_x')
    lin_y = make_parameter('lin_y', str(lin_y), 'lin_y')
    lin_z = make_parameter('lin_z', str(lin_z), 'lin_z')
    ang_x = make_parameter('ang_x', str(ang_x), 'ang_x')
    ang_y = make_parameter('ang_y', str(ang_y), 'ang_y')
    ang_z = make_parameter('ang_z', str(ang_z), 'ang_z')

    return [llx_pre, ulx_pre, lly_pre, uly_pre,
            llx_post, ulx_post, lly_post, uly_post,
            llx_inv, ulx_inv, lly_inv, uly_inv,
            lin_x, lin_y, lin_z,
            ang_x, ang_y, ang_z]


#               Preconditions           Postconditions          Invariants              Linear         Angular            # noqa
#               llx   ulx   lly   uly   llx   ulx   lly   uly   llx   ulx   lly   uly   x     y     z    x    y    z      # noqa
TEST_VALUES = [[5.54, 5.55, 5.54, 5.55, 7.53, 7.57, 5.54, 5.55, 5.54, 7.57, 5.54, 5.55,  1.0,  0.0, 0.0, 0.0, 0.0, 0.0],  # noqa
               [5.54, 5.55, 5.54, 5.55, 3.54, 3.55, 5.54, 5.55, 3.54, 5.55, 5.54, 5.55, -1.0,  0.0, 0.0, 0.0, 0.0, 0.0],  # noqa
               [5.54, 5.55, 5.54, 5.55, 5.54, 5.55, 7.53, 7.57, 3.54, 5.55, 5.54, 7.57,  0.0,  1.0, 0.0, 0.0, 0.0, 0.0],  # noqa
               [5.54, 5.55, 5.54, 5.55, 5.54, 5.55, 3.53, 3.57, 3.54, 5.55, 3.53, 5.55,  0.0, -1.0, 0.0, 0.0, 0.0, 0.0]]  # noqa


class TurtlesimMBTServices(Node):
    """Services for turtlesim MBT."""

    def __init__(self):
        """Initialize."""
        super().__init__('turtlesim_mbt_services')
        self.initialize_srv = self.create_service(MBTInitializer,
                                                  'initialize_turtlesim',
                                                  self.initialize_callback)
        self.iterate_srv = self.create_service(MBTIterator,
                                               'iterate_turtlesim',
                                               self.iterate_callback)

    def initialize_callback(self, request, response):
        """Callback for initializing the oracle."""
        return response

    def iterate_callback(self, request, response):
        """Callback for iterating the oracle."""
        iteration = request.iteration
        if iteration > 3:
            response.exhausted = True
            self.get_logger().info(
                f'Requested iteration {iteration} but iterator is exhausted')
            return response
        parameters = make_parameters(*(TEST_VALUES[iteration]))
        for parameter in parameters:
            response.parameters.append(parameter)
        response.exhausted = False
        return response


def main(args=None):
    """Start turtlesim MBT services."""
    rclpy.init(args=args)
    node = TurtlesimMBTServices()
    rclpy.spin(node)
    rclpy.shutdown()
