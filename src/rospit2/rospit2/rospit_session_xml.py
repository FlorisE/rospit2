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

"""ROS specific XML Parser for test session specifications."""

import os

from lxml import etree

from .ros import ROSPITSession, ROSPITSessionEpisode
from .ros_parameters import ROSPITParameter, \
                            parse_csv_parameters, \
                            parse_xml_parameters

NS = '{https://www.aist.go.jp/rospit-session}'
XSI_TYPE = '{http://www.w3.org/2001/XMLSchema-instance}type'

_ROOT = os.path.abspath(os.path.dirname(__file__))
SCHEMA_PATH = 'xml/session.xsd'


def get_parser():
    """Get the parser."""
    loc = open(os.path.join(_ROOT, SCHEMA_PATH), 'r')
    tree = etree.XML(loc.read())
    schema = etree.XMLSchema(tree)
    parser = etree.XMLParser(schema=schema)
    return parser


def get_session_from_xml_string(node, xml, validate=True):
    """Get a test session from an XML string."""
    if validate:
        tree = etree.fromstring(xml, get_parser())
    else:
        tree = etree.fromstring(xml)
    return SessionParser(node, tree)


def get_session_from_xml_path(node, path, validate=True):
    """Get a test session from a path to an XML file."""
    try:
        test_file = open(path, 'r')
        content = test_file.read()
    except OSError:
        node.get_logger().error(
            f'Could not open path {path}.')
        return False

    return get_session_from_xml_string(node, content, validate)


def with_ns(element):
    """Prepend element with name space."""
    return NS + element


class SessionParser(object):
    """A parser for ROSPIT XML Test Sessions."""

    def __init__(self, node, root):
        """Initialize."""
        self.node = node
        self.root = root
        self.session = None

    def parse(self):
        """Parse the test session."""
        self.session = ROSPITSession(self.node)

        for child in self.root.getchildren():
            if child.tag == with_ns('Episode'):
                self.session.episodes.append(
                    self.get_test_from_xml_element(child))
            else:
                raise RuntimeError(f'Unexpected child {child.tag}')

        return self.session

    def get_test_from_xml_element(self, test):
        """Parse Test element."""
        path = test.attrib['test_definition_path']
        name = test.attrib.get('name', 'Untitled')
        test_case = test.attrib.get('test_case', None)
        children = test.getchildren()
        parameters_xml = test.attrib.get('parameters_xml', None)
        parameters_csv = test.attrib.get('parameters_csv', None)

        parameters = []
        if parameters_xml and parameters_csv:
            raise RuntimeError(
                'Cannot have both parameters_csv and parameters_xml')
        if len(children) > 0 and (parameters_xml or parameters_csv):
            raise RuntimeError(
                'Cannot have both child parameters and a parameter file')

        if len(children) > 0:
            for child in children:
                if child.tag == with_ns('Parameter'):
                    parameters.append(
                        self.get_parameter_from_xml_element(child))
                else:
                    raise RuntimeError(f'Unexpected child {child.tag}')
        elif parameters_xml:
            parameters = parse_xml_parameters(parameters_xml)
        elif parameters_csv:
            parameters = parse_csv_parameters(parameters_xml)

        return ROSPITSessionEpisode(name, path, test_case, parameters)

    def get_parameter_from_xml_element(self, parameter):
        """Parse Parameter element."""
        return ROSPITParameter(
            parameter.attrib['node_name'],
            parameter.attrib['parameter_name'],
            parameter.attrib['parameter_value'],
            parameter.attrib.get('store_as', None))
