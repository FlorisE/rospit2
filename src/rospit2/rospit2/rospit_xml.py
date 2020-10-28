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

"""ROS specific XML Parser for test specifications."""

import os

from lxml import etree

import yaml

from .binary import BinaryCondition, StaticBooleanEvaluator
from .declarative import DummyStep
from .framework import ConditionEvaluatorPair
from .numeric import BothLimitsCondition, \
                     EqualToCondition, \
                     GreaterThanCondition, \
                     GreaterThanOrEqualToCondition, \
                     LessThanCondition, \
                     LessThanOrEqualToCondition, \
                     Limit, \
                     LowerLimitCondition, \
                     NotEqualToCondition, \
                     UpperLimitCondition
from .ros import ExecutionReturnedEvaluator, \
                 MessageEvaluator, \
                 MessageReceivedEvaluator, \
                 NumericMessageEvaluator, \
                 Occurrence, \
                 Publish, \
                 ROSDeclarativeTestCase, \
                 ROSTestSuite, \
                 ServiceCall, \
                 Sleep, \
                 StaticValue, \
                 StringEqualsCondition, \
                 TopicValue

NS = '{https://www.aist.go.jp/rospit}'
XSI_TYPE = '{http://www.w3.org/2001/XMLSchema-instance}type'

_ROOT = os.path.abspath(os.path.dirname(__file__))
SCHEMA_PATH = 'xml/rospit.xsd'


def with_ns(element):
    """Prepend element with name space."""
    return NS + element


def get_parser():
    """Get the parser."""
    loc = open(os.path.join(_ROOT, SCHEMA_PATH), 'r')
    tree = etree.XML(loc.read())
    schema = etree.XMLSchema(tree)
    parser = etree.XMLParser(schema=schema)
    return parser


def get_test_suite_from_xml_path(node, path, validate=True):
    """Get a test suite from a path to an XML file."""
    try:
        test_file = open(path, 'r')
        content = test_file.read()
    except OSError:
        return False

    return get_test_suite_from_xml_string(node, content, validate)


def get_bool(value):
    """Get a bool from value."""
    if str(value) == '1' or value == 'true':
        return True
    elif str(value) == '0' or value == 'false' or value is None:
        return False
    else:
        raise ValueError('value was not recognized as a valid boolean')


def get_test_suite_from_xml_string(node, xml, validate=True):
    """Get a test suite from an XML string."""
    if validate:
        tree = etree.fromstring(xml, get_parser())
    else:
        tree = etree.fromstring(xml)
    return Parser(node, tree)


class SubscriberFactory(object):
    """A factory for subscriber objects."""

    def __init__(self, root):
        """Initialize."""
        assert(root.tag == with_ns('TestSuite'))
        self.messages = {}
        self.msg_value_subscribers = []
        self.msg_received_subscribers = []
        self.message_received_on = set()
        for test_case_elem in root.getchildren():
            self.parse_test_case(test_case_elem)

    def parse_test_case(self, elem):
        """Parse a test case."""
        assert(elem.tag == with_ns('TestCase'))
        for phase_elem in elem.getchildren():
            self.parse_phase(phase_elem)

    def parse_phase(self, elem):
        """Parse a phase."""
        name = elem.tag
        if name == with_ns('SetUp') or \
           name == with_ns('Run') or \
           name == with_ns('TearDown'):
            for step_elem in elem.getchildren():
                self.parse_step_elem(step_elem)
        elif name == with_ns('Preconditions') or \
                name == with_ns('Postconditions') or \
                name == with_ns('Invariants'):
            for condition_elem in elem.getchildren():
                self.parse_condition_elem(condition_elem)
        else:
            raise ValueError('Unexpected tag: {}'.format(name))

    def parse_step_elem(self, elem):
        """Parse a step element."""
        assert(elem.tag == with_ns('Step'))
        for message_elem in elem.getchildren():
            self.parse_message_elem(message_elem)

    def parse_message_elem(self, elem):
        """Parse a message element."""
        assert(
            elem.tag == with_ns('Message') or
            elem.tag == with_ns('MessageYaml'))
        for param_elem in elem.getchildren():
            self.parse_param_elem(param_elem)

    def parse_param_elem(self, elem):
        """Parse a parameter element."""
        assert(elem.tag == with_ns('Parameter'))
        for value_elem in elem.getchildren():
            assert(value_elem.tag == with_ns('Value'))
            elem_type = value_elem.attrib[XSI_TYPE]
            if elem_type == 'TopicValue':
                topic = value_elem.attrib['topic']
                msg_type = value_elem.attrib['type']
                self.msg_value_subscribers.append((topic, msg_type))

    def parse_condition_elem(self, elem):
        """Parse a condition element."""
        assert(elem.tag == with_ns('Precondition') or
               elem.tag == with_ns('Postcondition') or
               elem.tag == with_ns('Invariant'))
        for child in elem.getchildren():
            if child.tag == with_ns('Evaluator'):
                self.parse_evaluator_elem(child)

    def parse_evaluator_elem(self, elem):
        """Parse an evaluator element."""
        assert(elem.tag == with_ns('Evaluator'))
        elem_type = elem.attrib[XSI_TYPE]
        if elem_type == 'MessageReceivedOnTopicEvaluator':
            topic = elem.attrib['topic']
            msg_type = elem.attrib['type']
            self.msg_received_subscribers.append((topic, msg_type))
        elif elem_type == 'MessageEvaluator':
            topic = elem.attrib['topic']
            msg_type = elem.attrib['type']
            self.msg_value_subscribers.append((topic, msg_type))


def get_occurrence(occurrence_str):
    """Convert an occurrence string to an enum value."""
    occurrence_map = {
            'once-and-only': Occurrence.ONCE_AND_ONLY,
            'only-once': Occurrence.ONLY_ONCE,
            'once': Occurrence.ONCE,
            'first': Occurrence.FIRST,
            'last': Occurrence.LAST
            }
    try:
        return occurrence_map[occurrence_str]
    except KeyError:
        raise ValueError(f'{occurrence_str} is not a valid occurrence string')


class Parser(object):
    """A parser for ROS PIT XML tests."""

    def __init__(self, node, root):
        """Initialize."""
        self.node = node
        self.root = root
        self.subs = SubscriberFactory(root)
        self.test_suite = None
        self.last_test_case = None
        self.current_test_case = None

    def parse(self):
        """Parse the test suite."""
        self.test_suite = ROSTestSuite(
            self.node, self.subs, self.root.attrib['name'])
        for child in self.root.getchildren():
            self.last_test_case = self.get_test_case_from_xml_element(child)
            self.test_suite.test_cases.append(self.last_test_case)
        return self.test_suite

    def get_test_case_from_xml_element(self, element):
        """Get test case from element."""
        wait_for_preconditions = get_bool(
            element.attrib.get('wait_for_preconditions', 'false'))
        depends_on_previous = get_bool(
            element.attrib.get('depends_on_previous', 'false'))
        depends_on = [self.last_test_case] if depends_on_previous else None
        test_case = ROSDeclarativeTestCase(
            self.test_suite, name=element.attrib['name'],
            wait_for_preconditions=wait_for_preconditions,
            depends_on=depends_on)
        self.current_test_case = test_case
        for child in element.getchildren():
            name = child.tag
            if name == with_ns('SetUp'):
                for step in child.getchildren():
                    test_case.set_up_steps.append(
                        self.get_step_from_xml_element(step))
            elif name == with_ns('Preconditions'):
                for condition in child.getchildren():
                    test_case.preconditions.append(
                        self.get_condition_from_xml_element(condition))
            elif name == with_ns('Invariants'):
                for invariant in child.getchildren():
                    test_case.invariants.append(
                        self.get_condition_from_xml_element(invariant))
            elif name == with_ns('Run'):
                for step in child.getchildren():
                    test_case.run_steps.append(
                        self.get_step_from_xml_element(step))
            elif name == with_ns('Postconditions'):
                for condition in child.getchildren():
                    test_case.postconditions.append(
                        self.get_condition_from_xml_element(condition))
            elif name == with_ns('TearDown'):
                for step in child.getchildren():
                    test_case.tear_down_steps.append(
                        self.get_step_from_xml_element(step))
            else:
                raise Exception('Unexpected child')
        return test_case

    def get_condition_from_xml_element(self, element):
        """Get condition from element."""
        for child in element.getchildren():
            if child.tag == with_ns('Condition'):
                condition = self.condition_factory(child)
            elif child.tag == with_ns('Evaluator'):
                evaluator = self.evaluator_factory(child)
            else:
                raise Exception('Unexpected child')
        return ConditionEvaluatorPair(condition, evaluator)

    def get_parameters(self, element):
        """Get parameter from element."""
        if element.tag == with_ns('Message'):
            return self.get_message(element)
        elif element.tag == with_ns('MessageYaml'):
            return yaml.safe_load(element.text)
        else:
            raise Exception(
                'Unidenfied message specification {}'.format(element.tag))

    def get_step_from_xml_element(self, element):
        """Get step from element."""
        elem_type = element.attrib[XSI_TYPE]
        if elem_type == 'Dummy':
            return DummyStep()
        elif elem_type == 'ServiceCall':
            if len(element) > 0:
                parameters = self.get_parameters(element[0])
            else:
                parameters = {}
            save_result = get_bool(element.attrib.get('save_result', 'false'))
            return ServiceCall(
                self.node, element.attrib['service'], element.attrib['type'],
                parameters, save_result=save_result)
        elif elem_type == 'Publish':
            topic = element.attrib.get('topic')
            msg_type = element.attrib.get('type')
            duration = float(element.attrib.get('duration'))
            rate = int(element.attrib.get('rate'))
            if len(element) > 0:
                parameters = self.get_parameters(element[0])
            else:
                parameters = {}
            return Publish(
                self.node, topic, msg_type, duration, rate, parameters)
        elif elem_type == 'Sleep':
            return Sleep(
                element.attrib['duration'],
                element.attrib.get('unit', 'second'))
        else:
            raise Exception('Unidenfied step {}'.format(elem_type))

    def get_message(self, element):
        """Get message from element."""
        children = element.getchildren()
        if len(children) == 1:
            message_elem = children[0]
            param_elems = message_elem.getchildren()
            if len(param_elems) != 0:
                params = self.get_param_from_elements(param_elems)
            else:
                params = {}
            return params
        else:
            return None

    def get_param_from_elements(self, elements):
        """Get param for elements."""
        params = {}
        for param in elements:
            current_param = params
            value = self.get_value_from_element(param.getchildren()[0])
            name = param.attrib['name']
            name_parts = name.split('/')
            while len(name_parts) > 0:
                current_part = name_parts.pop(0)
                if len(name_parts) > 0:
                    current_param = current_param.setdefault(current_part, {})
                else:
                    current_param[current_part] = value
        return params

    def get_value_from_element(self, element):
        """Get the value of element."""
        elem_type = element.attrib[XSI_TYPE]
        if elem_type == 'TopicValue':
            return TopicValue(
                element.attrib['topic'],
                element.attrib.get('field', None),
                self.test_suite)
        elif elem_type == 'StaticValue':
            return StaticValue(
                element.attrib['value'],
                element.attrib['type'],
                self.test_suite)
        else:
            raise ValueError('Element is of unknown type')

    def condition_factory(self, element):
        """Create a condition for element."""
        elem_type = element.attrib[XSI_TYPE]
        attr = element.attrib
        name = attr.get('name', '')
        if elem_type == 'Binary':
            return BinaryCondition(get_bool(attr['value']), name)
        elif elem_type == 'GreaterThan':
            return GreaterThanCondition(float(attr['value']), name)
        elif elem_type == 'GreaterThanOrEqualTo':
            return GreaterThanOrEqualToCondition(float(attr['value']), name)
        elif elem_type == 'NotEqualTo':
            return NotEqualToCondition(float(attr['value']), name)
        elif elem_type == 'EqualTo':
            return EqualToCondition(float(attr['value']), name)
        elif elem_type == 'LessThanOrEqualTo':
            return LessThanOrEqualToCondition(float(attr['value']), name)
        elif elem_type == 'LessThan':
            return LessThanCondition(float(attr['value']), name)
        elif elem_type == 'UpperLimit':
            limit = Limit(float(attr['value']), get_bool(attr['inclusive']))
            return UpperLimitCondition(limit, name)
        elif elem_type == 'LowerLimit':
            limit = Limit(float(attr['value']), get_bool(attr['inclusive']))
            return LowerLimitCondition(limit, name)
        elif elem_type == 'BothLimits':
            lower_limit = Limit(
                float(attr['lower_limit_value']),
                get_bool(attr.get('lower_limit_inclusive', 'true')))
            upper_limit = Limit(
                float(attr['upper_limit_value']),
                get_bool(attr.get('upper_limit_inclusive', 'true')))
            return BothLimitsCondition(lower_limit, upper_limit, name)
        elif elem_type == 'StringEquals':
            return StringEqualsCondition()
        else:
            raise ValueError('Unexpected type {}'.format(elem_type))

    def evaluator_factory(self, elem):
        """Create an evaluator for element."""
        elem_type = elem.attrib[XSI_TYPE]
        if elem_type == 'StaticBooleanEvaluator':
            return StaticBooleanEvaluator(get_bool(elem.attrib['value']))
        elif elem_type == 'MessageReceivedEvaluator':
            return MessageReceivedEvaluator(
                self.node, elem.attrib['topic'], elem.attrib['type'],
                elem.attrib.get('field', None))
        elif elem_type == 'MessageEvaluator':
            return MessageEvaluator(self.node, elem.attrib['topic'],
                                    elem.attrib['type'],
                                    get_occurrence(elem.attrib['occurrence']),
                                    get_bool(elem.attrib['negate']),
                                    elem.attrib.get('field', None))
        elif elem_type == 'ExecutionReturnedEvaluator':
            field = elem.attrib.get('field', None)
            return ExecutionReturnedEvaluator(self.current_test_case, field)
        elif elem_type == 'NumericMessageEvaluator':
            return NumericMessageEvaluator(
                self.node, elem.attrib['topic'], elem.attrib['type'],
                elem.attrib.get('field', None))
        else:
            raise ValueError('Unexpected type {}'.format(elem_type))
