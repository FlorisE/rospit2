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
from functools import partial

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
                     LowerLimitCondition, \
                     NotEqualToCondition, \
                     UpperLimitCondition
from .ros import ExecutionReturnedEvaluator, \
                 MessageEvaluator, \
                 MessageReceivedEvaluator, \
                 MessagesEvaluator, \
                 NumericMessageEvaluator, \
                 NumericMessagesEvaluator, \
                 Occurrence, \
                 ROSDeclarativeTestCase, \
                 ROSTestSuite, \
                 StringEqualsCondition, \
                 get_boolean_value, get_numeric_value, get_string_value
from .ros_steps import GetParameter, \
                       Launch, \
                       LoadParametersFromCSVFile, \
                       LoadParametersFromXMLFile, \
                       Publish, \
                       Run, \
                       ServiceCall, \
                       SetParameter, \
                       Sleep, \
                       StaticValue, \
                       TopicValue

NS = '{https://www.aist.go.jp/rospit}'
XSI_TYPE = '{http://www.w3.org/2001/XMLSchema-instance}type'

_ROOT = os.path.abspath(os.path.dirname(__file__))
SCHEMA_PATH = 'xml/rospit.xsd'


def get_parser():
    """Get the parser."""
    loc = open(os.path.join(_ROOT, SCHEMA_PATH), 'r')
    tree = etree.XML(loc.read())
    schema = etree.XMLSchema(tree)
    parser = etree.XMLParser(schema=schema)
    return parser


def get_test_suite_from_xml_string(node, xml, validate=True):
    """Get a test suite from an XML string."""
    if validate:
        tree = etree.fromstring(xml, get_parser())
    else:
        tree = etree.fromstring(xml)
    return TestSuiteParser(node, tree)


def get_test_suite_from_xml_path(node, path, validate=True):
    """Get a test suite from a path to an XML file."""
    try:
        test_file = open(path, 'r')
        content = test_file.read()
    except OSError:
        return False

    return get_test_suite_from_xml_string(node, content, validate)


def with_ns(element):
    """Prepend element with name space."""
    return NS + element


class SubscriberFactory(object):
    """A factory for subscriber objects."""

    def __init__(self, root):
        """Initialize."""
        assert(root.tag == with_ns('TestSuite'))
        self.messages = {}
        self.msg_value_subscribers = []
        self.msg_received_subscribers = []
        self.message_received_on = set()
        for elem in root.getchildren():
            if elem.tag == with_ns('TestCase'):
                self.parse_test_case(elem)

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
        elif elem_type == 'MessageEvaluator' or \
                elem_type == 'MessagesEvaluator' or \
                elem_type == 'NumericMessageEvaluator' or \
                elem_type == 'NumericMessagesEvaluator':
            topic = elem.attrib['topic']
            msg_type = elem.attrib['type']
            self.msg_value_subscribers.append((topic, msg_type))
        else:
            raise Exception(f'Unexpected child of type {elem_type}')


def get_bool(v):
    """Get a bool from value."""
    if str(v) == '1' or v == 'true' or v is True:
        return True
    elif str(v) == '0' or v == 'false' or v is None or v is False:
        return False
    else:
        raise ValueError(f'value {v} was not recognized as a valid boolean')


def get_occurrence(occurrence_str):
    """Convert an occurrence string to an enum value."""
    occurrence_map = {
            'once-and-only': Occurrence.ONCE_AND_ONLY,
            'only-once': Occurrence.ONLY_ONCE,
            'once': Occurrence.ONCE,
            'first': Occurrence.FIRST,
            'last': Occurrence.LAST,
            'always': Occurrence.ALWAYS
            }
    try:
        return occurrence_map[occurrence_str]
    except KeyError:
        raise ValueError(f'{occurrence_str} is not a valid occurrence string')


class TestSuiteParser(object):
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
            name = child.tag
            if name == with_ns('OneTimeSetUp'):
                for step in child.getchildren():
                    self.test_suite.one_time_set_up_steps.append(
                        self.get_step_from_xml_element(step))
            elif name == with_ns('SetUp'):
                for step in child.getchildren():
                    self.test_suite.set_up_steps.append(
                        self.get_step_from_xml_element(step))
            elif name == with_ns('TestCase'):
                self.last_test_case = self.get_test_case_from_xml_elem(child)
                self.test_suite.test_cases.append(self.last_test_case)
            elif name == with_ns('TearDown'):
                for step in child.getchildren():
                    self.test_suite.tear_down_steps.append(
                        self.get_step_from_xml_element(step))
            elif name == with_ns('OneTimeTearDown'):
                for step in child.getchildren():
                    self.test_suite.one_time_tear_down_steps.append(
                        self.get_step_from_xml_element(step))
        return self.test_suite

    def get_test_case_from_xml_elem(self, element):
        """Get test case from element."""
        wait_for_preconditions = get_bool(
            element.attrib.get('wait_for_preconditions', 'false'))
        depends_on_previous = get_bool(
            element.attrib.get('depends_on_previous', 'false'))
        depends_on = [self.last_test_case] if depends_on_previous else None

        # initialize and then update, so we can set current_test_case already
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

    def get_step_from_xml_element(self, element):
        """Get step from element."""
        elem_type = element.attrib[XSI_TYPE]
        if elem_type == 'Dummy':
            return DummyStep()
        elif elem_type == 'Launch':
            try:
                package_name = element.attrib['package_name']
            except KeyError:
                package_name = None
            try:
                launch_file_name = element.attrib['launch_file_name']
            except KeyError:
                launch_file_name = None
            try:
                launch_arguments = element.attrib['launch_arguments']
            except KeyError:
                launch_arguments = []
            try:
                debug = element.attrib['debug']
            except KeyError:
                debug = False

            return Launch(
                self.node,
                package_name,
                launch_file_name,
                launch_arguments,
                debug)
        elif elem_type == 'Publish':
            topic = element.attrib.get('topic')
            msg_type = element.attrib.get('type')
            duration = float(element.attrib.get('duration'))
            rate = int(element.attrib.get('rate'))
            use_substitution = False
            if len(element) > 0:
                if element[0].tag == with_ns('Message'):
                    parameters = self.get_message(element[0])
                elif element[0].tag == with_ns('MessageYaml'):
                    use_substitution = get_bool(
                        element[0].attrib.get(
                            'use_substitution', False))
                    parameters = yaml.safe_load(element[0].text)
                else:
                    raise Exception(
                        'Unidenfied message specification {}'.format(
                            element[0].tag))
            else:
                parameters = {}
            return Publish(
                self.node, topic, msg_type, duration, rate,
                parameters, self.test_suite.stored_parameters,
                use_substitution=use_substitution)
        elif elem_type == 'GetParameter':
            node_name = element.attrib.get('node_name')
            parameter_name = element.attrib.get('parameter_name')
            store_as = element.attrib.get('store_as', None)
            return GetParameter(
                self.node, node_name, parameter_name,
                self.test_suite.stored_parameters, store_as=store_as)
        elif elem_type == 'SetParameter':
            node_name = element.attrib.get('node_name')
            parameter_name = element.attrib.get('parameter_name')
            parameter_value = element.attrib.get('parameter_value')
            store_as = element.attrib.get('store_as', None)
            return SetParameter(
                self.node, node_name, parameter_name, parameter_value,
                self.test_suite.stored_parameters, store_as=store_as)
        elif elem_type == 'LoadParametersFromCSVFile':
            path = element.attrib['path']
            separator = element.attrib.get('separator', '\t')

            return LoadParametersFromCSVFile(
                self.node, path, separator, self.test_suite.stored_parameters)
        elif elem_type == 'LoadParametersFromXMLFile':
            path = element.attrib['path']

            return LoadParametersFromXMLFile(
                self.node, path, self.test_suite.stored_parameters)
        elif elem_type == 'Run':
            try:
                arguments = element.attrib['arguments']
            except KeyError:
                arguments = None
            try:
                blocking = element.attrib['blocking']
                blocking = get_bool(blocking)
            except KeyError:
                blocking = False
            return Run(
                self.node,
                element.attrib['package_name'],
                element.attrib['executable_name'],
                arguments,
                blocking)
        elif elem_type == 'ServiceCall':
            use_substitution = False
            if len(element) > 0:
                if element[0].tag == with_ns('Message'):
                    parameters = self.get_message(element[0])
                elif element[0].tag == with_ns('MessageYaml'):
                    use_substitution = get_bool(
                        element[0].attrib.get(
                            'use_substitution', False))
                    parameters = yaml.safe_load(element[0].text)
                else:
                    raise Exception(
                        'Unidenfied message specification {}'.format(
                            element[0].tag))
            else:
                parameters = {}
            save_result = get_bool(element.attrib.get('save_result', 'false'))
            return ServiceCall(
                self.node, element.attrib['service'], element.attrib['type'],
                parameters, self.test_suite.stored_parameters,
                use_substitution, save_result=save_result)
        elif elem_type == 'Sleep':
            return Sleep(
                self.node,
                float(element.attrib['duration']),
                element.attrib.get('unit', 'second'))
        else:
            raise Exception('Unidenfied step {}'.format(elem_type))

    def get_message(self, element):
        """Parse Message element."""
        parameters = element.getchildren()
        if len(parameters) > 0:
            params = self.get_param_from_elements(parameters)
            return params
        else:
            return None

    def get_param_from_elements(self, elements):
        """Parse Parameter element."""
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

        def get_value(parameter):
            return self.test_suite.stored_parameters[parameter]

        def get_value_numeric(parameter):
            return get_numeric_value(get_value(parameter))

        def get_value_bool(parameter):
            return get_boolean_value(get_value(parameter))

        def get_value_string(parameter):
            return get_string_value(get_value(parameter))

        get_partial_numeric = None
        get_partial_bool = None
        get_partial_string = None
        if 'param' in attr:
            get_partial_numeric = partial(get_value_numeric, attr['param'])
            get_partial_bool = partial(get_value_bool, attr['param'])
            get_partial_string = partial(get_value_string, attr['param'])

        def need_value_or_parameter(attr, val_case, param_case):
            if 'value' in attr:
                return val_case()
            elif 'parameter' in attr:
                return param_case()
            else:
                raise RuntimeError(
                    'Condition requires either value or parameter.')

        if elem_type == 'Binary':
            return need_value_or_parameter(
                attr,
                lambda: BinaryCondition(
                    value=get_bool(attr['value']), name=name),
                lambda: BinaryCondition(
                    retrieve_value=get_partial_bool, name=name))
        elif elem_type == 'GreaterThan':
            return need_value_or_parameter(
                attr,
                lambda: GreaterThanCondition(
                    value=float(attr['value']), name=name),
                lambda: GreaterThanCondition(
                    retrieve_value=get_partial_numeric, name=name))
        elif elem_type == 'GreaterThanOrEqualTo':
            return need_value_or_parameter(
                attr,
                lambda: GreaterThanOrEqualToCondition(
                    value=float(attr['value']), name=name),
                lambda: GreaterThanOrEqualToCondition(
                    retrieve_value=get_partial_numeric, name=name))
            return GreaterThanOrEqualToCondition(float(attr['value']), name)
        elif elem_type == 'NotEqualTo':
            return need_value_or_parameter(
                attr,
                lambda: NotEqualToCondition(
                    value=float(attr['value']),
                    epsilon=float(attr['epsilon']),
                    name=name),
                lambda: NotEqualToCondition(
                    retrieve_value=get_partial_numeric,
                    epsilon=float(attr['epsilon']),
                    name=name))
            return NotEqualToCondition(float(attr['value']), name)
        elif elem_type == 'EqualTo':
            return need_value_or_parameter(
                attr,
                lambda: EqualToCondition(
                    value=float(attr['value']),
                    epsilon=float(attr['epsilon']),
                    name=name),
                lambda: EqualToCondition(
                    retrieve_value=get_partial_numeric,
                    epsilon=float(attr['epsilon']),
                    name=name))
        elif elem_type == 'LessThanOrEqualTo':
            return need_value_or_parameter(
                attr,
                lambda: LessThanOrEqualToCondition(
                    value=float(attr['value']), name=name),
                lambda: LessThanOrEqualToCondition(
                    retrieve_value=get_partial_numeric,
                    name=name))
        elif elem_type == 'LessThan':
            return need_value_or_parameter(
                attr,
                lambda: LessThanCondition(
                    value=float(attr['value']), name=name),
                lambda: LessThanCondition(
                    retrieve_value=get_partial_numeric,
                    name=name))
        elif elem_type == 'UpperLimit':
            return need_value_or_parameter(
                attr,
                lambda: UpperLimitCondition(
                    upper_limit_is_inclusive=get_bool(attr['inclusive']),
                    upper_limit_value=float(attr['value']),
                    name=name),
                lambda: UpperLimitCondition(
                    upper_limit_is_inclusive=get_bool(attr['inclusive']),
                    retrieve_upper_limit=get_partial_numeric,
                    name=name))
        elif elem_type == 'LowerLimit':
            return need_value_or_parameter(
                attr,
                lambda: LowerLimitCondition(
                    lower_limit_is_inclusive=get_bool(attr['inclusive']),
                    lower_limit_value=float(attr['value']),
                    name=name),
                lambda: LowerLimitCondition(
                    lower_limit_is_inclusive=get_bool(attr['inclusive']),
                    retrieve_lower_limit=get_partial_numeric,
                    name=name))
        elif elem_type == 'BothLimits':
            llv = None
            if 'lower_limit_value' in attr:
                llv = float(attr['lower_limit_value'])

            ulv = None
            if 'upper_limit_value' in attr:
                ulv = float(attr['upper_limit_value'])

            llp = None
            if 'lower_limit_parameter' in attr:
                llp = partial(get_value_numeric, attr['lower_limit_parameter'])

            ulp = None
            if 'upper_limit_parameter' in attr:
                ulp = partial(get_value_numeric, attr['upper_limit_parameter'])

            return BothLimitsCondition(
                attr.get('lower_limit_inclusive', 'true'),
                attr.get('upper_limit_inclusive', 'true'),
                llv,
                ulv,
                llp,
                ulp,
                name)
        elif elem_type == 'StringEquals':
            return need_value_or_parameter(
                attr,
                lambda: StringEqualsCondition(attr['value'], name=name),
                lambda: StringEqualsCondition(get_partial_string, name=name))
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
                                    elem.attrib.get('field', None))
        elif elem_type == 'MessagesEvaluator':
            try:
                negate = elem.attrib['negate']
                negate = get_bool(negate)
            except KeyError:
                negate = False

            try:
                occurrence = elem.attrib['occurrence']
                occurrence = get_occurrence(occurrence)
            except KeyError:
                occurrence = Occurrence.LAST

            return MessagesEvaluator(self.node, elem.attrib['topic'],
                                     elem.attrib['type'],
                                     occurrence,
                                     negate,
                                     elem.attrib.get('field', None))
        elif elem_type == 'ExecutionReturnedEvaluator':
            field = elem.attrib.get('field', None)
            return ExecutionReturnedEvaluator(self.current_test_case, field)
        elif elem_type == 'NumericMessageEvaluator':
            return NumericMessageEvaluator(
                self.node, elem.attrib['topic'], elem.attrib['type'],
                elem.attrib.get('field', None))
        elif elem_type == 'NumericMessagesEvaluator':
            try:
                negate = elem.attrib['negate']
                negate = get_bool(negate)
            except KeyError:
                negate = False

            try:
                occurrence = elem.attrib['occurrence']
                occurrence = get_occurrence(occurrence)
            except KeyError:
                occurrence = Occurrence.LAST

            return NumericMessagesEvaluator(
                self.node, elem.attrib['topic'], elem.attrib['type'],
                occurrence,
                negate,
                elem.attrib.get('field', None))
        else:
            raise ValueError('Unexpected type {}'.format(elem_type))
