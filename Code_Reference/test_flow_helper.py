"""Testing file of the functions in FlowHelper."""

import unittest
from demand_modelling.daily_flow.flow import FlowHelper


class TestFlowHelper(unittest.TestCase):
    """Test cases of FlowHelper."""
    def setUp(self):
        self.routes_file = "fixtures/hello.rou.xml"
        self.output_file = "fixtures/flow.rou.xml"

    def test_remove_all(self):
        flows = FlowHelper(self.routes_file)
        flows.remove_all()
        self.assertEqual(len(flows.tree.getroot().getchildren()), 4)

    def test_add_flow_by_dict(self):
        flow_dict_list = [{'id': 'flow_1', 'type': 'Car',
                           'from': '1i', 'to': '2o',
                           'begin': '20', 'end': '100',
                           'vehsPerHour': '100', 'departLane': 'random'},
                          {'id': 'flow_2', 'type': 'Car',
                           'from': '2i', 'to': '1o',
                           'begin': '0', 'end': '100',
                           'vehsPerHour': '100', 'departLane': 'random'}]
        flows = FlowHelper(self.routes_file)
        flows.remove_all()
        flows.add_flow_by_dict(flow_dict_list)
        self.assertEqual(len(flows.tree.getroot().getchildren()), 6)

    def test_add_flow(self):
        flows = FlowHelper(self.routes_file)
        flows.add_flow('flow_add', 'Car', '1i', '2o', 30, 200, 100)
        flows.write_xml(self.output_file)
        self.assertEqual(len(flows.tree.getroot().getchildren()), 8)

    def test_add_sin_flow(self):
        flows = FlowHelper(self.routes_file)
        flows.remove_all()
        flows.add_sin_flow('flow_add', 'Car', '1i', '2o', 0, 3600, 20, min_volume=100, max_volume=1000)
        flows.write_xml(self.output_file)

    def test_add_linear_flow(self):
        flows = FlowHelper(self.routes_file)
        flows.remove_all()
        flows.add_linear_flow('flow_linear', 'Car', '1i', '2o', 0, 1800, 10, begin_volume=800, end_volume=200)
        flows.write_xml(self.output_file)
