import unittest
from unittest.mock import Mock, patch

from opendbc.car.can_definitions import CanData
from opendbc.car.disable_ecu import disable_ecu


class TestDisableEcu(unittest.TestCase):
  @staticmethod
  def queries(communication_response):
    session_query = Mock()
    session_query.get_data.return_value = {(0x787, None): b''}
    communication_query = Mock()
    communication_query.get_data.return_value = communication_response
    return session_query, communication_query

  def test_captured_conditions_not_correct_response_fails_closed(self):
    pending = []
    sent = []

    def can_send(msgs):
      for msg in msgs:
        dat = bytes(msg.dat)
        sent.append(dat)
        if dat[1:3] == b'\x10\x03':
          pending.append(CanData(0x78F, bytes.fromhex("065003003201c200"), 2))
        elif dat[1:4] == b'\x28\x03\x01':
          pending.append(CanData(0x78F, bytes.fromhex("037f282200000000"), 2))

    def can_recv(wait_for_one=False):
      return [[pending.pop(0)]] if pending else []

    disabled = disable_ecu(can_recv, can_send, bus=2, addr=0x787, com_cont_req=b'\x28\x03\x01', timeout=0.2, retry=1)

    self.assertFalse(disabled)
    self.assertTrue(any(dat[1:4] == b'\x28\x03\x01' for dat in sent))

  @patch("opendbc.car.disable_ecu.IsoTpParallelQuery")
  def test_unsuppressed_communication_control_requires_positive_response(self, query_cls):
    session_query, communication_query = self.queries({})
    query_cls.side_effect = [session_query, communication_query]

    disabled = disable_ecu(Mock(), Mock(), bus=2, addr=0x787, com_cont_req=b'\x28\x03\x01', timeout=0.2, retry=1)

    self.assertFalse(disabled)
    communication_query.get_data.assert_called_once_with(0.2)
    self.assertEqual(query_cls.call_args_list[1].args[5], [b'\x68\x03'])

  @patch("opendbc.car.disable_ecu.IsoTpParallelQuery")
  def test_unsuppressed_communication_control_accepts_positive_response(self, query_cls):
    session_query, communication_query = self.queries({(0x787, None): b''})
    query_cls.side_effect = [session_query, communication_query]

    disabled = disable_ecu(Mock(), Mock(), bus=2, addr=0x787, com_cont_req=b'\x28\x03\x01', timeout=0.2, retry=1)

    self.assertTrue(disabled)
    communication_query.get_data.assert_called_once_with(0.2)

  @patch("opendbc.car.disable_ecu.IsoTpParallelQuery")
  def test_suppressed_communication_control_remains_fire_and_forget(self, query_cls):
    session_query, communication_query = self.queries({})
    query_cls.side_effect = [session_query, communication_query]

    disabled = disable_ecu(Mock(), Mock(), bus=2, addr=0x787, com_cont_req=b'\x28\x83\x01', timeout=0.2, retry=1)

    self.assertTrue(disabled)
    communication_query.get_data.assert_called_once_with(0)
    self.assertEqual(query_cls.call_args_list[1].args[5], [b''])


if __name__ == "__main__":
  unittest.main()
