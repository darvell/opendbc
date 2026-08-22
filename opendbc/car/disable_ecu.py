from opendbc.car import uds
from opendbc.car.carlog import carlog
from opendbc.car.isotp_parallel_query import IsoTpParallelQuery

EXT_DIAG_REQUEST = b'\x10\x03'
EXT_DIAG_RESPONSE = b'\x50\x03'

COM_CONT_RESPONSE = bytes([0x40 + uds.SERVICE_TYPE.COMMUNICATION_CONTROL])


def disable_ecu(can_recv, can_send, bus=0, addr=0x7d0, sub_addr=None, com_cont_req=b'\x28\x83\x01', timeout=0.1, retry=10):
  """Silence an ECU by disabling sending and receiving messages using UDS 0x28.
  The ECU will stay silent as long as openpilot keeps sending Tester Present.

  This is used to disable the radar in some cars. Openpilot will emulate the radar.
  WARNING: THIS DISABLES AEB!"""
  carlog.warning(f"ecu disable {hex(addr), sub_addr} ...")

  for i in range(retry):
    try:
      query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr, sub_addr)], [EXT_DIAG_REQUEST], [EXT_DIAG_RESPONSE])

      for _, _ in query.get_data(timeout).items():
        carlog.warning("communication control disable tx/rx ...")

        control_type = com_cont_req[1] & 0x7F
        suppress_response = bool(com_cont_req[1] & 0x80)
        expected_response = b'' if suppress_response else COM_CONT_RESPONSE + bytes([control_type])
        query = IsoTpParallelQuery(can_send, can_recv, bus, [(addr, sub_addr)], [com_cont_req], [expected_response])
        response = query.get_data(0 if suppress_response else timeout)
        if suppress_response or response:
          carlog.warning("ecu disabled")
          return True

        carlog.error("communication control rejected or timed out")

    except Exception:
      carlog.exception("ecu disable exception")

    carlog.error(f"ecu disable retry ({i + 1}) ...")
  carlog.error("ecu disable failed")
  return False
