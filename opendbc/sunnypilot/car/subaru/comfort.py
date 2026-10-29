"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.

Start-of-drive comfort settings the Gen2 Crosstrek forgets every ignition cycle: Auto Vehicle Hold
on, and the auto start-stop engine shutoff off. Measured and first implemented in
NylonDiamond/opendbc (b89a5a61, d2befaf8, 06304f73, 781091ab).
"""
import copy
from enum import StrEnum

from opendbc.can.parser import CANParser
from opendbc.car import Bus, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.subaru.values import CAR, CanBus
from opendbc.sunnypilot.car.subaru.values_ext import SubaruFlagsSP

# Each request is made at the start of a drive, only while the car is in the wrong state, repeated
# until the car answers, then latched done for the drive. Frames are 100 Hz.
COMFORT_SETTLE_FRAMES = 500     # 5 s, long enough for every message compared against to arrive
COMFORT_DEADLINE_FRAMES = 6000  # 60 s, after which this is no longer a start-of-drive action
COMFORT_BURST_LEN = 8           # AVH request frames per attempt, matching a real touchscreen press
COMFORT_BURST_STEP = 5          # 50 ms apart, also matching
COMFORT_BURST_SETTLE = 200      # 2 s between attempts; the car answers in about 90 ms

# The start-stop button is edge triggered and the car keeps sending its own Dashlights at 10 Hz with
# the button bit clear. Each of those reads as a release, so a press spanning several of them lands as
# several presses and toggles the setting back. Keep the press inside one 100 ms gap, starting right
# after one of the car's frames, and check the result instead of assuming it.
COMFORT_PRESS_FRAMES = 2        # 20 ms of button bit
COMFORT_PRESS_SETTLE = 40       # 400 ms for the car to answer before judging the press
COMFORT_PRESS_TRIES = 5         # give up rather than keep toggling

AVH_REQUEST_ON = 2
STOP_START_SWITCHED_OFF = 3

# Steady-state Comfort_Control bytes 3-7 seen with the engine running. Byte 7 differs between cars
# (0x00 on NylonDiamond's 2024 Wilderness, 0x0A on ours), so the request copies the car's own frame.
COMFORT_CONTROL_STATIC = ("Signal1", "Signal2", "Signal3", "Signal4", "Signal5", "Signal6")
DASHLIGHTS_COPIED = ("Signal1", "Signal2", "UNITS", "Signal3", "ICY_ROAD", "Signal4", "Signal5", "SEATBELT_FL",
                     "Signal6", "LEFT_BLINKER", "RIGHT_BLINKER", "Signal7", "Signal8", "Signal9")


def comfort_supported(car_fingerprint: str) -> bool:
  # Only measured on the 2024 Crosstrek (ours and NylonDiamond's). Panda additionally refuses the requests
  # anywhere but the alt bus of a Gen2 angle-LKAS car under stock longitudinal.
  return car_fingerprint == CAR.SUBARU_CROSSTREK_2025


def create_comfort_control(packer, counter: int, comfort_control_msg: dict[str, float]) -> CanData:
  values = {s: comfort_control_msg[s] for s in COMFORT_CONTROL_STATIC}
  values["COUNTER"] = counter % 0x10
  values["AVH_REQUEST"] = AVH_REQUEST_ON
  return packer.make_can_msg("Comfort_Control", CanBus.alt, values)


def create_stop_start_press(packer, counter: int, dashlights_msg: dict[str, float]) -> CanData:
  # Everything but the button bit is copied from the car's own frame so blinker and seatbelt stay true.
  values = {s: dashlights_msg[s] for s in DASHLIGHTS_COPIED}
  values["COUNTER"] = counter % 0x10
  values["STOP_START"] = 1
  return packer.make_can_msg("Dashlights", CanBus.alt, values)


class ComfortCarState:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    self.CP = CP
    self.CP_SP = CP_SP
    # All stay None until the car has actually sent them, so a missing message never looks like
    # a car sitting in the wrong state.
    self.dashlights_msg: dict[str, float] | None = None
    self.comfort_control_msg: dict[str, float] | None = None
    self.avh_active: bool | None = None
    self.stop_start_disabled: bool | None = None

  @staticmethod
  def comfort_messages(CP: structs.CarParams) -> list[tuple[str, float]]:
    # Optional: a car that never sends these must not invalidate CAN.
    if not comfort_supported(CP.carFingerprint):
      return []
    return [("Comfort_Control", float("nan")), ("Comfort_Status", float("nan")), ("Engine_Stop_Start", float("nan"))]

  def update_comfort(self, can_parsers: dict[StrEnum, CANParser]) -> None:
    if not comfort_supported(self.CP.carFingerprint):
      return

    cp = can_parsers[Bus.pt]
    if cp.ts_nanos["Dashlights"]["STOP_START"] != 0:
      self.dashlights_msg = copy.copy(cp.vl["Dashlights"])
    if cp.ts_nanos["Comfort_Control"]["AVH_REQUEST"] != 0:
      self.comfort_control_msg = copy.copy(cp.vl["Comfort_Control"])
    if cp.ts_nanos["Comfort_Status"]["AVH_ACTIVE"] != 0:
      self.avh_active = cp.vl["Comfort_Status"]["AVH_ACTIVE"] == 1
    if cp.ts_nanos["Engine_Stop_Start"]["STOP_START_STATE"] != 0:
      self.stop_start_disabled = cp.vl["Engine_Stop_Start"]["STOP_START_STATE"] == STOP_START_SWITCHED_OFF


class ComfortCarController:
  def __init__(self, CP: structs.CarParams, CP_SP: structs.CarParamsSP):
    supported = comfort_supported(CP.carFingerprint)
    self.avh_wanted = supported and bool(CP_SP.flags & SubaruFlagsSP.AUTO_VEHICLE_HOLD)
    self.stop_start_wanted = supported and bool(CP_SP.flags & SubaruFlagsSP.STOP_START_OFF)

    self.avh_burst_left = 0
    self.avh_counter = 0
    self.avh_wait = 0
    self.avh_done = False
    self.stop_start_press_left = 0
    self.stop_start_wait = 0
    self.stop_start_tries = 0
    self.stop_start_counter = 0
    self.stop_start_seen_counter: int | None = None
    self.stop_start_done = False

  def create_comfort(self, packer, CS, frame: int) -> list[CanData]:
    """Only ever push in the direction the driver had to by hand: AVH on, start-stop off. Once the car is
    in that state the request latches done, so a later touchscreen press always wins."""
    can_sends: list[CanData] = []
    if not (self.avh_wanted or self.stop_start_wanted) or frame < COMFORT_SETTLE_FRAMES:
      return can_sends
    if frame > COMFORT_DEADLINE_FRAMES:
      # Surprising the driver with this mid-drive is worse than not doing it at all.
      self.avh_done = self.stop_start_done = True
      return can_sends

    # *** Auto Vehicle Hold ***
    # Not gated on standstill: the request arms the hold, it does not apply the brakes, and the car only
    # holds after the driver has braked to a stop. openpilot is often still starting as the driver pulls
    # away, so a standstill gate mostly just misses the drive.
    if self.avh_wanted and not self.avh_done and CS.comfort_control_msg is not None:
      if self.avh_burst_left > 0:
        # A real press keeps going after the car answers, so finish the burst.
        if frame % COMFORT_BURST_STEP == 0:
          self.avh_counter += 1
          can_sends.append(create_comfort_control(packer, self.avh_counter, CS.comfort_control_msg))
          self.avh_burst_left -= 1
          if self.avh_burst_left == 0:
            self.avh_wait = COMFORT_BURST_SETTLE
      elif CS.avh_active:
        self.avh_done = True
      elif self.avh_wait > 0:
        self.avh_wait -= 1
      elif CS.avh_active is False:
        self.avh_burst_left = COMFORT_BURST_LEN

    # *** auto start-stop engine shutoff ***
    # A toggling button, so only press it while the shutoff is still armed.
    if self.stop_start_wanted and not self.stop_start_done and CS.dashlights_msg is not None:
      # The car's counter advances once per Dashlights frame; a change means one just landed and the gap
      # before the next one is ours.
      car_counter = int(CS.dashlights_msg["COUNTER"])
      fresh_frame = self.stop_start_seen_counter is not None and car_counter != self.stop_start_seen_counter
      self.stop_start_seen_counter = car_counter

      if CS.stop_start_disabled:
        self.stop_start_done = True
      elif self.stop_start_press_left > 0:
        self.stop_start_counter += 1
        can_sends.append(create_stop_start_press(packer, self.stop_start_counter, CS.dashlights_msg))
        self.stop_start_press_left -= 1
        if self.stop_start_press_left == 0:
          self.stop_start_wait = COMFORT_PRESS_SETTLE
      elif self.stop_start_wait > 0:
        self.stop_start_wait -= 1
      elif self.stop_start_tries >= COMFORT_PRESS_TRIES:
        # This car does not behave as measured; stop rather than keep toggling it.
        self.stop_start_done = True
      elif CS.stop_start_disabled is False and fresh_frame:
        self.stop_start_counter = car_counter
        self.stop_start_tries += 1
        self.stop_start_press_left = COMFORT_PRESS_FRAMES

    return can_sends
