import copy
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.subaru.values import DBC, CanBus, SubaruFlags
from opendbc.car import CanSignalRateCalculator

from opendbc.sunnypilot.car.subaru.mads import MadsCarState
from opendbc.sunnypilot.car.subaru.stop_and_go import SnGCarState


class CarState(CarStateBase, MadsCarState, SnGCarState):
  BSM_STALE_FRAMES = 20
  BSM_APPROACHING_HOLD_FRAMES = 100
  SONAR_STALE_FRAMES = 20

  def __init__(self, CP, CP_SP):
    CarStateBase.__init__(self, CP, CP_SP)
    MadsCarState.__init__(self, CP, CP_SP)
    SnGCarState.__init__(self, CP, CP_SP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    self.shifter_values = can_define.dv["Transmission"]["Gear"]

    self.angle_rate_calulator = CanSignalRateCalculator(50)
    self.bsm_stale_frames = self.BSM_STALE_FRAMES + 1
    self.left_approaching_hold_frames = 0
    self.right_approaching_hold_frames = 0
    self.sonar_stale_frames = self.SONAR_STALE_FRAMES + 1

  @staticmethod
  def _feature_state(available, disabled):
    # Matches the stock camera's state mapping recovered from cs_eyesight.c.
    return 1 if available else (3 if disabled else 2)

  def update_subaru_surroundings(self, cp, cp_cam, ret, ret_sp):
    if self.CP.enableBsm:
      bsm = cp.vl["BSD_RCTA"]
      bsm_updated = len(cp.vl_all["BSD_RCTA"]["L_ADJACENT"]) > 0
      self.bsm_stale_frames = 0 if bsm_updated else self.bsm_stale_frames + 1

      left_adjacent = bsm["L_ADJACENT"] == 1
      left_approaching = bsm["L_APPROACHING"] == 1
      right_adjacent = bsm["R_ADJACENT"] == 1
      right_approaching = bsm["R_APPROACHING"] == 1

      self.left_approaching_hold_frames = self.BSM_APPROACHING_HOLD_FRAMES if left_approaching else \
        max(0, self.left_approaching_hold_frames - 1)
      self.right_approaching_hold_frames = self.BSM_APPROACHING_HOLD_FRAMES if right_approaching else \
        max(0, self.right_approaching_hold_frames - 1)

      ret_sp.blindSpotLeftAdjacent = left_adjacent
      ret_sp.blindSpotLeftApproaching = left_approaching
      ret_sp.blindSpotRightAdjacent = right_adjacent
      ret_sp.blindSpotRightApproaching = right_approaching
      ret_sp.blindSpotMonitorValid = self.bsm_stale_frames <= self.BSM_STALE_FRAMES
      ret_sp.rearCrossTrafficLeft = bsm["L_RCTA"] == 1
      ret_sp.rearCrossTrafficRight = bsm["R_RCTA"] == 1

      # Treat stale BSM data as occupied. Positive detection is a veto; missing data is never permission.
      ret.leftBlindspot = left_adjacent or left_approaching or self.left_approaching_hold_frames > 0 or not ret_sp.blindSpotMonitorValid
      ret.rightBlindspot = right_adjacent or right_approaching or self.right_approaching_hold_frames > 0 or not ret_sp.blindSpotMonitorValid

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      return

    sonar = cp_cam.vl["ES_STATIC_1"]
    sonar_updated = len(cp_cam.vl_all["ES_STATIC_1"]["Sonar_Warn_Left"]) > 0
    self.sonar_stale_frames = 0 if sonar_updated else self.sonar_stale_frames + 1

    ret_sp.rearSonarAvailable = bool(sonar["Sonar_Available"] and not sonar["Sonar_Off"])
    ret_sp.rearSonarValid = bool(ret_sp.rearSonarAvailable and self.sonar_stale_frames <= self.SONAR_STALE_FRAMES and
                                 not sonar["Sonar_System_Halt"] and not sonar["Sonar_System_Fail"])
    ret_sp.rearSonarLeft = int(sonar["Sonar_Warn_Left"])
    ret_sp.rearSonarCenter = int(sonar["Sonar_Warn_Center"] if sonar["Sonar_Warn_Center"] < 5 else 0)
    ret_sp.rearSonarRight = int(sonar["Sonar_Warn_Right"])
    ret_sp.rearSonarSystemHalted = bool(sonar["Sonar_System_Halt"])
    ret_sp.rearSonarSystemFaulted = bool(sonar["Sonar_System_Fail"])
    ret_sp.rearAutomaticBrakingAvailable = bool(sonar["RAB_Available"] and not sonar["RAB_Off"])
    ret_sp.rearAutomaticBrakingState = self._feature_state(sonar["RAB_Available"], sonar["RAB_Off"])
    ret_sp.rearAutomaticBrakingAlert = int(sonar["RAB_Warn_State"] if sonar["RAB_Warn_State"] < 3 else 0)

  def update(self, can_parsers) -> tuple[structs.CarState, structs.CarStateSP]:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]
    cp_alt = can_parsers[Bus.alt]
    ret = structs.CarState()
    ret_sp = structs.CarStateSP()

    throttle_msg = cp.vl["Throttle"] if not (self.CP.flags & SubaruFlags.HYBRID) else cp_alt.vl["Throttle_Hybrid"]
    ret.gasPressed = throttle_msg["Throttle_Pedal"] > 1e-5
    if self.CP.flags & SubaruFlags.PREGLOBAL:
      ret.brakePressed = cp.vl["Brake_Pedal"]["Brake_Pedal"] > 0
    else:
      cp_brakes = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
      ret.brakePressed = cp_brakes.vl["Brake_Status"]["Brake"] == 1

    cp_es_distance = cp_alt if self.CP.flags & (SubaruFlags.GLOBAL_GEN2 | SubaruFlags.HYBRID) else cp_cam
    if not (self.CP.flags & SubaruFlags.HYBRID):
      eyesight_fault = bool(cp_es_distance.vl["ES_Distance"]["Cruise_Fault"])

      # if openpilot is controlling long, an eyesight fault is a non-critical fault. otherwise it's an ACC fault
      if self.CP.openpilotLongitudinalControl:
        ret.carFaultedNonCritical = eyesight_fault
      else:
        ret.accFaulted = eyesight_fault

    cp_wheels = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
    self.parse_wheel_speeds(ret,
      cp_wheels.vl["Wheel_Speeds"]["FL"],
      cp_wheels.vl["Wheel_Speeds"]["FR"],
      cp_wheels.vl["Wheel_Speeds"]["RL"],
      cp_wheels.vl["Wheel_Speeds"]["RR"],
    )
    ret.standstill = ret.vEgoRaw == 0

    # continuous blinker signals for assisted lane change
    ret.leftBlinker, ret.rightBlinker = self.update_blinker_from_lamp(50, cp.vl["Dashlights"]["LEFT_BLINKER"],
                                                                      cp.vl["Dashlights"]["RIGHT_BLINKER"])

    self.update_subaru_surroundings(cp, cp_cam, ret, ret_sp)

    cp_transmission = cp_alt if self.CP.flags & SubaruFlags.HYBRID else cp
    can_gear = int(cp_transmission.vl["Transmission"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    if not (self.CP.flags & SubaruFlags.LKAS_ANGLE):
      ret.steeringAngleDeg = cp.vl["Steering_Torque"]["Steering_Angle"]
      steering_updated = len(cp.vl_all["Steering_Torque"]["Steering_Angle"]) > 0
    else:
      # Steering_Torque->Steering_Angle exists on SUBARU_FORESTER_2022, SUBARU_OUTBACK_2023, SUBARU_ASCENT_2023 where
      # it is identical to Steering_2's signal. However, it is always zero on newer LKAS_ANGLE cars
      # such as 2024+ Crosstrek, 2023+ Ascent, etc. Use a universal signal for LKAS_ANGLE cars.
      ret.steeringAngleDeg = cp.vl["Steering_2"]["Steering_Angle"]
      steering_updated = len(cp.vl_all["Steering_2"]["Steering_Angle"]) > 0

    if not (self.CP.flags & SubaruFlags.PREGLOBAL):
      # ideally we get this from the car, but unclear if it exists. diagnostic software doesn't even have it
      ret.steeringRateDeg = self.angle_rate_calulator.update(ret.steeringAngleDeg, steering_updated)

    ret.steeringTorque = cp.vl["Steering_Torque"]["Steer_Torque_Sensor"]
    ret.steeringTorqueEps = cp.vl["Steering_Torque"]["Steer_Torque_Output"]

    steer_threshold = 75 if self.CP.flags & SubaruFlags.PREGLOBAL else 80
    ret.steeringPressed = abs(ret.steeringTorque) > steer_threshold

    cp_cruise = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp
    cp_es_brake = cp_alt if self.CP.flags & SubaruFlags.GLOBAL_GEN2 else cp_cam

    if self.CP.flags & (SubaruFlags.HYBRID | SubaruFlags.LKAS_ANGLE):
      # ES_DashStatus->Cruise_Activated_Dash is likely intended for the dash display only, as it falls
      # during user gas override and at standstill. ES_Status is missing on hybrid, so we use ES_Brake instead

      # TODO: ES_Brake->Cruise_Activated has been seen staying high when Crosstrek 2025 angle LKAS user pressed
      #  brake while engaged at a stop. ES_Status and ES_DashStatus->Signal7 correctly fell, but is either missing or
      #  always zero on hybrids. Probably need to split angle & hybrid. 0x27 and 0x225 on hybrids may work for them.
      ret.cruiseState.enabled = cp_es_brake.vl["ES_Brake"]['Cruise_Activated'] != 0
      ret.cruiseState.available = cp_cam.vl["ES_DashStatus"]['Cruise_On'] != 0
    else:
      ret.cruiseState.enabled = cp_cruise.vl["CruiseControl"]["Cruise_Activated"] != 0
      ret.cruiseState.available = cp_cruise.vl["CruiseControl"]["Cruise_On"] != 0
    ret.cruiseState.speed = cp_cam.vl["ES_DashStatus"]["Cruise_Set_Speed"] * CV.KPH_TO_MS

    if (self.CP.flags & SubaruFlags.PREGLOBAL and cp.vl["Dash_State2"]["UNITS"] == 1) or \
       (not (self.CP.flags & SubaruFlags.PREGLOBAL) and cp.vl["Dashlights"]["UNITS"] == 1):
      ret.cruiseState.speed *= CV.MPH_TO_KPH

    ret.seatbeltUnlatched = cp.vl["Dashlights"]["SEATBELT_FL"] == 1
    ret.doorOpen = any([cp.vl["BodyInfo"]["DOOR_OPEN_RR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_RL"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FR"],
                        cp.vl["BodyInfo"]["DOOR_OPEN_FL"]])
    ret.steerFaultPermanent = cp.vl["Steering_Torque"]["Steer_Error_1"] == 1

    if self.CP.flags & SubaruFlags.PREGLOBAL:
      self.cruise_button = cp_cam.vl["ES_Distance"]["Cruise_Button"]
      self.ready = not cp_cam.vl["ES_DashStatus"]["Not_Ready_Startup"]
    else:
      ret.steerFaultTemporary = cp.vl["Steering_Torque"]["Steer_Warning"] == 1
      ret.cruiseState.nonAdaptive = cp_cam.vl["ES_DashStatus"]["Conventional_Cruise"] == 1
      ret.cruiseState.standstill = cp_cam.vl["ES_DashStatus"]["Cruise_State"] == 3
      ret.stockFcw = (cp_cam.vl["ES_LKAS_State"]["LKAS_Alert"] == 1) or \
                     (cp_cam.vl["ES_LKAS_State"]["LKAS_Alert"] == 2)

      self.es_lkas_state_msg = copy.copy(cp_cam.vl["ES_LKAS_State"])
      self.es_brake_msg = copy.copy(cp_es_brake.vl["ES_Brake"])

      # TODO: Hybrid cars don't have ES_Distance, need a replacement
      if not (self.CP.flags & SubaruFlags.HYBRID):
        # 8 is known AEB, there are a few other values related to AEB we ignore
        ret.stockAeb = (cp_es_distance.vl["ES_Brake"]["AEB_Status"] == 8) and \
                       (cp_es_distance.vl["ES_Brake"]["Brake_Pressure"] != 0)

        self.es_status_msg = copy.copy(cp_es_brake.vl["ES_Status"])
        self.cruise_control_msg = copy.copy(cp_cruise.vl["CruiseControl"])

    if not (self.CP.flags & SubaruFlags.HYBRID):
      self.es_distance_msg = copy.copy(cp_es_distance.vl["ES_Distance"])

    self.es_dashstatus_msg = copy.copy(cp_cam.vl["ES_DashStatus"])
    if self.CP.flags & SubaruFlags.SEND_INFOTAINMENT:
      self.es_infotainment_msg = copy.copy(cp_cam.vl["ES_Infotainment"])

    MadsCarState.update_mads(self, ret, can_parsers)
    SnGCarState.update(self, ret, can_parsers)

    return ret, ret_sp

  @staticmethod
  def get_can_parsers(CP, CP_SP):
    return {
      Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.main),
      Bus.cam: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.camera),
      Bus.alt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CanBus.alt)
    }
