from dataclasses import dataclass, field
from enum import Enum, IntFlag

from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, CarSpecs, DbcDict, PlatformConfig, Platforms, uds
from opendbc.car.lateral import AngleSteeringLimitsVM, ISO_LATERAL_ACCEL
from opendbc.car.structs import CarParams
from opendbc.car.docs_definitions import CarFootnote, CarHarness, CarDocs, CarParts, Column
from opendbc.car.fw_query_definitions import FwQueryConfig, Request, StdQueries, p16

Ecu = CarParams.Ecu


# Safety has no roll estimate, so permit the ISO 11270 lateral limits plus a typical banked-road allowance.
AVERAGE_ROAD_ROLL = 0.06


class CarControllerParams:
  # The EPS faults above roughly 200 degrees while LKAS is actively requesting steering.
  ACTIVE_ANGLE_MAX = 190
  ANGLE_LIMITS: AngleSteeringLimitsVM = AngleSteeringLimitsVM(
    STEER_ANGLE_MAX=190,
    MAX_LATERAL_ACCEL=ISO_LATERAL_ACCEL + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),
    MAX_LATERAL_JERK=3.0 + (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL),
    MAX_ANGLE_RATE=5,
  )
  # With LKAS_Request clear, the command must still echo the measured full-lock angle.
  ANGLE_INACTIVE_MAX = 650

  def __init__(self, CP):
    self.STEER_STEP = 2                # how often we update the steer cmd
    self.STEER_DELTA_UP = 50           # torque increase per refresh, 0.8s to max
    self.STEER_DELTA_DOWN = 70         # torque decrease per refresh
    self.STEER_DRIVER_ALLOWANCE = 60   # allowed driver torque before start limiting
    self.STEER_DRIVER_MULTIPLIER = 50  # weight driver torque heavily
    self.STEER_DRIVER_FACTOR = 1       # from dbc

    # Driver override hysteresis prevents request-bit chatter around the threshold.
    self.STEER_OVERRIDE_TORQUE_HIGH = 200
    self.STEER_OVERRIDE_TORQUE_LOW = 150

    if CP.flags & SubaruFlags.GLOBAL_GEN2:
      self.STEER_MAX = 1500
      self.STEER_DELTA_UP = 35
      self.STEER_DELTA_DOWN = 50
    elif CP.carFingerprint == CAR.SUBARU_IMPREZA_2020:
      self.STEER_DELTA_UP = 35
      self.STEER_MAX = 1439
    else:
      self.STEER_MAX = 2047

  # Experimental longitudinal maps derived from 65,769 stock EyeSight samples on the validated
  # 2025 Crosstrek route 38b065e31c0a9ed7/0000000b--eab0d07145.
  LONG_SPEED_BP = [0., 5., 10., 15., 20., 25., 30.]
  THROTTLE_BASE_V = [1818., 1908., 1928., 2042., 2350., 2666., 2689.]
  THROTTLE_MAX_V = [3076., 3299., 3307., 3340., 3765., 3837., 3868.]
  RPM_BASE_V = [100., 1182., 1216., 1250., 1350., 1514., 1594.]
  RPM_MAX_V = [843., 1822., 2034., 2050., 2572., 2749., 2750.]
  BRAKE_RPM_V = [300., 1030., 1050., 1100., 1235., 1350., 1500.]
  BRAKE_MAX_V = [400., 350., 320., 300., 290., 295., 315.]

  # Stock EyeSight coasts through light deceleration before applying the brakes.
  # These thresholds are the approximate future-acceleration points where the
  # observed brake command becomes dominant in the validation route.
  BRAKE_START_ACCEL_V = [-0.05, -0.25, -0.35, -0.40, -0.50, -0.60, -0.60]

  THROTTLE_MIN = 808
  THROTTLE_MAX = 3900
  THROTTLE_INACTIVE = 1818
  THROTTLE_ENGINE_BRAKE = 808

  BRAKE_MIN = 0
  BRAKE_MAX = 410

  RPM_MIN = 0
  RPM_MAX = 2800


class SubaruSafetyFlags(IntFlag):
  GEN2 = 1
  LONG = 2
  PREGLOBAL_REVERSED_DRIVER_TORQUE = 4
  LKAS_ANGLE = 8


class SubaruFlags(IntFlag):
  # Detected flags
  SEND_INFOTAINMENT = 1
  DISABLE_EYESIGHT = 2

  # Static flags
  GLOBAL_GEN2 = 4

  # Cars that temporarily fault when steering angle rate is greater than some threshold.
  # Appears to be all torque-based cars produced around 2019 - present
  STEER_RATE_LIMITED = 8
  PREGLOBAL = 16
  HYBRID = 32
  LKAS_ANGLE = 64


GLOBAL_ES_ADDR = 0x787
GEN2_ES_BUTTONS_DID = b'\x11\x30'
# The observed active-low Gen3 cruise-main bit is the fail-closed engagement
# control: turning cruise off revokes control, and a deliberate off-to-on edge
# requests engagement. Set/Resume remain unavailable until DID 0x1130 is decoded.
GEN3_LONGITUDINAL_READY = True


class CanBus:
  main = 0
  alt = 1
  camera = 2


class Footnote(Enum):
  GLOBAL = CarFootnote(
    "In the non-US market, openpilot requires the car to come equipped with EyeSight with Lane Keep Assistance.",
    Column.PACKAGE)
  EXP_LONG = CarFootnote(
    "Enabling longitudinal control (alpha) will disable all EyeSight functionality, including AEB, LDW, and RAB.",
    Column.LONGITUDINAL)


@dataclass
class SubaruCarDocs(CarDocs):
  package: str = "EyeSight Driver Assistance"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.subaru_a]))
  footnotes: list[Enum] = field(default_factory=lambda: [Footnote.GLOBAL])

  def init_make(self, CP: CarParams):
    if CP.alphaLongitudinalAvailable:
      self.footnotes.append(Footnote.EXP_LONG)


@dataclass
class SubaruPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: {Bus.pt: 'subaru_global_2017_generated'})

  def init(self):
    if self.flags & SubaruFlags.HYBRID:
      self.dbc_dict = {Bus.pt: 'subaru_global_2020_hybrid_generated'}


@dataclass
class SubaruGen2PlatformConfig(SubaruPlatformConfig):
  def init(self):
    super().init()
    self.flags |= SubaruFlags.GLOBAL_GEN2
    if not (self.flags & SubaruFlags.LKAS_ANGLE):
      self.flags |= SubaruFlags.STEER_RATE_LIMITED


class CAR(Platforms):
  # Global platform
  SUBARU_ASCENT = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Ascent 2019-21", "All")],
    CarSpecs(mass=2031, wheelbase=2.89, steerRatio=13.5),
  )
  SUBARU_OUTBACK = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Outback 2020-22", "All", car_parts=CarParts.common([CarHarness.subaru_b]))],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=17),
  )
  SUBARU_LEGACY = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Legacy 2020-22", "All", car_parts=CarParts.common([CarHarness.subaru_b]))],
    SUBARU_OUTBACK.specs,
  )
  SUBARU_IMPREZA = SubaruPlatformConfig(
    [
      SubaruCarDocs("Subaru Impreza 2017-19"),
      SubaruCarDocs("Subaru Crosstrek 2018-19", video="https://youtu.be/Agww7oE1k-s?t=26"),
      SubaruCarDocs("Subaru XV 2018-19", video="https://youtu.be/Agww7oE1k-s?t=26"),
    ],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=15),
  )
  SUBARU_IMPREZA_2020 = SubaruPlatformConfig(
    [
      SubaruCarDocs("Subaru Impreza 2020-22"),
      SubaruCarDocs("Subaru Crosstrek 2020-23"),
      SubaruCarDocs("Subaru XV 2020-21"),
    ],
    CarSpecs(mass=1480, wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.STEER_RATE_LIMITED,
  )
  # TODO: is there an XV and Impreza too?
  SUBARU_CROSSTREK_HYBRID = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Crosstrek Hybrid 2020", car_parts=CarParts.common([CarHarness.subaru_b]))],
    CarSpecs(mass=1668, wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.HYBRID,
  )
  SUBARU_FORESTER = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2019-21", "All")],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.STEER_RATE_LIMITED,
  )
  SUBARU_FORESTER_HYBRID = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester Hybrid 2020")],
    SUBARU_FORESTER.specs,
    flags=SubaruFlags.HYBRID,
  )
  # Pre-global
  SUBARU_FORESTER_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2017-18")],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=20),
    {Bus.pt: 'subaru_forester_2017_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_LEGACY_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Legacy 2015-18")],
    CarSpecs(mass=1568, wheelbase=2.67, steerRatio=12.5),
    {Bus.pt: 'subaru_outback_2015_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_OUTBACK_PREGLOBAL = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Outback 2015-17")],
    SUBARU_FORESTER_PREGLOBAL.specs,
    {Bus.pt: 'subaru_outback_2015_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  SUBARU_OUTBACK_PREGLOBAL_2018 = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Outback 2018-19")],
    SUBARU_FORESTER_PREGLOBAL.specs,
    {Bus.pt: 'subaru_outback_2019_generated'},
    flags=SubaruFlags.PREGLOBAL,
  )
  # Angle LKAS
  SUBARU_FORESTER_2022 = SubaruPlatformConfig(
    [SubaruCarDocs("Subaru Forester 2022-24", "All", car_parts=CarParts.common([CarHarness.subaru_c]))],
    SUBARU_FORESTER.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )
  SUBARU_OUTBACK_2023 = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Outback 2023", "All", car_parts=CarParts.common([CarHarness.subaru_d]))],
    SUBARU_OUTBACK.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )
  SUBARU_ASCENT_2023 = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Ascent 2023", "All", car_parts=CarParts.common([CarHarness.subaru_d]))],
    SUBARU_ASCENT.specs,
    flags=SubaruFlags.LKAS_ANGLE,
  )
  SUBARU_CROSSTREK_2025 = SubaruGen2PlatformConfig(
    [SubaruCarDocs("Subaru Crosstrek 2024-25", "All", car_parts=CarParts.common([CarHarness.subaru_d]))],
    CarSpecs(mass=1529, wheelbase=2.67, steerRatio=17),
    flags=SubaruFlags.LKAS_ANGLE,
  )


SUBARU_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)
SUBARU_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(uds.DATA_IDENTIFIER_TYPE.APPLICATION_DATA_IDENTIFICATION)

# The EyeSight ECU takes 10s to respond to SUBARU_VERSION_REQUEST properly,
# log this alternate manufacturer-specific query
SUBARU_ALT_VERSION_REQUEST = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER]) + \
  p16(0xf100)
SUBARU_ALT_VERSION_RESPONSE = bytes([uds.SERVICE_TYPE.READ_DATA_BY_IDENTIFIER + 0x40]) + \
  p16(0xf100)

FW_QUERY_CONFIG = FwQueryConfig(
  fw_version_regex=br"(?:[\x00-\xff]{4,5}|[\x00-\xff]{8}|[\x00-\xff]{10})",
  requests=[
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      logging=True,
    ),
    # Non-OBD requests
    # Some Eyesight modules fail on TESTER_PRESENT_REQUEST
    # TODO: check if this resolves the fingerprinting issue for the 2023 Ascent and other new Subaru cars
    Request(
      [SUBARU_VERSION_REQUEST],
      [SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
    ),
    Request(
      [SUBARU_ALT_VERSION_REQUEST],
      [SUBARU_ALT_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.DEFAULT_DIAGNOSTIC_REQUEST, StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.DEFAULT_DIAGNOSTIC_RESPONSE, StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.fwdCamera],
      bus=0,
      logging=True,
    ),
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      bus=0,
    ),
    # GEN2 powertrain bus query
    Request(
      [StdQueries.TESTER_PRESENT_REQUEST, SUBARU_VERSION_REQUEST],
      [StdQueries.TESTER_PRESENT_RESPONSE, SUBARU_VERSION_RESPONSE],
      whitelist_ecus=[Ecu.abs, Ecu.eps, Ecu.fwdCamera, Ecu.engine, Ecu.transmission],
      bus=1,
      obd_multiplexing=False,
    ),
  ],
  # We don't get the EPS from non-OBD queries on GEN2 cars. Note that we still attempt to match when it exists
  non_essential_ecus={
    Ecu.eps: [c for c in CAR if c.config.flags & SubaruFlags.GLOBAL_GEN2],
  }
)

DBC = CAR.create_dbc_map()
