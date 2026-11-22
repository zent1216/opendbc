"""PV5 단속 정보 판단 테스트. 프레임은 당근파일럿(ajouatom)이 기록한 PV5 실차 데이터."""
from opendbc.can import CANParser
from opendbc.car.car_helpers import interfaces
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import junpilot_navi as jn
from opendbc.car.hyundai.carstate import PV5_HDA_MSG, PV5_STATUS_MSG
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import CAR

H = bytes.fromhex
SECTION_HDA = H("5ef6be000000000041509a1100000100")       # 80km/h, MapSource 2
SECTION_LOW = H("80a5d30001000000000000325010f1000000000000000000")
SECTION_HIGH = H("b99cdd0001000000000010325010f1000000000000000000")  # 구간단속 진입 펄스
SECTION_EXIT_HDA = H("ec0d040000000000521e9a0940000100")  # MapSource 1 (경고 끝)
SECTION_EXIT = H("ba962c0001000000000000320010f1000000000000000000")
CAM_HDA = H("c8f096000000000040329a1100000100")           # 50km/h, MapSource 2
CAM_END_HDA = H("bbf1a0000000000040329a0900000100")       # MapSource 1
CAM_STATUS = H("3af78e0001000000000000323210f1000000000000000000")
CAM_APPROACH = H("d0af0b4001000030000000323210f1000000000000000000")


def decode(addr, dat):
  p = CANParser("hyundai_canfd_generated", [(addr, 0)], 0)
  p.update([(0, [(addr, dat, 0)])])
  return dict(p.vl[addr])


def feed(n, hda, status, v=20.0, steps=1, fresh=True):
  out = None
  for _ in range(steps):
    out = n.update(decode(868, hda), fresh, decode(896, status), fresh, v, 0.01)
  return out


def test_section_zone():
  n = jn.Pv5Navi()
  assert feed(n, SECTION_HDA, SECTION_LOW)[0] == jn.CAMERA  # 진입 전에도 경고 중이면 카메라로 감속
  kind, lim = feed(n, SECTION_HDA, SECTION_HIGH)
  assert kind == jn.SECTION and round(lim * CV.MS_TO_KPH) == 80
  # 펄스가 꺼져도 구간 안에서는 유지 (거리 제한 없음)
  assert feed(n, SECTION_HDA, SECTION_LOW, steps=100_000)[0] == jn.SECTION
  # 경고가 끝나면 해제
  assert feed(n, SECTION_EXIT_HDA, SECTION_EXIT) == (jn.NONE, 0.0)


def test_camera_warning_and_consume():
  n = jn.Pv5Navi()
  kind, lim = feed(n, CAM_HDA, CAM_APPROACH)
  assert kind == jn.CAMERA and round(lim * CV.MS_TO_KPH) == 50
  # 1000m 이상 지나면 같은 경고가 남아 있어도 다시 감속하지 않음
  assert feed(n, CAM_HDA, CAM_STATUS, v=20.0, steps=int(1000 / 0.2) + 10) == (jn.NONE, 0.0)
  assert feed(n, CAM_HDA, CAM_STATUS) == (jn.NONE, 0.0)
  # 경고가 끝난 뒤 새 경고는 다시 적용
  feed(n, CAM_END_HDA, CAM_STATUS)
  assert feed(n, CAM_HDA, CAM_APPROACH)[0] == jn.CAMERA


def test_stale_messages_clear_immediately():
  n = jn.Pv5Navi()
  feed(n, SECTION_HDA, SECTION_LOW)
  feed(n, SECTION_HDA, SECTION_HIGH)
  assert feed(n, SECTION_HDA, SECTION_HIGH, fresh=False) == (jn.NONE, 0.0)
  # 끊긴 뒤 오래된 HIGH가 다시 와도 구간 재진입으로 보지 않음
  assert feed(n, SECTION_HDA, SECTION_HIGH)[0] == jn.CAMERA


def test_invalid_limits_ignored():
  assert not jn.valid_limit(0) and not jn.valid_limit(255) and not jn.valid_limit(83) and jn.valid_limit(80)


def test_carstate_integration():
  CI = interfaces[CAR.KIA_PV5]
  CP = CI.get_params(CAR.KIA_PV5, {i: {} for i in range(8)}, [], False, False, False)
  ci = CI(CP)
  bus = CanBus(CP)
  assert ci.can_parsers.keys() >= {"pt", "cam", "alt"}
  t = 0
  for dat_hda, dat_st in ((SECTION_HDA, SECTION_LOW), (SECTION_HDA, SECTION_HIGH)):
    t += 10_000_000
    cs = ci.update([(t, [(0x364, dat_hda, bus.ECAN), (0x380, dat_st, bus.ACAN)])])
  assert cs.junpilotNaviKind == jn.SECTION and round(cs.junpilotNaviLimit * CV.MS_TO_KPH) == 80
  # 내비 메시지가 아예 없어도 CAN 오류로 보지 않음
  ci2 = CI(CP)
  cs2 = ci2.update([(10_000_000, [])])
  assert ci2.can_parsers["alt"].can_valid and not ci2.can_parsers["alt"].bus_timeout
  assert cs2.junpilotNaviKind == 0
