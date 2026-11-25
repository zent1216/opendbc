from opendbc.can.packer import CANPacker
from opendbc.can.parser import CANParser
from opendbc.car.can_definitions import CanData
from opendbc.car.hyundai import hyundaicanfd
from opendbc.car.hyundai.hyundaicanfd import CanBus, hkg_can_fd_checksum
from opendbc.car.hyundai.junpilot_buttons import JunpilotGapSetter, GAP_INTERVAL, GAP_MAX_PRESSES
from opendbc.car.hyundai.values import CAR, DBC
from opendbc.car import Bus

# 실차 PV5 0x10B 프레임 (버튼 없음)
REAL = bytes.fromhex("7d64e000000000000000002000000000")


def _parse(dat):
  p = CANParser(DBC[CAR.KIA_PV5][Bus.pt], [("CRUISE_BUTTONS_ALT2", 0)], 0)
  p.update([(1, [CanData(0x10b, dat, 0)])])
  return dict(p.vl["CRUISE_BUTTONS_ALT2"])


def test_real_frame_checksum_matches():
  assert hkg_can_fd_checksum(0x10b, None, bytearray(REAL)) == REAL[0] | (REAL[1] << 8)


def test_create_pv5_buttons_copies_and_sets_button():
  packer = CANPacker(DBC[CAR.KIA_PV5][Bus.pt])

  msg = hyundaicanfd.create_pv5_buttons(packer, CanBus(None, fingerprint={0: {}, 1: {}, 2: {}}), _parse(REAL), 0xe1, 3)
  addr, dat, bus = msg
  assert addr == 0x10b and bus == 2 and len(dat) == 16
  assert dat[10] & 0xF == 3 and not dat[10] & 0x80
  assert dat[10] & 0x70 == REAL[10] & 0x70  # 나머지 비트는 원래 값 그대로
  assert dat[2] == 0xe1
  assert hkg_can_fd_checksum(0x10b, None, bytearray(dat)) == dat[0] | (dat[1] << 8)
  assert dat[3:10] == REAL[3:10] and dat[11:] == REAL[11:]


def _run(g, frames, **kw):
  presses = 0
  for _ in range(frames):
    presses += g.update(**kw)
  return presses


def test_gap_setter():
  g = JunpilotGapSetter()
  assert _run(g, 50, want_gap=2, cur_gap=4, cruise_enabled=True, driver_gap_pressed=False) == 0  # 켜진 직후 1초 대기
  n = _run(g, int(5 * GAP_INTERVAL / 0.01), want_gap=2, cur_gap=4, cruise_enabled=True, driver_gap_pressed=False)
  assert 1 <= n <= GAP_MAX_PRESSES
  assert _run(g, 200, want_gap=2, cur_gap=2, cruise_enabled=True, driver_gap_pressed=False) == 0  # 맞으면 안 누름
  assert _run(g, 200, want_gap=0, cur_gap=4, cruise_enabled=True, driver_gap_pressed=False) == 0  # 0 = 그대로 두기


def test_gap_setter_stops_after_driver_touch():
  g = JunpilotGapSetter()
  g.update(want_gap=2, cur_gap=4, cruise_enabled=True, driver_gap_pressed=True)
  assert _run(g, 500, want_gap=2, cur_gap=4, cruise_enabled=True, driver_gap_pressed=False) == 0


def test_gap_setter_max_presses():
  g = JunpilotGapSetter()
  assert _run(g, 3000, want_gap=2, cur_gap=4, cruise_enabled=True, driver_gap_pressed=False) == GAP_MAX_PRESSES


def test_pv5_parser_accepts_counter_step_2():
  """실차 PV5 0x10B 카운터는 2씩 증가. 카운터 검사를 하면 CAN 오류(알 수 없는 차량 사양)가 난다."""
  from opendbc.car.hyundai.interface import CarInterface
  from opendbc.car.hyundai.carstate import CarState
  CP = CarInterface.get_params("KIA_PV5", {b: {} for b in range(8)}, [], False, False, False)
  ps = CarState(CP).get_can_parsers(CP)
  pt = ps[Bus.pt]
  for i in range(40):
    dat = bytearray(REAL)
    dat[2] = (i * 2) & 0xff
    cs = hkg_can_fd_checksum(0x10b, None, dat)
    dat[0], dat[1] = cs & 0xff, cs >> 8
    pt.update([(int(1e9 + i * 4e7), [CanData(0x10b, bytes(dat), 0)])])
  assert pt.message_states[0x10b].counter_fail == 0
