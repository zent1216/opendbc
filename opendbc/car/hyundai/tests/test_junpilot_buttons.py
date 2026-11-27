from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai import junpilot_buttons as jb
from opendbc.car.hyundai.values import Buttons

KPH = CV.KPH_TO_MS


class FakeSCC:
  """버튼을 받으면 설정속도를 1km/h씩 바꾸는 가짜 순정 크루즈 (ignore=True면 무시)."""
  def __init__(self, kph):
    self.kph = kph
    self.ignore = False
    self.delay = 20  # 반영까지 0.2초
    self.queue = []

  def step(self, btn):
    self.queue = [(t - 1, d) for t, d in self.queue]
    for t, d in self.queue:
      if t <= 0:
        self.kph += d
    self.queue = [(t, d) for t, d in self.queue if t > 0]
    if btn != Buttons.NONE and not self.ignore:
      self.queue.append((self.delay, 1 if btn == Buttons.RES_ACCEL else -1))


def run(f, scc, target_kph, seconds, **kw):
  args = dict(cruise_enabled=True, standstill=False, driver_button=False, gas_or_brake=False) | kw
  presses = []
  for _ in range(int(seconds / jb.DT_CTRL)):
    btn = f.update(target_kph * KPH, scc.kph * KPH, **args)
    if btn != Buttons.NONE:
      presses.append(btn)
    scc.step(btn)
  return presses


def test_reaches_target_both_directions():
  f, scc = jb.JunpilotButtonFollower(), FakeSCC(100)
  run(f, scc, 90, 10)
  assert scc.kph == 90
  run(f, scc, 95, 10)
  assert scc.kph == 95
  assert run(f, scc, 95, 2) == []  # 목표 도달 후엔 안 누름


def test_one_press_at_a_time():
  f, scc = jb.JunpilotButtonFollower(), FakeSCC(100)
  presses = run(f, scc, 80, 1.0)
  assert 1 <= len(presses) <= int(1.0 / jb.BASE_INTERVAL) + 1


def test_inactive_conditions():
  for kw in (dict(cruise_enabled=False), dict(standstill=True), dict(gas_or_brake=True)):
    f, scc = jb.JunpilotButtonFollower(), FakeSCC(100)
    assert run(f, scc, 80, 3, **kw) == []
  f, scc = jb.JunpilotButtonFollower(), FakeSCC(100)
  assert run(f, scc, 0, 3) == []  # 목표 0 = 사용 안 함


def test_driver_button_holdoff():
  f, scc = jb.JunpilotButtonFollower(), FakeSCC(100)
  f.update(80 * KPH, 100 * KPH, True, False, True, False)  # 운전자가 버튼 누름
  assert run(f, scc, 80, jb.DRIVER_HOLDOFF - 0.1) == []
  assert run(f, scc, 80, 1.0) != []


def test_never_below_min_set():
  f, scc = jb.JunpilotButtonFollower(), FakeSCC(35)
  run(f, scc, 10, 15)
  assert scc.kph == jb.MIN_SET_KPH


def test_ignored_buttons_backoff_pause_and_recover():
  f, scc = jb.JunpilotButtonFollower(), FakeSCC(100)
  scc.ignore = True
  run(f, scc, 80, (jb.CONFIRM_TIME + jb.BASE_INTERVAL) * jb.MISS_BACKOFF + 0.5)
  assert f.status.interval > jb.BASE_INTERVAL  # 간격을 늘림
  run(f, scc, 80, 30)
  assert f.status.paused or f.paused_until > 0  # 계속 무시되면 일시 중지
  # 차가 다시 받아주면 원래대로 복귀
  scc.ignore = False
  run(f, scc, 80, jb.PAUSE_TIME + 20)
  assert scc.kph == 80
  assert f.status.misses == 0 and f.status.interval == jb.BASE_INTERVAL and not f.status.paused


def test_fast_with_slow_cluster():
  """계기판 반영이 0.8초 늦어도 20km/h를 몇 초 안에 내림 (예전엔 1km/h에 1초 넘게 걸림)."""
  from opendbc.car.hyundai.junpilot_buttons import JunpilotButtonFollower, MAX_INFLIGHT
  from opendbc.car.hyundai.values import Buttons
  f = JunpilotButtonFollower()
  set_kph, queue, t = 80, [], 0.0
  for i in range(1000):
    t = i * 0.01
    while queue and queue[0][0] <= t:
      set_kph += queue.pop(0)[1]
    btn = f.update(60 / 3.6, set_kph / 3.6, True, False, False, False)
    if btn != Buttons.NONE:
      queue.append((t + 0.8, 1 if btn == Buttons.RES_ACCEL else -1))
    assert len(queue) <= MAX_INFLIGHT
    if set_kph == 60 and not queue:
      break
  assert set_kph == 60 and t < 6.0


def test_pv5_press_uses_odd_counters_between_stock():
  """PV5: 원래 0x10B(짝수 카운터, 40ms)가 올 때마다 카운터+1로 버튼 프레임을 하나씩, 3프레임 누르고 1프레임 뗌."""
  from opendbc.car.hyundai.carcontroller import CarController, JP_PRESS_FRAMES
  from opendbc.car.hyundai.values import Buttons

  class FakeCC(CarController):
    def __init__(self):  # 버튼 관련 상태만
      self.jp_btn_queue, self.jp_btn_cur, self.jp_btn_hold, self.jp_btn_release, self.jp_btn_last_cnt = [], 0, 0, 0, -1
      self.frame = 0
      self.packer = None
      self.CAN = None

  import opendbc.car.hyundai.carcontroller as ccmod
  sent = []
  orig = ccmod.hyundaicanfd.create_pv5_buttons
  ccmod.hyundaicanfd.create_pv5_buttons = lambda packer, CAN, msg, cnt, btn: sent.append((cnt, btn)) or (0x10B, b"", 2)
  try:
    c = FakeCC()
    c.jp_btn_queue = [Buttons.SET_DECEL, Buttons.SET_DECEL]

    class CS:
      pv5_btn_msg = {"COUNTER": 0}
    for stock in range(0, 40, 2):
      CS.pv5_btn_msg = {"COUNTER": stock}
      for _ in range(4):  # 같은 원래 프레임 동안 여러 주기 호출돼도 한 번만
        c._pv5_press(CS)
  finally:
    ccmod.hyundaicanfd.create_pv5_buttons = orig
  assert [cnt for cnt, _ in sent] == [1, 3, 5, 9, 11, 13]  # 3프레임 누르고 7(=6+1)은 뗌
  assert all(cnt % 2 == 1 for cnt, _ in sent) and len(sent) == 2 * JP_PRESS_FRAMES
