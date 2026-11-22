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
