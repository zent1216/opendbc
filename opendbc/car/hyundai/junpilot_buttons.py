"""준파일럿: 순정 스마트크루즈 설정속도를 크루즈 버튼(RES+/SET-)으로 목표값에 맞춘다.

가감속은 계속 순정 SCC가 하고, 여기서는 설정속도만 바꾼다. 긴급제동(AEB)도 그대로다.

무시 감지와 복구:
  버튼을 보냈는데 계기판 설정속도가 바뀌지 않으면 '무시'로 센다.
  - 연속 MISS_BACKOFF회: 잠시 멈췄다가 간격을 늘려 다시 시도
  - 연속 MISS_PAUSE회: PAUSE_TIME 동안 자동 조절 중지 (운전자 알림용 상태 노출)
  - 한 번이라도 반영되면 원래 간격으로 복귀
"""
from dataclasses import dataclass

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import Buttons

DT_CTRL = 0.01

BASE_INTERVAL = 0.30     # 버튼 누름 간격(초)
MAX_INTERVAL = 1.20
BURST = 5                # 한 번 누를 때 보내는 같은 메시지 수 (실차 로그로 조정 예정)
CONFIRM_TIME = 1.0       # 누른 뒤 반영을 기다리는 시간(초)
MISS_BACKOFF = 3
MISS_PAUSE = 6
BACKOFF_TIME = 2.0
PAUSE_TIME = 10.0
DRIVER_HOLDOFF = 2.0     # 운전자가 버튼을 만진 뒤 자동 조절을 쉬는 시간(초)
MIN_SET_KPH = 30         # 순정 SCC 최저 설정속도 근처는 건드리지 않음


@dataclass
class FollowerStatus:
  active: bool = False
  paused: bool = False
  misses: int = 0
  interval: float = BASE_INTERVAL
  last_button: int = Buttons.NONE


class JunpilotButtonFollower:
  def __init__(self, is_metric: bool = True):
    self.to_unit = CV.MS_TO_KPH if is_metric else CV.MS_TO_MPH
    self.min_set = MIN_SET_KPH if is_metric else round(MIN_SET_KPH * CV.KPH_TO_MPH)
    self.status = FollowerStatus()
    self.t = 0.0
    self.next_press_t = 0.0
    self.pending: tuple[float, int, int] | None = None  # (누른 시각, 누르기 전 설정속도, 버튼)
    self.paused_until = 0.0
    self.driver_until = 0.0

  def _units(self, ms: float) -> int:
    return int(round(ms * self.to_unit))

  def update(self, target_ms: float, set_speed_ms: float, cruise_enabled: bool, standstill: bool,
             driver_button: bool, gas_or_brake: bool) -> int:
    """매 제어 주기(100Hz) 호출. 이번 주기에 보낼 버튼을 돌려준다 (NONE이면 안 보냄)."""
    self.t += DT_CTRL
    st = self.status
    cur = self._units(set_speed_ms)

    if driver_button:
      self.driver_until = self.t + DRIVER_HOLDOFF
      self.pending = None

    # 누른 버튼이 반영됐는지 확인
    if self.pending is not None:
      t0, before, btn = self.pending
      if cur != before:
        self.pending = None
        st.misses = 0
        st.interval = BASE_INTERVAL
      elif self.t - t0 > CONFIRM_TIME:
        self.pending = None
        st.misses += 1
        if st.misses >= MISS_PAUSE:
          self.paused_until = self.t + PAUSE_TIME
          st.misses = 0
          st.interval = BASE_INTERVAL
        elif st.misses >= MISS_BACKOFF:
          self.next_press_t = self.t + BACKOFF_TIME
          st.interval = min(st.interval * 2, MAX_INTERVAL)

    st.paused = self.t < self.paused_until
    usable = (target_ms > 0 and cruise_enabled and not standstill and not gas_or_brake
              and self.t >= self.driver_until and cur >= self.min_set)
    st.active = usable and not st.paused
    st.last_button = Buttons.NONE
    if not st.active or self.pending is not None or self.t < self.next_press_t:
      return Buttons.NONE

    tgt = max(self._units(target_ms), self.min_set)
    if tgt == cur:
      return Buttons.NONE
    btn = Buttons.RES_ACCEL if tgt > cur else Buttons.SET_DECEL
    self.pending = (self.t, cur, btn)
    self.next_press_t = self.t + st.interval
    st.last_button = btn
    return btn


GAP_BUTTON = 3
GAP_INTERVAL = 0.6       # 차간거리 버튼 누름 간격(초)
GAP_MAX_PRESSES = 6      # 크루즈를 한 번 켤 때 최대 누름 수 (단계는 4개라 한 바퀴 반이면 충분)


class JunpilotGapSetter:
  """준파일럿: 크루즈가 켜질 때 순정 차간거리 단계를 설정값으로 맞춘다.

  - 지금 단계(1~4)가 원하는 단계와 다르면 차간거리 버튼을 GAP_INTERVAL마다 한 번씩 누른다.
  - 운전자가 핸들 버튼으로 차간거리를 바꾸면 이번 시동 동안은 더 건드리지 않는다.
  - 크루즈를 껐다 켜면 다시 맞춘다 (운전자가 바꾼 경우 제외).
  """

  def __init__(self):
    self.t = 0.0
    self.next_press_t = 0.0
    self.presses = 0
    self.driver_touched = False
    self.prev_enabled = False

  def update(self, want_gap: int, cur_gap: int, cruise_enabled: bool, driver_gap_pressed: bool) -> bool:
    """매 제어 주기 호출. 이번 주기에 차간거리 버튼을 보내야 하면 True."""
    self.t += DT_CTRL
    if driver_gap_pressed:
      self.driver_touched = True
    if cruise_enabled and not self.prev_enabled:
      self.presses = 0
      self.next_press_t = self.t + 1.0  # 크루즈가 켜진 직후 1초는 기다림
    self.prev_enabled = cruise_enabled

    if not cruise_enabled or self.driver_touched or not (1 <= want_gap <= 4) or not (1 <= cur_gap <= 4):
      return False
    if cur_gap == want_gap or self.presses >= GAP_MAX_PRESSES or self.t < self.next_press_t:
      return False
    self.presses += 1
    self.next_press_t = self.t + GAP_INTERVAL
    return True
