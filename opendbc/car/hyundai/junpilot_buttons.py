"""준파일럿: 순정 스마트크루즈 설정속도를 크루즈 버튼(RES+/SET-)으로 목표값에 맞춘다.

가감속은 계속 순정 SCC가 하고, 여기서는 설정속도만 바꾼다. 긴급제동(AEB)도 그대로다.

빠르게 맞추기:
  한 번 누르고 계기판에 반영될 때까지(약 1초) 기다리면 1km/h에 1초 넘게 걸려 앞차·끼어들기에 늦는다.
  그래서 반영을 기다리지 않고 FAST_INTERVAL마다 누르되, 아직 반영 안 된 누름은 MAX_INFLIGHT개까지만 둔다.

무시 감지와 복구:
  버튼을 보냈는데 CONFIRM_TIME 동안 계기판 설정속도가 하나도 바뀌지 않으면 '무시'로 센다.
  - 연속 MISS_BACKOFF회: 잠시 멈췄다가 간격을 늘려 다시 시도
  - 연속 MISS_PAUSE회: PAUSE_TIME 동안 자동 조절 중지 (운전자 알림용 상태 노출)
  - 한 번이라도 반영되면 원래 간격으로 복귀
"""
from dataclasses import dataclass

from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.values import Buttons

DT_CTRL = 0.01

BASE_INTERVAL = 0.20     # 버튼 누름 간격(초): PV5는 한 번 누르는 데 약 0.16초 (0x10B 3프레임 + 떼기 1프레임)
MAX_INFLIGHT = 4         # 반영을 기다리는 누름 최대 수
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
    self.pending: tuple[float, int, int] | None = None  # (마지막 반영 확인 시각, 그때 설정속도, 버튼)
    self.inflight = 0  # 누르고 아직 반영 안 된 수
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
      self.pending, self.inflight = None, 0

    # 누른 버튼이 반영됐는지 확인: 설정속도가 누른 방향으로 바뀐 만큼 대기 중 누름을 지움
    if self.pending is not None:
      t0, before, btn = self.pending
      moved = (cur - before) if btn == Buttons.RES_ACCEL else (before - cur)
      if moved != 0:
        self.inflight = max(self.inflight - max(moved, 0), 0)
        self.pending = (self.t, cur, btn) if self.inflight > 0 else None
        st.misses = 0
        st.interval = BASE_INTERVAL
      elif self.t - t0 > CONFIRM_TIME:
        self.pending = None
        self.inflight = 0
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
    if not st.active:
      self.pending, self.inflight = None, 0
      return Buttons.NONE
    if self.t < self.next_press_t or self.inflight >= MAX_INFLIGHT:
      return Buttons.NONE

    tgt = max(self._units(target_ms), self.min_set)
    # 이미 누른(반영 대기) 만큼 갔다고 보고 남은 차이만 누름
    sign = 0 if self.pending is None else (1 if self.pending[2] == Buttons.RES_ACCEL else -1)
    expected = cur + sign * self.inflight
    if tgt == expected:
      return Buttons.NONE
    btn = Buttons.RES_ACCEL if tgt > expected else Buttons.SET_DECEL
    if self.pending is not None and btn != self.pending[2]:
      return Buttons.NONE  # 방향이 바뀌면 앞의 누름이 다 반영된 뒤에
    if self.pending is None:
      self.pending = (self.t, cur, btn)
    self.inflight += 1
    self.next_press_t = self.t + st.interval
    st.last_button = btn
    return btn


GAP_BUTTON = 3
GAP_INTERVAL = 0.6       # 차간거리 버튼 누름 간격(초)
GAP_MAX_PRESSES = 6      # 크루즈를 한 번 켤 때 최대 누름 수 (단계는 4개라 한 바퀴 반이면 충분)


class JunpilotGapSetter:
  """준파일럿: 크루즈가 켜질 때 순정 차간거리 단계를 설정값으로 맞춘다.

  - 지금 단계(1~4)가 원하는 단계와 다르면 차간거리 버튼을 GAP_INTERVAL마다 한 번씩 누른다.
  - 운전자가 핸들 버튼으로 차간거리를 바꾸면 그대로 둔다. 크루즈를 다시 켜거나, 원하는 단계가 새로 바뀌면
    (자가 학습이 운전자 선택을 배웠거나 상황이 바뀜) 다시 맞춘다.
  """

  def __init__(self):
    self.t = 0.0
    self.next_press_t = 0.0
    self.presses = 0
    self.driver_touched = False
    self.prev_enabled = False
    self.prev_want = 0

  def update(self, want_gap: int, cur_gap: int, cruise_enabled: bool, driver_gap_pressed: bool) -> bool:
    """매 제어 주기 호출. 이번 주기에 차간거리 버튼을 보내야 하면 True."""
    self.t += DT_CTRL
    if cruise_enabled and not self.prev_enabled:
      self.presses = 0
      self.driver_touched = False
      self.next_press_t = self.t + 1.0  # 크루즈가 켜진 직후 1초는 기다림
    self.prev_enabled = cruise_enabled
    if want_gap != self.prev_want:  # 원하는 단계가 바뀌면(자동 차간거리·자가 학습) 다시 맞춤
      self.presses = 0
      self.prev_want = want_gap
      self.driver_touched = False
    if driver_gap_pressed:
      self.driver_touched = True

    if not cruise_enabled or self.driver_touched or not (1 <= want_gap <= 4) or not (1 <= cur_gap <= 4):
      return False
    if cur_gap == want_gap or self.presses >= GAP_MAX_PRESSES or self.t < self.next_press_t:
      return False
    self.presses += 1
    self.next_press_t = self.t + GAP_INTERVAL
    return True
