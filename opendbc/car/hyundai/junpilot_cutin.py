"""준파일럿: 전방 레이더 트랙으로 옆 차선 차의 끼어들기를 미리 감지 (OPKR 끼어들기 감속 참고).

옆 차선(가로 1.2~3.3m)에서 움직이는 차가 우리 차선 쪽으로 옆걸음질(0.3m/s 이상)하고
1.5초 안에 차선 경계(1.3m)에 닿을 것 같으면 '끼어들기'로 본다.
같은 차를 0.2초 이상 연속으로 볼 때만 인정하고, 한 번 인정하면 2초 동안 유지한다.
결과: 끼어드는 차의 속도(m/s)와 거리(m). 없으면 0.
"""
RADAR_MSG = "RADAR_TRACK_{:x}"
RADAR_ADDRS = range(0x3A5, 0x3C5)

LANE_EDGE = 1.3       # m, 우리 차선 중심에서 경계까지 (이보다 안쪽은 이미 들어온 차: 앞차 처리에 맡김)
SIDE_MAX = 3.3        # m, 옆 차선 바깥 끝
LAT_SPEED_MIN = 0.3   # m/s, 우리 쪽으로 다가오는 옆 속도
LAT_SPEED_MAX = 2.5   # m/s, 이보다 빠르면 교차로 횡단 차 등 (차선 변경이 아님)
ENTER_TIME = 1.5      # s, 이 시간 안에 경계에 닿을 것 같으면
MIN_DIST, MAX_DIST = 2.0, 45.0
MOVING_MIN = 3.0      # m/s, 서 있는 물체(주차 차, 가드레일) 제외
CONFIRM_TIME = 0.2
HOLD_TIME = 2.0


def candidate(t: dict, v_ego: float) -> tuple[float, float] | None:
  """레이더 트랙 하나가 끼어드는 차면 (속도, 거리)를 돌려준다."""
  if t["VALID_CNT"] <= 10:
    return None
  d, y, vy = t["LONG_DIST"], t["LAT_DIST"], t["LAT_SPEED"]
  v_lead = v_ego + t["REL_SPEED"]
  if not (MIN_DIST < d < max(MAX_DIST, v_ego * 2.5)) or v_lead < MOVING_MIN:
    return None
  ay = abs(y)
  if not (LANE_EDGE - 0.1 < ay < SIDE_MAX) or abs(vy) > LAT_SPEED_MAX:
    return None
  toward = -vy if y > 0 else vy  # 우리 차선 쪽으로 다가오는 속도
  if toward < LAT_SPEED_MIN or (ay - LANE_EDGE) / toward > ENTER_TIME:
    return None
  return v_lead, d


class CutInDetector:
  def __init__(self):
    self.seen_t = 0.0      # 연속으로 본 시간
    self.hold_t = 0.0      # 남은 유지 시간
    self.v = 0.0
    self.d = 0.0

  def update(self, tracks: list[dict], v_ego: float, dt: float) -> tuple[float, float]:
    found = [c for t in tracks if (c := candidate(t, v_ego)) is not None]
    if found:
      self.seen_t += dt
      if self.seen_t >= CONFIRM_TIME:
        v, d = min(found, key=lambda c: c[1])  # 가장 가까운 차
        self.v, self.d, self.hold_t = v, d, HOLD_TIME
    else:
      self.seen_t = 0.0
    if self.hold_t > 0:
      self.hold_t -= dt
      return self.v, self.d
    return 0.0, 0.0
