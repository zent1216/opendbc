"""준파일럿: 기아 PV5 순정 내비의 단속 정보(과속카메라, 구간단속)를 읽는다.

신호 해석은 당근파일럿(ajouatom)의 PV5 실차 기록을 바탕으로 한다.
- 0x364 CANFD_HDA_INFO (E-CAN): MapSource==2 이면 내비가 단속 경고 중, SPEED_LIMIT = 단속 속도
- 0x380 CANFD_NAVI_STATUS (A-CAN): SECTION_ALERT 상승 = 구간단속 진입, SPEED_LIMIT = 구간 속도
판단 규칙
- 과속카메라: 경고가 시작된 뒤 CAMERA_HOLD_M 를 지나면 그 경고는 '소모'된다. 같은 속도 경고가 계속 남아 있어도
  다시 감속하지 않는다 (카메라를 지난 뒤에도 경고가 남는 경우 대비). 경고가 끝나거나 속도가 바뀌면 초기화.
- 구간단속: 진입 신호(상승)가 보이고 두 메시지의 속도가 같을 때 시작, 경고가 끝나거나 속도가 바뀌면 종료.
- 두 메시지 중 하나라도 1초 넘게 끊기면 즉시 해제.
"""
from opendbc.car.common.conversions import Conversions as CV

STALE_NS = 1_000_000_000
CAMERA_HOLD_M = 1000.0
NONE, CAMERA, SECTION = 0, 1, 2


def valid_limit(kph: float) -> bool:
  return 30 <= kph <= 150 and int(kph) % 5 == 0


class Pv5Navi:
  def __init__(self):
    self.kind = NONE
    self.limit_kph = 0
    self.camera_start_dist: float | None = None
    self.camera_speed = 0
    self.consumed_speed = 0
    self.section_prev = True  # 끊긴 뒤에는 낮은 값을 한 번 본 다음에만 진입 인정
    self.distance = 0.0

  def _clear(self):
    self.kind, self.limit_kph = NONE, 0
    self.camera_start_dist = None
    self.camera_speed = 0

  def update(self, hda: dict | None, hda_fresh: bool, status: dict | None, status_fresh: bool,
             v_ego: float, dt: float) -> tuple[int, float]:
    """(종류, 제한속도 m/s) 반환. 종류 0이면 제한 없음."""
    self.distance += max(v_ego, 0.0) * dt
    if not (hda_fresh and status_fresh and hda is not None and status is not None):
      self._clear()
      self.consumed_speed = 0
      self.section_prev = True
      return NONE, 0.0

    hda_speed = int(hda.get("SPEED_LIMIT", 0))
    warning = int(hda.get("MapSource", 0)) == 2 and valid_limit(hda_speed)
    section_alert = bool(status.get("SECTION_ALERT", 0))
    rising = section_alert and not self.section_prev
    self.section_prev = section_alert

    if not warning:
      self._clear()
      self.consumed_speed = 0
      return NONE, 0.0

    # 구간단속
    if self.kind == SECTION and hda_speed != self.limit_kph:
      self._clear()
    if rising and int(status.get("SPEED_LIMIT", 0)) == hda_speed:
      self.kind, self.limit_kph = SECTION, hda_speed
      self.camera_start_dist = None
    if self.kind == SECTION:
      return SECTION, self.limit_kph * CV.KPH_TO_MS

    # 과속카메라
    if hda_speed != self.consumed_speed:
      self.consumed_speed = 0
    if self.consumed_speed:
      return NONE, 0.0
    if self.camera_start_dist is None or hda_speed != self.camera_speed:
      self.camera_start_dist = self.distance
      self.camera_speed = hda_speed
    if self.distance - self.camera_start_dist > CAMERA_HOLD_M:
      self.consumed_speed = hda_speed
      self._clear()
      return NONE, 0.0
    self.kind, self.limit_kph = CAMERA, hda_speed
    return CAMERA, hda_speed * CV.KPH_TO_MS


def message_fresh(parser, name: str, now_ns: int) -> bool:
  ts = max(parser.ts_nanos.get(name, {}).values(), default=0) if name in parser.ts_nanos else 0
  return ts > 0 and 0 <= now_ns - ts <= STALE_NS and not parser.bus_timeout
