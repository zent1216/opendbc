from opendbc.car.hyundai.junpilot_cutin import CutInDetector, candidate


def trk(d, y, vy, rel=0.0, cnt=20):
  return {"VALID_CNT": cnt, "LONG_DIST": d, "LAT_DIST": y, "LAT_SPEED": vy, "REL_SPEED": rel}


V = 20.0


def test_candidate():
  assert candidate(trk(15, 2.5, -0.8), V) is not None      # 왼쪽 차가 우리 쪽으로
  assert candidate(trk(15, -2.5, 0.8), V) is not None      # 오른쪽 차가 우리 쪽으로
  assert candidate(trk(15, 2.5, 0.8), V) is None           # 멀어지는 중
  assert candidate(trk(15, 2.5, -0.1), V) is None          # 그냥 옆 차선 주행
  assert candidate(trk(15, 0.5, -0.8), V) is None          # 이미 우리 차선 (앞차 처리)
  assert candidate(trk(15, 2.5, -0.8, rel=-V), V) is None  # 서 있는 물체
  assert candidate(trk(15, 2.5, -3.5), V) is None          # 너무 빠른 옆 이동 (교차 차)
  assert candidate(trk(15, 2.5, -0.8, cnt=5), V) is None   # 아직 불확실한 트랙


def test_confirm_and_hold():
  det = CutInDetector()
  t = [trk(15, 2.5, -0.8, rel=-2.0)]
  assert det.update(t, V, 0.1) == (0.0, 0.0)  # 0.1초는 아직
  v, d = det.update(t, V, 0.1)
  assert abs(v - 18.0) < 1e-6 and d == 15
  for _ in range(15):
    assert det.update([], V, 0.1)[0] > 0      # 2초 유지
  for _ in range(10):
    out = det.update([], V, 0.1)
  assert out == (0.0, 0.0)
