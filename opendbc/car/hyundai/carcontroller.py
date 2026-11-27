import numpy as np
from opendbc.can import CANPacker
from opendbc.car import Bus, DT_CTRL, make_tester_present_msg, structs
from opendbc.car.lateral import apply_driver_steer_torque_limits, apply_steer_angle_limits_vm, common_fault_avoidance
from opendbc.car.vehicle_model import VehicleModel
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.hyundai.junpilot_buttons import JunpilotButtonFollower, JunpilotGapSetter, GAP_BUTTON, BURST as JP_BURST
from opendbc.car.hyundai import hyundaicanfd, hyundaican
from opendbc.car.hyundai.hyundaicanfd import CanBus
from opendbc.car.hyundai.values import HyundaiFlags, Buttons, CarControllerParams, CAR
from opendbc.car.interfaces import CarControllerBase

VisualAlert = structs.CarControl.HUDControl.VisualAlert
LongCtrlState = structs.CarControl.Actuators.LongControlState

# EPS faults if you apply torque while the steering angle is above 90 degrees for more than 1 second
# All slightly below EPS thresholds to avoid fault
MAX_ANGLE = 85
MAX_ANGLE_FRAMES = 89
MAX_ANGLE_CONSECUTIVE_FRAMES = 2

# On some HKG CAN and CAN FD non-CANFD_ALT_BUTTONS, the cancel button (CF_Clu_CruiseSwState / CRUISE_BUTTONS = 4) is
# a pause/resume toggle, not a dedicated cancel. Firing it mid-brake inadvertently can cause a re-enable attempt
# and triggers the "SCC Conditions Not Met" alert. Delaying the button send lets factory SCC disengage
# naturally on brake press. We send ~100 ms later if it fails to do so, or if we want to cancel for another reason.
CANCEL_BUTTON_DELAY_FRAMES = 10


def process_hud_alert(enabled, fingerprint, hud_control):
  sys_warning = (hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw))

  # initialize to no line visible
  # TODO: this is not accurate for all cars
  sys_state = 1
  if hud_control.leftLaneVisible and hud_control.rightLaneVisible or sys_warning:  # HUD alert only display when LKAS status is active
    sys_state = 3 if enabled or sys_warning else 4
  elif hud_control.leftLaneVisible:
    sys_state = 5
  elif hud_control.rightLaneVisible:
    sys_state = 6

  # initialize to no warnings
  left_lane_warning = 0
  right_lane_warning = 0
  if hud_control.leftLaneDepart:
    left_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2
  if hud_control.rightLaneDepart:
    right_lane_warning = 1 if fingerprint in (CAR.GENESIS_G90, CAR.GENESIS_G80) else 2

  return sys_warning, sys_state, left_lane_warning, right_lane_warning


# 준파일럿 각도 조향
ANGLE_FILTER_TAU = 0.05  # 20Hz 계단을 이어주는 정도 (고속)
# 저속 부드럽게: 속도(km/h)별 목표 각도 필터 시간상수(초)·무시할 작은 변화(도)·조향 힘
FILTER_SPEED_KPH = [0., 10., 20., 40., 60.]
ANGLE_FILTER_TAUS = [0.15, 0.13, 0.10, 0.06, ANGLE_FILTER_TAU]
LOW_SPEED_KPH = [0., 10., 20., 30.]
ANGLE_DEADBAND = [0.4, 0.3, 0.15, 0.0]
GAIN_SPEED_KPH = [0., 10., 30., 60.]
GAIN_BY_SPEED = [150., 170., 215., 234.]
DRIVER_TQ_TAU = 0.12
ANGLE_OVERRIDE_TORQUE = 250.
# 저속 회전 양보
TURN_YIELD_SPEED = 30. / 3.6   # m/s
TURN_YIELD_TORQUE = 80.        # 깜빡이를 켠 상태에서 이만큼만 돌려도 양보 (평소 250)
TURN_YIELD_GAIN = 20.          # 양보 중 조향 힘 (평소 234, 운전자 개입 85)
TURN_YIELD_GAIN_DOWN = 8.      # 프레임당, 약 0.3초 만에 내려감

class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.CAN = CanBus(CP)
    self.params = CarControllerParams(CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.angle_limit_counter = 0

    self.accel_last = 0
    self.apply_torque_last = 0
    self.car_fingerprint = CP.carFingerprint
    self.last_button_frame = 0
    self.jp_buttons = JunpilotButtonFollower(is_metric=True)  # 준파일럿: 한국 km/h 기준
    self.jp_gap = JunpilotGapSetter()
    self.jp_pv5 = CP.carFingerprint == CAR.KIA_PV5
    self.jp_resume_frame = -100
    self.cancel_counter = 0
    # 준파일럿: 각도 조향 차 (PV5)
    self.angle_control = bool(CP.flags & HyundaiFlags.ANGLE_CONTROL)
    self.apply_angle_last = 0.0
    self.angle_gain = 0.0
    self.angle_target_f = 0.0
    self.driver_tq_f = 0.0
    self.turn_yield = False
    self.angle_hold = 0.0
    self.lfa_alt_counter = 0
    self.VM = VehicleModel(CP) if self.angle_control else None

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    hud_control = CC.hudControl

    # steering torque
    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_driver_steer_torque_limits(new_torque, self.apply_torque_last, CS.out.steeringTorque, self.params)

    # >90 degree steering fault prevention
    self.angle_limit_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringAngleDeg) >= MAX_ANGLE, CC.latActive,
                                                                       self.angle_limit_counter, MAX_ANGLE_FRAMES,
                                                                       MAX_ANGLE_CONSECUTIVE_FRAMES)

    if not CC.latActive:
      apply_torque = 0

    # 준파일럿: 각도 조향 차는 토크 명령을 쓰지 않고 각도 명령(LFA_ALT)으로 조향
    if self.angle_control:
      apply_torque = 0
      apply_steer_req = False
      if CC.latActive:
        self.driver_tq_f += DT_CTRL / (DRIVER_TQ_TAU + DT_CTRL) * (CS.out.steeringTorque - self.driver_tq_f)
      else:
        self.driver_tq_f = 0.0
      tq = abs(self.driver_tq_f)

      # 저속 회전 양보: 30km/h 아래에서 깜빡이를 켜고 핸들을 돌리면 운전자에게 핸들을 거의 다 넘김
      # (골목·주차장 진입처럼 크게 꺾을 때 오파가 버텨서 핸들이 무거운 문제). 깜빡이를 끄거나 손을 놓으면 서서히 돌려받음
      low_turn = CC.latActive and CS.out.vEgoRaw < TURN_YIELD_SPEED and (CS.out.leftBlinker or CS.out.rightBlinker)
      if low_turn and tq > TURN_YIELD_TORQUE:
        self.turn_yield = True
      elif not low_turn or tq < TURN_YIELD_TORQUE * 0.5:
        self.turn_yield = False

      # 모델은 20Hz라 목표 각도가 50ms마다 계단처럼 바뀜 → 100Hz로 부드럽게 이어줌.
      # (큰 잔떨림은 modeld가 지연을 보정하면서 곡률을 부드럽게 함)
      # 저속에서는 같은 곡률 잔떨림도 핸들 각도로는 크게 커져서 '두두둑' 하고 계속 미세 조정함 (실차 로그: 20km/h 아래에서
      # 모델 목표가 50ms마다 ±0.5도씩 오락가락). 저속일수록 ① 아주 작은 변화는 무시(유격)하고 ② 더 부드럽게 이어 줌
      v_kph = CS.out.vEgoRaw * 3.6
      if CC.latActive and not self.turn_yield:
        target = actuators.steeringAngleDeg
        db = float(np.interp(v_kph, LOW_SPEED_KPH, ANGLE_DEADBAND))
        if target > self.angle_hold + db:
          self.angle_hold = target - db
        elif target < self.angle_hold - db:
          self.angle_hold = target + db
        tau = float(np.interp(v_kph, FILTER_SPEED_KPH, ANGLE_FILTER_TAUS))
        self.angle_target_f += DT_CTRL / (tau + DT_CTRL) * (self.angle_hold - self.angle_target_f)
      else:
        self.angle_target_f = CS.out.steeringAngleDeg  # 양보 중엔 운전자가 돌린 각도를 그대로 따라감 (버티지 않음)
        self.angle_hold = CS.out.steeringAngleDeg
      self.apply_angle_last = apply_steer_angle_limits_vm(self.angle_target_f, self.apply_angle_last, CS.out.vEgoRaw,
                                                          CS.out.steeringAngleDeg, CC.latActive, self.params, self.VM)
      # 조향 힘(토크 비율): 당근파일럿처럼 운전자가 확실히 돌리면 천천히 낮추고, 손을 놓으면 서서히 회복.
      # 손만 얹어도 토크가 100~300 나와서 순간값으로 판단하면 힘이 수시로 빠졌다 들어왔다 함
      if CC.latActive:
        if self.turn_yield:
          self.angle_gain = max(self.angle_gain - TURN_YIELD_GAIN_DOWN, TURN_YIELD_GAIN)
        elif tq > ANGLE_OVERRIDE_TORQUE:
          self.angle_gain = max(self.angle_gain - self.params.ANGLE_TORQUE_GAIN_DOWN, self.params.ANGLE_TORQUE_GAIN_OVERRIDE)
        elif tq < ANGLE_OVERRIDE_TORQUE * 0.6:
          # 저속은 조향 힘을 낮춰 덜 딱딱하게 (사람처럼 느슨하게 잡음, 운전자가 끼어들기도 쉬움)
          max_gain = float(np.interp(v_kph, GAIN_SPEED_KPH, GAIN_BY_SPEED))
          if self.angle_gain > max_gain:
            self.angle_gain = max(self.angle_gain - self.params.ANGLE_TORQUE_GAIN_UP, max_gain)
          else:
            self.angle_gain = min(self.angle_gain + self.params.ANGLE_TORQUE_GAIN_UP, max_gain)
      else:
        self.angle_gain = max(self.angle_gain - self.params.ANGLE_TORQUE_GAIN_DOWN, 0.0)

    self.apply_torque_last = apply_torque

    # accel + longitudinal
    accel = float(np.clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    stopping = actuators.longControlState == LongCtrlState.stopping
    set_speed_in_units = hud_control.setSpeed * (CV.MS_TO_KPH if CS.is_metric else CV.MS_TO_MPH)

    can_sends = []

    # *** common hyundai stuff ***

    # tester present - w/ no response (keeps relevant ECU disabled)
    if self.frame % 100 == 0 and not (self.CP.flags & HyundaiFlags.CANFD_CAMERA_SCC) and self.CP.openpilotLongitudinalControl:
      # for longitudinal control, either radar or ADAS driving ECU
      addr, bus = 0x7d0, self.CAN.ECAN if self.CP.flags & HyundaiFlags.CANFD else 0
      if self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG.value:
        addr, bus = 0x730, self.CAN.ECAN
      can_sends.append(make_tester_present_msg(addr, bus, suppress_response=True))

      # for blinkers
      if self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
        can_sends.append(make_tester_present_msg(0x7b1, self.CAN.ECAN, suppress_response=True))

    # Delay the cancel button send so the brake can disengage factory SCC first.
    # Reset whenever openpilot is no longer requesting cancel.
    self.cancel_counter = self.cancel_counter + 1 if CC.cruiseControl.cancel else 0

    # *** CAN/CAN FD specific ***
    if self.CP.flags & HyundaiFlags.CANFD:
      can_sends.extend(self.create_canfd_msgs(apply_steer_req, apply_torque, set_speed_in_units, accel,
                                              stopping, hud_control, CS, CC))
    else:
      # Hold torque with induced temporary fault when cutting the actuation bit
      # FIXME: we don't use this with CAN FD?
      torque_fault = CC.latActive and not apply_steer_req

      can_sends.extend(self.create_can_msgs(apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel,
                                            stopping, hud_control, actuators, CS, CC))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    if self.angle_control:
      new_actuators.steeringAngleDeg = self.apply_angle_last
    new_actuators.accel = accel

    self.frame += 1
    return new_actuators, can_sends

  def create_can_msgs(self, apply_steer_req, apply_torque, torque_fault, set_speed_in_units, accel, stopping, hud_control, actuators, CS, CC):
    can_sends = []

    # HUD messages
    sys_warning, sys_state, left_lane_warning, right_lane_warning = process_hud_alert(CC.enabled, self.car_fingerprint,
                                                                                      hud_control)

    can_sends.append(hyundaican.create_lkas11(self.packer, self.frame, self.CP, apply_torque, apply_steer_req,
                                              torque_fault, CS.lkas11, sys_warning, sys_state, CC.enabled,
                                              hud_control.leftLaneVisible, hud_control.rightLaneVisible,
                                              left_lane_warning, right_lane_warning))

    # Button messages
    if not self.CP.openpilotLongitudinalControl:
      if self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
        can_sends.append(hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.CANCEL, self.CP))
      elif CC.cruiseControl.resume:
        # send resume at a max freq of 10Hz
        if (self.frame - self.last_button_frame) * DT_CTRL > 0.1:
          # send 25 messages at a time to increases the likelihood of resume being accepted
          can_sends.extend([hyundaican.create_clu11(self.packer, self.frame, CS.clu11, Buttons.RES_ACCEL, self.CP)] * 25)
          if (self.frame - self.last_button_frame) * DT_CTRL >= 0.15:
            self.last_button_frame = self.frame

    if self.frame % 2 == 0 and self.CP.openpilotLongitudinalControl:
      # TODO: unclear if this is needed
      jerk = 3.0 if actuators.longControlState == LongCtrlState.pid else 1.0
      use_fca = self.CP.flags & HyundaiFlags.USE_FCA.value
      can_sends.extend(hyundaican.create_acc_commands(self.packer, CC.enabled, accel, jerk, int(self.frame / 2),
                                                      hud_control, set_speed_in_units, stopping,
                                                      CC.cruiseControl.override, use_fca, self.CP))

    # 20 Hz LFA MFA message
    if self.frame % 5 == 0 and self.CP.flags & HyundaiFlags.SEND_LFA.value:
      can_sends.append(hyundaican.create_lfahda_mfc(self.packer, CC.enabled))

    # 5 Hz ACC options
    if self.frame % 20 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.extend(hyundaican.create_acc_opt(self.packer, self.CP))

    # 2 Hz front radar options
    if self.frame % 50 == 0 and self.CP.openpilotLongitudinalControl:
      can_sends.append(hyundaican.create_frt_radar_opt(self.packer))

    return can_sends

  def create_canfd_msgs(self, apply_steer_req, apply_torque, set_speed_in_units, accel, stopping, hud_control, CS, CC):
    can_sends = []

    lka_steering = self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG
    lka_steering_long = lka_steering and self.CP.openpilotLongitudinalControl

    # steering control
    can_sends.extend(hyundaicanfd.create_steering_messages(self.packer, self.CP, self.CAN, CC.enabled, apply_steer_req, apply_torque))
    # 준파일럿: 각도 조향 (카메라의 LFA_ALT는 판다가 막고, 오픈파일럿이 대신 보냄)
    if self.angle_control and CS.lfa_alt is not None:
      self.lfa_alt_counter = (self.lfa_alt_counter + 1) % 256
      can_sends.append(hyundaicanfd.create_lfa_alt(self.packer, self.CAN, CS.lfa_alt, self.lfa_alt_counter, CC.latActive,
                                                   self.apply_angle_last, self.angle_gain))
      # 조향장치 상태를 카메라로 전달 (진짜 MDPS는 판다가 카메라 쪽으로 못 가게 막음)
      for mdps in CS.mdps_frames:
        can_sends.append(hyundaicanfd.create_mdps_to_camera(self.packer, self.CAN, mdps, CS.lfa_alt))
      # 계기판에 오픈파일럿이 인식한 차선 표시 (카메라 원본은 판다가 막고 바꾼 값으로 대신 보냄)
      for ccnc in CS.ccnc_frames:
        can_sends.append(hyundaicanfd.create_ccnc_display(self.packer, self.CAN, ccnc, CC.latActive, CC.hudControl,
                                                          CS.out.leftBlinker, CS.out.rightBlinker, (self.frame // 50) % 2 == 0))

    # prevent LFA from activating on LKA steering cars by sending "no lane lines detected" to ADAS ECU
    if self.frame % 5 == 0 and lka_steering:
      can_sends.append(hyundaicanfd.create_suppress_lfa(self.packer, self.CAN, CS.lfa_block_msg,
                                                        self.CP.flags & HyundaiFlags.CANFD_LKA_STEER_MSG_ALT))

    # LFA and HDA icons
    # 준파일럿 (각도 조향): 카메라가 보낸 값을 그대로 복사하고 LFA 아이콘만 실제 조향 상태로
    # (직접 만들면 HDA·단속 표시용 다른 값이 0이 되고, LFA를 꺼도 아이콘이 켜져 있었음)
    if self.CP.flags & HyundaiFlags.ANGLE_CONTROL:
      for frame in CS.lfahda_frames:
        can_sends.append(hyundaicanfd.create_lfahda_copy(self.packer, self.CAN, frame, CC.latActive))
    elif self.frame % 5 == 0 and (not lka_steering or lka_steering_long):
      can_sends.append(hyundaicanfd.create_lfahda_cluster(self.packer, self.CAN, CC.enabled))

    # blinkers
    if lka_steering and self.CP.flags & HyundaiFlags.CANFD_ENABLE_BLINKERS:
      can_sends.extend(hyundaicanfd.create_spas_messages(self.packer, self.CAN, CC.leftBlinker, CC.rightBlinker))

    if self.CP.openpilotLongitudinalControl:
      if lka_steering:
        can_sends.extend(hyundaicanfd.create_adrv_messages(self.packer, self.CAN, self.frame))
      else:
        can_sends.extend(hyundaicanfd.create_fca_warning_light(self.packer, self.CAN, self.frame))
      if self.frame % 2 == 0:
        can_sends.append(hyundaicanfd.create_acc_control(self.packer, self.CAN, CC.enabled, self.accel_last, accel, stopping, CC.cruiseControl.override,
                                                         set_speed_in_units, hud_control))
        self.accel_last = accel
    else:
      # button presses
      if (self.frame - self.last_button_frame) * DT_CTRL > 0.25:
        # cruise cancel
        if CC.cruiseControl.cancel:
          # Here we send ACC message to cancel, not buttons. Don't delay
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            can_sends.append(hyundaicanfd.create_acc_cancel(self.packer, self.CP, self.CAN, CS.cruise_info))
            self.last_button_frame = self.frame
          elif self.cancel_counter > CANCEL_BUTTON_DELAY_FRAMES:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.CANCEL))
            self.last_button_frame = self.frame

        # cruise standstill resume
        elif CC.cruiseControl.resume:
          if self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS:
            # TODO: resume for alt button cars
            pass
          else:
            for _ in range(20):
              can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, Buttons.RES_ACCEL))
            self.last_button_frame = self.frame

      # 준파일럿 (PV5): 버튼은 0x10B. 차간거리 맞추기가 먼저, 그다음 설정속도 맞추기
      if self.jp_pv5 and CS.pv5_btn_msg is not None:
        busy = CC.cruiseControl.cancel or CC.cruiseControl.resume
        driver_btn = CS.cruise_buttons[-1]
        btn = Buttons.NONE
        if self.jp_gap.update(int(getattr(CC.cruiseControl, "junpilotGap", 0)), int(CS.out.junpilotCruiseGap),
                              CS.out.cruiseState.enabled and CC.enabled and not busy, driver_btn == GAP_BUTTON):
          btn = GAP_BUTTON
        speed_btn = self.jp_buttons.update(0.0 if busy or not CC.enabled else CC.cruiseControl.junpilotTargetSpeed,
                                           CS.out.cruiseState.speedCluster or CS.out.cruiseState.speed,
                                           CS.out.cruiseState.enabled, CS.out.cruiseState.standstill,
                                           driver_btn != Buttons.NONE, CS.out.gasPressed or CS.out.brakePressed)
        if btn == Buttons.NONE:
          btn = speed_btn
        # 정차 후 앞차 출발: 순정 SCC가 멈춰 있으면 RES+를 대신 눌러줌 (0.3초 간격)
        if CC.cruiseControl.resume and CS.out.cruiseState.standstill:
          btn = Buttons.NONE
          if self.frame - self.jp_resume_frame >= 30:
            btn = Buttons.RES_ACCEL
            self.jp_resume_frame = self.frame
        if btn != Buttons.NONE:
          cnt = int(CS.pv5_btn_msg.get("COUNTER", 0))
          for i in range(JP_BURST):
            can_sends.append(hyundaicanfd.create_pv5_buttons(self.packer, self.CAN, CS.pv5_btn_msg, cnt + 1 + i, btn))
          self.last_button_frame = self.frame

      # 준파일럿: 순정 크루즈 설정속도를 목표값으로 맞춤 (취소/재개 버튼을 보낼 때는 쉼)
      elif not (self.CP.flags & HyundaiFlags.CANFD_ALT_BUTTONS):
        busy = CC.cruiseControl.cancel or CC.cruiseControl.resume
        btn = self.jp_buttons.update(0.0 if busy or not CC.enabled else CC.cruiseControl.junpilotTargetSpeed,
                                     CS.out.cruiseState.speedCluster or CS.out.cruiseState.speed,
                                     CS.out.cruiseState.enabled, CS.out.cruiseState.standstill,
                                     CS.cruise_buttons[-1] != Buttons.NONE, CS.out.gasPressed or CS.out.brakePressed)
        if btn != Buttons.NONE:
          for _ in range(JP_BURST):
            can_sends.append(hyundaicanfd.create_buttons(self.packer, self.CP, self.CAN, CS.buttons_counter + 1, btn))
          self.last_button_frame = self.frame

    return can_sends
