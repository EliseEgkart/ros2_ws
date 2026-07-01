"""
===============================================================================
File        : gripper_node.py
Package     : arm_controller_pkg
Author      : Jeho Yoon, Chaerin Seong, Dahan Lee, Donghyuk Jeong, Deokhui Han, Donggil Lee
Created     : 2026-07-02
Environment : Ubuntu 22.04, ROS2 Humble, Python 3.10

Description
-----------
ROS2 node that controls the serial-connected gripper for robot1.
Exposes /gripper/grip and /gripper/open services (Trigger) to open or close
the gripper via UART commands sent to /dev/ttyARDUINO at 115200 baud.

Main Features
-------------
- /gripper/grip  service (Trigger): closes the gripper, response.message carries pos
- /gripper/open  service (Trigger): opens the gripper
- Serial communication with gripper hardware over /dev/ttyARDUINO
- Thread-safe serial access with lock
- Result polling with configurable timeout

Required Nodes
--------------
- None (hardware driver node)

Notes
-----
- Device path: /dev/ttyARDUINO, baud: 115200

Revision History
----------------
- 2026-07-01: control_gripper(SetBool) 단일 서비스를 /gripper/grip, /gripper/open
              (Trigger) 두 개로 분리. amr_robot_node.py 가 실제로 호출하는
              서비스 이름/타입에 맞춤.

===============================================================================
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import serial
import time
import threading
import re
GRIPPER_PORT = "/dev/ttyARDUINO"
GRIPPER_BAUD = 115200


class GripperNode(Node):
    def __init__(self):
        super().__init__('gripper_node')
        self.grip_srv = self.create_service(Trigger, '/gripper/grip', self.grip_cb)
        self.grip100_srv = self.create_service(Trigger, '/gripper/grip100', self.grip100_cb)
        self.grip110_srv = self.create_service(Trigger, '/gripper/grip110', self.grip110_cb)
        self.grip120_srv = self.create_service(Trigger, '/gripper/grip120', self.grip120_cb)
        self.open_srv = self.create_service(Trigger, '/gripper/open', self.open_cb)
        self._serial_lock = threading.Lock()
        self._last_pos: int | None = None

        try:
            # timeout=0.5: readline 한 번당 최대 대기. 실제 완료 판정은 _wait_for_result에서 함
            self.ser = serial.Serial("/dev/ttyARDUINO", 115200, timeout=0.5)
            self.get_logger().info("✅ Gripper Serial Connected. 초기 open 확인 중...")

            # Case 1: 시리얼 연결 시 Arduino가 리셋되면 setup()에서 자동 open → [OPEN] 출력
            result = self._wait_for_result("[OPEN]", None, total_timeout_s=5.0)
            if result:
                self.get_logger().info("✅ 부팅 자동 open 완료.")
            else:
                # Case 2: Arduino가 이미 실행 중(리셋 없음) → 명시적 open 전송
                self.get_logger().info("⚠️ 부팅 auto-open 없음. 명시적 open 전송...")
                self.ser.reset_input_buffer()
                self.ser.write(b"open\n")
                result2 = self._wait_for_result("[OPEN]", None, total_timeout_s=4.0)
                if result2:
                    self.get_logger().info("✅ 명시적 open 완료.")
                else:
                    self.get_logger().warn("⚠️ open 응답 없음. 계속 진행합니다.")

        except Exception as e:
            self.get_logger().error(f"❌ Serial Error: {e}")

    def _wait_for_result(self, success_kw, fail_kw, total_timeout_s):
        """
        Arduino 응답 라인을 읽어가며 키워드가 포함된 라인을 찾는다.
          success_kw 발견 → True
          fail_kw 발견    → False
          전체 타임아웃    → None
        호출자가 _serial_lock을 보유한 상태에서 호출한다.
        """
        deadline = time.monotonic() + total_timeout_s
        while time.monotonic() < deadline:
            line = self.ser.readline().decode(errors='ignore').strip()
            if line:
                self.get_logger().info(f"[ARDUINO] {line}")
                m = re.search(r'pos[:\s=]*(\d+)', line, re.IGNORECASE)
                if m:
                    self._last_pos = int(m.group(1))
            if success_kw and success_kw in line:
                return True
            if fail_kw and fail_kw in line:
                return False
        return None

    def _do_grip(self, command: bytes, response):
        """물체를 파지한다. amr_robot_node 의 장축/단축 판정을 위해
        response.message 에 최종 pos 값을 'Gripped|pos=<값>' 형식으로 실어 보낸다.
        command: 아두이노로 보낼 grip 명령 (예: b"grip\\n", b"grip110\\n")."""
        try:
            with self._serial_lock:
                self._last_pos = None  # 그립 시작 전 pos 초기화
                self.ser.write(command)
                self.get_logger().info(f"📌 Sent: {command.decode().strip()} — Arduino 파지 완료 신호 대기 중...")

                result = self._wait_for_result("Torque remains ON", "Failed", 5.0)
                pos_str = str(self._last_pos) if self._last_pos is not None else "-1"

                if result is True:
                    self.get_logger().info(f"📌 Grip SUCCESS (pos={pos_str})")
                    response.success = True
                    response.message = f"Gripped|pos={pos_str}"
                elif result is False:
                    self.get_logger().warn("📌 Grip FAIL: 물체 없음 또는 타임아웃. 그리퍼 열리는 중...")
                    self._wait_for_result("[OPEN]", None, 4.0)
                    response.success = False
                    response.message = f"Grip failed (no object or timeout)|pos={pos_str}"
                else:
                    self.get_logger().warn("📌 Grip: Arduino 응답 타임아웃. 파지 완료로 간주합니다.")
                    response.success = True
                    response.message = f"Gripped (no ACK fallback)|pos={pos_str}"

        except Exception as e:
            self.get_logger().error(f"❌ Grip Service Error: {e}")
            response.success = False
            response.message = str(e)

        return response

    def grip_cb(self, request, response):
        return self._do_grip(b"grip\n", response)

    def grip100_cb(self, request, response):
        return self._do_grip(b"grip100\n", response)

    def grip110_cb(self, request, response):
        return self._do_grip(b"grip110\n", response)

    def grip120_cb(self, request, response):
        return self._do_grip(b"grip120\n", response)

    def open_cb(self, request, response):
        try:
            with self._serial_lock:
                self.ser.write(b"open\n")
                self.get_logger().info("📌 Sent: open — Arduino 완료 신호 대기 중...")

                result = self._wait_for_result("[OPEN]", None, 4.0)
                response.success = True
                response.message = "Opened" if result else "Opened (no ACK fallback)"

        except Exception as e:
            self.get_logger().error(f"❌ Open Service Error: {e}")
            response.success = False
            response.message = str(e)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = GripperNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
            node.get_logger().info("✅ Serial Closed")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
