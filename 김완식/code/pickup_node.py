import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from std_msgs.msg import Bool, Int32, String


class PickupNode(Node):
    def __init__(self):
        super().__init__("pickup_node")

        # 카메라 광학 -> AGV base_link 오프셋 (실측, 단위 m)
        self.OFFSET_X = 0.03    # 앞으로 3cm
        self.OFFSET_Y = -0.045  # 오른쪽 4.5cm -> Y축 -
        self.OFFSET_Z = 0.39    # 바닥에서 39cm

        # 마지막 유효 좌표 hold (51: 변환된 AGV base 좌표)
        self.last_target = None
        # 마지막 유효 블록 원본 좌표 (50: 카메라 광학 x/z, 거리각도 계산용)
        self.last_block = None
        self.last_block_time = 0.0    # 마지막 /selected_block 수신 시각
        self.last_laser = None        # 마지막 레이저 지점 좌표 (50 유도용)
        self.last_laser_time = 0.0    # 마지막 /laser_point 수신 시각
        # 현재 agv_mode (50=유도, 51=좌표송출)
        self.agv_mode = 0
        # FSM 제어 플래그 (51 좌표송출 게이트)
        self.active = False
        # 픽업 진행 플래그 (status 1 -> object_pose 발행 게이트)
        self.pickup_engaged = False
        # 51 좌표송출 중 마지막 블록 좌표 스냅샷 (status 1 시점 발행용)
        self.locked_block = None
        self.last_class = None        # 최근 감지된 블록 클래스명
        self.locked_class = None      # status 1 시점 저장된 타겟 클래스명
        self.research_active = False  # AGV_status 1 이후 재탐색 모드
        self.research_sent = 0        # 재탐색 매칭 발행 횟수 (최대 3)

        # 구독
        self.block_sub = self.create_subscription(
            Point, "/selected_block", self.on_selected_block, 10)
        self.laser_sub = self.create_subscription(
            Point, "/laser_point", self.on_laser_point, 10)
        self.block_class_sub = self.create_subscription(
            String, "/selected_block_class", self.on_selected_block_class, 10)
        self.active_sub = self.create_subscription(
            Bool, "/pickup_active", self.on_pickup_active, 10)
        self.mode_sub = self.create_subscription(
            Int32, "/agv_mode", self.on_agv_mode, 10)
        self.status_sub = self.create_subscription(
            Int32, "/AGV_status", self.on_agv_status, 10)

        # 발행
        self.target_pub = self.create_publisher(Point, "/pickup_target", 10)
        self.cmd_pub = self.create_publisher(Twist, "/target_cmd", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/object_pose", 10)

        # 10Hz 발행 타이머
        self.timer = self.create_timer(0.1, self.publish_target)

        self.get_logger().info("pickup_node started")

    def on_laser_point(self, msg):
        # 레이저 지점 좌표 저장 (50 유도용)
        self.last_laser = msg
        self.last_laser_time = time.time()

    def on_selected_block(self, msg):
        # 원본 블록 좌표 저장 (50 유도 거리각도 계산용)
        self.last_block = msg
        self.last_block_time = time.time()
        # 카메라 광학 좌표 -> AGV base_link 좌표 변환 (51 좌표송출용)
        target = Point()
        target.x = msg.z + self.OFFSET_X
        target.y = -msg.x + self.OFFSET_Y
        target.z = -msg.y + self.OFFSET_Z
        self.last_target = target
        # 재탐색 모드: locked_class와 매칭되면 뎁스 원본 XYZ를 최대 3회 발행
        if (self.research_active and self.research_sent < 3
                and self.last_class == self.locked_class):
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = "camera_link"
            pose.pose.position.x = float(msg.x)
            pose.pose.position.y = float(msg.y)
            pose.pose.position.z = float(msg.z)
            pose.pose.orientation.w = 1.0
            self.pose_pub.publish(pose)
            self.research_sent += 1
            self.get_logger().info(
                f"재탐색 매칭({self.locked_class}) -> /object_pose 발행 "
                f"[{self.research_sent}/3] "
                f"(x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f})")
            if self.research_sent >= 3:
                self.research_active = False
                self.pickup_engaged = False
                self.get_logger().info("재탐색 3회 발행 완료 -> 종료")

    def on_agv_mode(self, msg):
        prev = self.agv_mode
        self.agv_mode = msg.data
        # 픽업 좌표전송(51) 진입 여부 기억 (status 1 -> object_pose 발행 게이트)
        if self.agv_mode == 51:
            self.pickup_engaged = True
        # 픽업(50/51) 이탈 시 블록 캐시 초기화 (이전 좌표 잔존 방지)
        if self.agv_mode not in (50, 51):
            self.last_block = None
            self.last_target = None
            self.last_class = None
            self.locked_class = None
            self.research_active = False
            self.research_sent = 0
            self.pickup_engaged = False
            self.get_logger().info(
                f"[RESET] mode={self.agv_mode} -> 블록 캐시 초기화 완료")

    def on_agv_status(self, msg):
        # 밀착 완료(1): 재탐색 모드로 전환 (카메라에서 locked_class 찾히면 발행)
        if msg.data != 1:
            return
        if not self.pickup_engaged:
            return
        if self.locked_class is None:
            self.get_logger().warn("AGV_status=1 수신했으나 locked_class 없음 -> 재탐색 불가")
            self.pickup_engaged = False
            return
        self.research_active = True
        self.research_sent = 0
        self.get_logger().info(
            f"AGV_status=1 -> 재탐색 시작 (class={self.locked_class})")

    def on_selected_block_class(self, msg):
        self.last_class = msg.data

    def on_pickup_active(self, msg):
        self.active = msg.data
        if self.active:
            self.get_logger().info("pickup active: ON")
        else:
            self.get_logger().info("pickup active: OFF")
            self.last_target = None  # 중단 시 hold 초기화

    def publish_target(self):
        # [51] 좌표 송출 단계: pickup_active true일 때 변환좌표 hold 발행
        if self.active:
            if self.last_target is not None:
                self.target_pub.publish(self.last_target)
            # 마지막 블록 원본좌표 스냅샷 (status 1 시점 object_pose 발행용)
            if self.last_block is not None:
                self.locked_block = self.last_block
            # 클래스명 스냅샷 (status 1 시점 재탐색 기준)
            if self.last_class is not None:
                self.locked_class = self.last_class
            return
        # [50] 유도 단계: agv_mode 50일 때 블록 거리/각도로 target_cmd 발행
        if self.agv_mode == 50:
            self.publish_guide_cmd()

    def publish_guide_cmd(self):
        cmd = Twist()
        # /laser_point 최근 미수신(레이저 사라짐) 시 발행 스킵
        if time.time() - self.last_laser_time > 0.3:
            return
        if self.last_laser is None:
            # 레이저 미검출 -> 아무것도 안 보냄 (AGV 측 거동에 위임)
            return
        bx = self.last_laser.x   # 광학 x (오른쪽 +)
        bz = self.last_laser.z   # 광학 z (전방 +)
        dist = math.sqrt(bx * bx + bz * bz)     # 수평 직선거리
        angle = -math.atan2(bx, bz)             # 오른쪽(x+) -> AGV 우회전(angular.z -)
        cmd.linear.z = 0.0        # 추종 플래그
        cmd.linear.x = float(dist)
        cmd.angular.z = float(angle)
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PickupNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
