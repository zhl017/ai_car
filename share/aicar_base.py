#!/usr/bin/env python3


import time
import rclpy
from rclpy.node import Node


from dynamixel_sdk import PortHandler
from dxl_arm import Arm_Controller
from dxl_wheel import Wheel_Controller


from sensor_msgs.msg import JointState
from std_msgs.msg import Empty, Int8, Int32, Int32MultiArray, Float32MultiArray
from geometry_msgs.msg import Twist


# ── 常數定義 ────────────────────────────────────────────────
BAUD_RATE        = 1000000   # Dynamixel 通訊鮑率
WHEEL_SEPERATION = 0.16      # 左右輪距 (m)
LIMIT_VEL        = 265       # 最大速度值
VELOCITY2VALUE   = 41.69988758
XM_TICK2RAD      = 0.001533981




def constrain(vel):
    if vel < -LIMIT_VEL: return -LIMIT_VEL
    if vel > LIMIT_VEL: return LIMIT_VEL
    return vel


class AICAR_BASE(Node):
    def __init__(self):
        super().__init__('aicar_base')


        # --- 初始化 Dynamixel Port ---
        self.get_logger().info('Connect U2D2 ...')        
        # TODO: 建立 PortHandler 實例，連接 U2D2，設定鮑率，成功後印出 '[ OK ] AICAR start'，失敗則印出錯誤訊息並結束程式
        self.port = PortHandler('/dev/U2D2')
        
        try:
            # TODO: 設定鮑率，成功後印出 '[ OK ] AICAR start'
            self.port.openPort()
            self.port.setBaudRate(BAUD_RATE)
            self.get_logger().info('[OK] AICAR start')
            pass
        except Exception as e:
            self.get_logger().error(f'{e}')
            raise SystemExit


        # TODO: 建立 Arm_Controller 與 Wheel_Controller 實例
        self.arm   = Arm_Controller(self.port)
        self.wheel = Wheel_Controller(self.port)


        # --- Subscribers ---
        # TODO: 建立以下五個 Subscriber，綁定對應的 callback
        #   /reset        (Empty)            → cb_reset
        #   /torque       (Int8)             → cb_torque
        #   /tool         (Int32)            → cb_tool
        #   /joint        (Int32MultiArray)  → cb_joint
        #   /cmd_vel      (Twist)            → cb_cmd
        self.create_subscription(Empty, '/reset', self.cb_reset, 1)
        self.create_subscription(Int8, '/torque', self.cb_torque, 1)
        self.create_subscription(Int32, '/tool', self.cb_tool, 1)
        self.create_subscription(Int32MultiArray, '/joint', self.cb_joint, 1)
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd, 1)


        # --- Publishers ---
        # TODO: 建立以下兩個 Publisher
        #   /joint_states  (JointState)         queue=1
        #   /diff_rad      (Float32MultiArray)  queue=1
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 1)
        self.diff_rad_pub     = self.create_publisher(Float32MultiArray, '/diff_rad', 1)


        # --- JointState 初始化 ---
        self.joint_states = JointState()
        self.joint_states.name = [
            'wheel_left_joint',
            'wheel_right_joint',
            'wheel_rear_left_joint',
            'wheel_rear_right_joint',
            'arm_joint_1',
            'arm_joint_2',
            'tool_joint'
        ]


        # --- 狀態變數 ---
        self.torque_data = 0
        self.tool_data   = 0.0
        self.joint_data  = [0.0, 0.0]
        self.cmd_data    = [0.0, 0.0]


        self.reset_received  = False
        self.torque_received = False
        self.tool_received   = False
        self.joint_received  = False
        self.cmd_received    = False


        self.state_is_reset    = False
        self.last_left         = self.last_right         = 0
        self.last_rear_left    = self.last_rear_right    = 0
        self.diff_left         = self.diff_right         = 0
        self.diff_rear_left    = self.diff_rear_right    = 0
        self.total_left        = self.total_right        = 0
        self.total_rear_left   = self.total_rear_right   = 0


        # --- 主迴圈 Timer (30 Hz) ---
        # TODO: 建立一個 30 Hz 的 Timer，callback 為 cb_timer
        self.create_timer( 1 / 30, self.cb_timer)
        pass


    # ── Subscriber Callbacks ────────────────────────────────


    def cb_reset(self, msg):
        """
        TODO:
        1. 印出 '[ SET ] AICAR reset'
        2. 將 total / last 里程計歸零
        3. 設定 state_is_reset = False
        4. 設定 reset_received  = True
        """
        self.get_logger().info('[ SET ] AICAR reset')
        self.total_left = self.total_right = self.total_rear_left = self.total_rear_right = 0
        self.last_left = self.last_right = self.last_rear_left = self.last_rear_right = 0
        self.state_is_reset = False
        self.reset_received = True
        pass


    def cb_torque(self, msg):
        """
        TODO:
        1. 印出 '[ SET ] ARM Torque : {msg.data}'
        2. 儲存 torque_data
        3. 設定 torque_received = True
        """
        self.get_logger().info(f'[ SET ] ARM Torque : {msg.data}')
        self.torque_data = msg.data
        self.torque_received = True
        pass


    def cb_tool(self, msg):
        """
        TODO: 只在 tool_received 為 False 時處理：
        1. 印出 '[ SET ] ARM Tool : {msg.data:.2f}'
        2. 儲存 tool_data
        3. 設定 tool_received = True
        """
        if not self.tool_received:
            self.get_logger().info(f'[ SET ] ARM Tool : {msg.data:.2f}')
            self.tool_data = msg.data
            self.tool_received = True
        pass


    def cb_joint(self, msg):
        """
        TODO: 只在 joint_received 為 False 時處理：
        1. 印出 '[ SET ] ARM Joint : {msg.data[0]:.2f}, {msg.data[1]:.2f}'
        2. 儲存 joint_data
        3. 設定 joint_received = True
        """
        self.get_logger().info(f'[ SET ] ARM Joint : {msg.data[0]:.2f}, {msg.data[1]:.2f}')
        self.joint_data[0] = msg.data[0]
        self.joint_data[1] = msg.data[1]
        self.joint_received = True
        pass


    def cb_cmd(self, msg):
        msg.linear.x
        msg.angular.z
        """
        TODO: 只在 cmd_received 為 False 時處理：
        1. 從 msg.linear.x 與 msg.angular.z 計算差速驅動的左右輪速度：
               left_vel  = lin_vel - ang_vel * WHEEL_SEPERATION / 2
               right_vel = lin_vel + ang_vel * WHEEL_SEPERATION / 2
        2. 將速度換算成 Dynamixel 數值 (乘以 VELOCITY2VALUE / 0.033)
        3. 用 constrain() 限制數值範圍
        4. 印出 '[ SET ] WHEEL : {int(left_vel)}, {int(right_vel)}'
        5. 儲存 cmd_data，設定 cmd_received = True
        """
        if not self.cmd_received:
            lin_vel = msg.linear.x
            ang_vel = msg.angular.z


            left_vel = lin_vel - (ang_vel * WHEEL_SEPERATION / 2)
            right_vel = lin_vel + (ang_vel * WHEEL_SEPERATION / 2)


            left_vel = left_vel * (VELOCITY2VALUE / 0.033)
            right_vel = right_vel * (VELOCITY2VALUE / 0.033)


            left_vel = constrain(left_vel)
            right_vel = constrain(right_vel)


            self.get_logger().info(f'[ SET ] WHEEL : {int(left_vel)}, {int(right_vel)}')


            self.cmd_data[0] = left_vel
            self.cmd_data[1] = right_vel


            self.cmd_received = True


        pass


    # ── 主 Timer Callback ───────────────────────────────────


    def cb_timer(self):
        """
        TODO (依序完成):
        1. 呼叫 fn_update_states() 更新感測器狀態
        2. 發布 /joint_states (包含 7 個關節位置)
        3. 發布 /diff_rad     (left, right 差值)
        4. 若 tool_received  → 執行 arm.set_tool()    並清旗標
        5. 若 joint_received → 執行 arm.set_joint()   並清旗標
        6. 若 cmd_received   → 執行 wheel.set_wheel() 並清旗標
        7. 若 reset_received → 執行 arm.reset() 與 wheel.reset()，
                               建立一個 2 秒後觸發 cb_flag 的 Timer，並清旗標
        8. 若 torque_received→ 執行 arm.set_torque()  並清旗標
        """
        if self.reset_received:
            self.arm.reset()
            self.wheel.reset()
            self.flag_timer = self.create_timer(2, self.cb_flag)
            self.reset_received = False


        if self.tool_received:
            self.arm.set_tool(self.tool_data)
            self.tool_received = False


        if self.joint_received:
            self.arm.set_joint([self.joint_data[0], self.joint_data[1]])
            self.joint_received = False


        if self.torque_received:
            self.arm.set_torque(self.torque_data)
            self.wheel.set_torque(self.torque_data)
            self.torque_received = False


        if self.cmd_received:
            self.wheel.set_wheel(self.cmd_data)
            self.cmd_received = False
        pass


    def cb_flag(self):
        """
        TODO: 將 state_is_reset 設為 False。
        """
        self.state_is_reset = False
        self.flag_timer.cancel()
        self.flag_timer.destroy()
        pass


    # ── 功能函式 ────────────────────────────────────────────


    def fn_update_states(self):
        """
        TODO:
        1. 從 arm.get_arm()   取得 arm_states
        2. 從 wheel.get_wheel() 取得 wheel_states
        3. 若 state_is_reset 為 False，將四個 last_* 初始化為當前讀值，
           並設 state_is_reset = True
        4. 對四個輪子分別計算：
               diff  = (current - last) * XM_TICK2RAD
               若 |diff| <= 2 * XM_TICK2RAD 則視為雜訊，設為 0
               total += diff
               更新 last
        """
        pass


    def fn_shutdown(self):
        """
        TODO:
        1. 印出 'Exiting ...'
        2. 呼叫 arm.reset()
        3. 呼叫 wheel.set_wheel([0, 0])
        4. 等待 3 秒
        5. 關閉 arm 與 wheel 的 torque
        6. 關閉 dxl_port
        """
        pass




def main():
    rclpy.init()
    node = AICAR_BASE()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.fn_shutdown()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()




if __name__ == '__main__':
    main()
