#!/usr/bin/env python3

import sys, select, termios, tty, math, threading

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor


# ── 速度上限 ────────────────────────────────────────────────
MAX_LIN_VEL = 0.15
MAX_ANG_VEL = 4.5

# ── 手臂角度範圍 (度) ───────────────────────────────────────
MAX_J1 = 90;  MIN_J1 = 0
MAX_J2 = 90;  MIN_J2 = 0
MAX_TOOL = 45; MIN_TOOL = 0

# ── 每次按鍵的步進量 ────────────────────────────────────────
STEP_LIN = 0.01
STEP_ANG = 1.0
STEP_ARM = 2

msg = """
Control Your AICAR!
----------------------------------------------
Moving around:
            w(+)
   a(+)      s      d(-)
            x(-)
 
Arm control:
    Joint1 : u(+), j(-)
    Joint2 : i(+), k(-)
     Tool  : o(+), l(-)
 
h : Arm Home Pose
s : force stop
 
CTRL-C to quit
"""

def constrain(vel):
    if vel < -LIMIT_VEL: return -LIMIT_VEL
    if vel > LIMIT_VEL: return LIMIT_VEL
    return vel


class AICAR_TELEOP(Node):
    def __init__(self):
        super().__init__('aicar_teleop')

        # --- Subscribers ---
        # TODO: 建立以下兩個 Subscriber，綁定對應的 callback
        #   /joint_states  (JointState) → cb_joint_states
        #   /reset         (Empty)      → cb_reset

        # --- Publishers ---
        # TODO: 建立以下三個 Publisher
        #   /cmd_vel  (Twist)           queue=1  → self.cmd_pub
        #   /joint      (Int32MultiArray) queue=1  → self.joint_pub
        #   /tool     (Int32)           queue=1  → self.tool_pub
        self.cmd_pub  = None
        self.joint_pub = None
        self.tool_pub = None

        # --- 狀態變數 ---
        self.lin_vel = self.ang_vel = 0.0
        self.get_arm_data = False
        self.j1 = self.j2 = self.tool = 0

        self.settings = termios.tcgetattr(sys.stdin)

    # ── Subscriber Callbacks ────────────────────────────────

    def cb_joint_states(self, msg):
        """
        TODO: 只在 get_arm_data 為 False 時處理：
        1. 從 msg.position[4~6] 讀取弧度值，換算成整數角度（* 180 / pi）
           分別存入 self.j1, self.j2, self.tool
        2. 印出 '[ ARM  ] init : j1=..., j2=..., tool=...'
        3. 設定 get_arm_data = True
        """
        pass

    def cb_reset(self, msg):
        """
        TODO: 將 j1, j2, tool, lin_vel, ang_vel 全部歸零。
        """
        pass

    # ── 功能函式 ────────────────────────────────────────────

    def fn_get_key(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def fn_wheel_pub(self):
        """
        TODO:
        1. 印出 '[ WHEEL ] linear : %.2f, angular : %.2f'
        2. 建立 Twist 訊息，填入 lin_vel / ang_vel 並發布
        """
        pass

    def fn_joint_pub(self):
        """
        TODO:
        1. 印出 '[  ARM  ] j1 : %d, j2 : %d, tool : %d'
        2. 建立 Int32MultiArray，填入 [j1, j2] 並發布到 /joint
        3. 建立 Int32，填入 tool 並發布到 /tool
        """
        pass

    def fn_run(self):
        """
        TODO: 主鍵盤控制迴圈
        1. 印出操作說明 msg
        2. 進入 while True 迴圈，每次呼叫 fn_get_key() 取得按鍵
        3. 根據按鍵對應的動作（參考 msg 說明）：
               w / x  → lin_vel ± STEP_LIN，呼叫 fn_wheel_pub()
               a / d  → ang_vel ± STEP_ANG，呼叫 fn_wheel_pub()
               u / j  → j1      ± STEP_ARM，呼叫 fn_arm_pub()
               i / k  → j2      ± STEP_ARM，呼叫 fn_arm_pub()
               o / l  → tool    ± STEP_ARM，呼叫 fn_arm_pub()
               h      → j1=j2=tool=0，呼叫 fn_arm_pub()
               s      → lin_vel=ang_vel=0.0，呼叫 fn_wheel_pub()
               CTRL-C ('\x03') → 跳出迴圈
           每個有效按鍵讓 status += 1；status 達到 20 時重新印出 msg 並歸零
        4. finally 區塊：停止車輪並還原終端機設定
        """
        try:
            pass
        
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

        pass


def main():
    rclpy.init()
    node = AICAR_TELEOP()

    # 用 MultiThreadedExecutor 在背景執行 spin，讓主執行緒跑鍵盤迴圈
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    node.fn_run()

    node.destroy_node()
    if rclpy.ok(): rclpy.shutdown()


if __name__ == '__main__':
    main()