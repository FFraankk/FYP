#!/usr/bin/env python3
import tkinter as tk
import rospy
from geometry_msgs.msg import Twist
import math
import threading

class MoveBaseGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("ROS 移动平台指令可视化")
        self.root.geometry("400x500")
        
        # 数据初始化
        self.lx = 0.0
        self.ly = 0.0
        self.az = 0.0
        self.last_update = rospy.Time(0)
        
        # --- UI 布局 ---
        self.canvas = tk.Canvas(root, width=300, height=300, bg="white", highlightthickness=2, highlightbackground="#333")
        self.canvas.pack(pady=20)
        
        self.info_label = tk.Label(root, text="等待 ROS 数据...", font=("Consolas", 12), justify="left")
        self.info_label.pack(pady=10)
        
        self.status_label = tk.Label(root, text="无信号", font=("Helvetica", 14, "bold"), fg="red")
        self.status_label.pack()

        # 绘制坐标系
        self.draw_axes()
        
        # 启动定时检查（处理超时）
        self.update_ui()

    def draw_axes(self):
        # 绘制雷达底图
        self.canvas.create_oval(50, 50, 250, 250, outline="#ddd")
        self.canvas.create_oval(100, 100, 200, 200, outline="#eee")
        self.canvas.create_line(150, 20, 150, 280, fill="#ccc", dash=(4, 4)) # X轴
        self.canvas.create_line(20, 150, 280, 150, fill="#ccc", dash=(4, 4)) # Y轴
        self.canvas.create_text(150, 15, text="前 (+X)", fill="gray")
        self.canvas.create_text(285, 150, text="右 (+Y)", fill="gray", anchor="w")

    def callback(self, msg):
        self.lx = msg.linear.x
        self.ly = msg.linear.y
        self.az = msg.angular.z
        self.last_update = rospy.Time.now()

    def update_ui(self):
        # 检查是否超时 (1秒)
        now = rospy.Time.now()
        if (now - self.last_update).to_sec() > 1.0:
            self.status_label.config(text="⚠️ 没有消息 (Timeout)", fg="red")
            self.canvas.delete("vector") # 清除箭头
        else:
            self.status_label.config(text="✅ 接收中 (Live)", fg="#2ecc71")
            self.draw_vector()

        # 更新文字数值
        self.info_label.config(text=f"Linear X:  {self.lx:5.2f}\nLinear Y:  {self.ly:5.2f}\nAngular Z: {self.az:5.2f}")
        
        # 每100毫秒刷新一次
        self.root.after(100, self.update_ui)

    def draw_vector(self):
        self.canvas.delete("vector")
        cx, cy = 150, 150  # 中心点
        
        # 缩放比例：假设 1.0 m/s 对应 100 像素
        scale = 100
        # 注意：Canvas坐标系 Y轴向下，而 ROS X轴向上，所以要做映射
        # ROS X -> Canvas 上 (-Y)
        # ROS Y -> Canvas 右 (+X)
        target_x = cx + (self.ly * scale) 
        target_y = cy - (self.lx * scale)
        
        # 限制长度
        dist = math.sqrt((target_x-cx)**2 + (target_y-cy)**2)
        if dist > 140: # 防止超出画布
            ratio = 140 / dist
            target_x = cx + (target_x - cx) * ratio
            target_y = cy + (target_y - cy) * ratio

        # 绘制箭头 (线速度方向)
        if abs(self.lx) > 0.01 or abs(self.ly) > 0.01:
            self.canvas.create_line(cx, cy, target_x, target_y, 
                                    fill="red", width=4, arrow=tk.LAST, tags="vector")
        
        # 绘制旋转环 (角速度方向)
        if abs(self.az) > 0.05:
            direction = -1 if self.az > 0 else 1 # ROS Z轴向上为逆时针
            extent = self.az * 50 # 旋转弧度可视化
            self.canvas.create_arc(120, 120, 180, 180, start=90, extent=extent, 
                                   outline="blue", width=3, style=tk.ARC, tags="vector")

def ros_thread(node):
    rospy.Subscriber("/cmd_vel", Twist, node.callback)
    rospy.spin()

if __name__ == "__main__":
    rospy.init_node('tk_move_base_viz')
    
    root = tk.Tk()
    app = MoveBaseGUI(root)
    
    # 开启 ROS 订阅线程，避免阻塞 Tkinter 主循环
    t = threading.Thread(target=ros_thread, args=(app,))
    t.daemon = True
    t.start()
    
    root.mainloop()