import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
import tkinter as tk
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

# 配置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

class TrajController:
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.accel_time = 0.0
        self.decel_time = 0.0
        self.cruise_time = 0.0
        self.total_time = 0.0
        self.init_pos = 0.0
        self.init_speed = 0.0
        self.final_pos = 0.0
        self.start_cruise_pos = 0.0
        
        self.set_pos = 0.0
        self.set_speed = 0.0
        self.set_accel = 0.0
        self.task_done = True
        
        self.state_timer = 0.0
    
    def plan_trajectory(self, target_pos, current_pos, current_speed, cruise_speed, max_accel, max_decel):
        if max_accel < 0.0:
            max_accel = -max_accel
        if max_decel < 0.0:
            max_decel = -max_decel
        if cruise_speed < 0.0:
            cruise_speed = -cruise_speed
        if max_accel == 0.0 or max_decel == 0.0 or cruise_speed == 0.0:
            return
        
        self.task_done = False
        self.state_timer = 0.0
        dX = target_pos - current_pos
        min_stop_dist = (current_speed * current_speed) / (2.0 * max_decel)
        dX_stop = min_stop_dist if current_speed >= 0 else -min_stop_dist
        s = 1.0 if (dX - dX_stop) >= 0 else -1.0
        
        self.ref_accel = s * max_accel
        self.ref_decel = -s * max_decel
        self.ref_speed = s * cruise_speed
        
        print(f"ref_accel: {self.ref_accel}, ref_decel: {self.ref_decel}, ref_speed: {self.ref_speed}")

        if (s * current_speed) > (s * self.ref_speed):
            self.ref_accel = -self.ref_accel
        
        self.accel_time = (self.ref_speed - current_speed) / self.ref_accel
        self.decel_time = -self.ref_speed / self.ref_decel
        
        dX_min = 0.5 * self.accel_time * (self.ref_speed + current_speed) + 0.5 * self.decel_time * self.ref_speed
        
        if s * dX < s * dX_min:
            # 三角形速度曲线
            self.ref_speed = s * np.sqrt(max((self.ref_decel * current_speed * current_speed + 
                                         2.0 * self.ref_accel * self.ref_decel * dX) / 
                                        (self.ref_decel - self.ref_accel), 0.0))
            self.accel_time = max((self.ref_speed - current_speed) / self.ref_accel, 0.0)
            self.decel_time = max(-self.ref_speed / self.ref_decel, 0.0)
            self.cruise_time = 0.0
        else:
            # 梯形速度曲线
            self.cruise_time = (dX - dX_min) / self.ref_speed
        
        self.total_time = self.accel_time + self.cruise_time + self.decel_time
        self.init_pos = current_pos
        self.init_speed = current_speed
        self.final_pos = target_pos
        self.start_cruise_pos = current_pos + current_speed * self.accel_time + 0.5 * self.ref_accel * (self.accel_time * self.accel_time)

    def plan_trapezoidal_speed(self, current_speed, target_speed, time_to_accelerate):
        if time_to_accelerate <= 0.0:
            return
        self.reset()
        speed_delta = target_speed - current_speed
        self.ref_accel = speed_delta / time_to_accelerate
        self.init_speed = current_speed
        self.ref_speed = target_speed
        self.accel_time = time_to_accelerate
        self.cruise_time = 0.0
        self.decel_time = 0.0
    
    def get_state_at_time(self, t):
        if t < self.accel_time:  # 加速阶段
            pos = self.init_pos + self.init_speed * t + 0.5 * self.ref_accel * (t * t)
            speed = self.init_speed + self.ref_accel * t
            accel = self.ref_accel
        elif t < self.accel_time + self.cruise_time:  # 匀速阶段
            pos = self.start_cruise_pos + self.ref_speed * (t - self.accel_time)
            speed = self.ref_speed
            accel = 0.0
        elif t < self.total_time:  # 减速阶段
            td = t - self.total_time
            pos = self.final_pos + 0.5 * self.ref_decel * (td * td)
            speed = self.ref_decel * td
            accel = self.ref_decel
        else:  # 完成
            pos = self.final_pos
            speed = 0.0
            accel = 0.0
        
        return pos, speed, accel

class TrajectoryPlannerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("轨迹规划可视化工具")
        self.root.geometry("1200x800")
        
        self.controller = TrajController()
        
        # 创建UI框架
        self.create_ui()
        
    def create_ui(self):
        # 创建左侧参数输入区域
        input_frame = ttk.LabelFrame(self.root, text="参数设置")
        input_frame.pack(side=tk.LEFT, fill=tk.Y, padx=10, pady=10)
        
        # 创建参数输入字段
        param_labels = [
            "目标位置 (rad):", 
            "当前位置 (rad):", 
            "当前速度 (rad/s):", 
            "巡航速度 (rad/s):", 
            "最大加速度 (rad/s²):", 
            "最大减速度 (rad/s²):"
        ]
        
        self.param_entries = []
        
        for i, label in enumerate(param_labels):
            ttk.Label(input_frame, text=label).grid(row=i, column=0, sticky=tk.W, padx=5, pady=5)
            entry = ttk.Entry(input_frame, width=15)
            entry.grid(row=i, column=1, padx=5, pady=5)
            self.param_entries.append(entry)
        
        # 设置默认值
        default_values = ["10.0", "0.0", "0.0", "2.0", "1.0", "1.0"]
        for entry, value in zip(self.param_entries, default_values):
            entry.insert(0, value)
        
        # 添加规划按钮
        plan_button = ttk.Button(input_frame, text="开始规划", command=self.plan_and_visualize)
        plan_button.grid(row=len(param_labels), column=0, columnspan=2, pady=20)
        
        # 添加预设场景选择
        ttk.Label(input_frame, text="预设场景:").grid(row=len(param_labels)+1, column=0, sticky=tk.W, padx=5, pady=5)
        self.scenario_var = tk.StringVar()
        scenario_combo = ttk.Combobox(input_frame, textvariable=self.scenario_var, width=15)
        scenario_combo['values'] = ("标准梯形曲线", "三角形曲线", "非零初始速度")
        scenario_combo.current(0)
        scenario_combo.grid(row=len(param_labels)+1, column=1, padx=5, pady=5)
        scenario_combo.bind("<<ComboboxSelected>>", self.load_preset)
        
        # 创建右侧图表区域
        plot_frame = ttk.LabelFrame(self.root, text="轨迹可视化")
        plot_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 创建matplotlib图表
        self.fig = Figure(figsize=(8, 8))
        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 创建信息显示区域
        self.info_frame = ttk.LabelFrame(input_frame, text="规划信息")
        self.info_frame.grid(row=len(param_labels)+2, column=0, columnspan=2, sticky=tk.W+tk.E, padx=5, pady=10)
        
        self.info_text = tk.Text(self.info_frame, height=10, width=30, wrap=tk.WORD)
        self.info_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.info_text.config(state=tk.DISABLED)
    
    def load_preset(self, event=None):
        scenario = self.scenario_var.get()
        
        if scenario == "标准梯形曲线":
            values = ["10.0", "0.0", "0.0", "2.0", "1.0", "1.0"]
        elif scenario == "三角形曲线":
            values = ["2.0", "0.0", "0.0", "2.0", "1.0", "1.0"]
        elif scenario == "非零初始速度":
            values = ["8.0", "0.0", "1.0", "2.0", "1.0", "1.0"]
        
        for entry, value in zip(self.param_entries, values):
            entry.delete(0, tk.END)
            entry.insert(0, value)
    
    def plan_and_visualize(self):
        try:
            # 获取参数值
            target_pos = float(self.param_entries[0].get())
            current_pos = float(self.param_entries[1].get())
            current_speed = float(self.param_entries[2].get())
            cruise_speed = float(self.param_entries[3].get())
            max_accel = float(self.param_entries[4].get())
            max_decel = float(self.param_entries[5].get())
            
            # 规划轨迹
            self.controller.plan_trajectory(
                target_pos, current_pos, current_speed, cruise_speed, max_accel, max_decel
            )
            
            # 更新信息显示
            self.update_info()
            
            # 可视化轨迹
            self.visualize_trajectory()
            
        except ValueError as e:
            self.show_error(f"输入参数错误: {str(e)}")
    
    def update_info(self):
        # 更新信息显示区域
        info = (
            f"加速时间: {self.controller.accel_time:.2f} s\n"
            f"巡航时间: {self.controller.cruise_time:.2f} s\n"
            f"减速时间: {self.controller.decel_time:.2f} s\n"
            f"总时间: {self.controller.total_time:.2f} s\n"
            f"曲线类型: {'三角形速度曲线' if self.controller.cruise_time == 0 else '梯形速度曲线'}"
        )
        
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(tk.END, info)
        self.info_text.config(state=tk.DISABLED)
    
    def show_error(self, message):
        self.info_text.config(state=tk.NORMAL)
        self.info_text.delete(1.0, tk.END)
        self.info_text.insert(tk.END, f"错误: {message}")
        self.info_text.config(state=tk.DISABLED)
    
    def visualize_trajectory(self):
        # 清除现有图表
        self.fig.clear()
        
        # 生成时间点
        t_values = np.linspace(0, self.controller.total_time * 1.1, 1000)
        pos_values = []
        speed_values = []
        accel_values = []
        
        for t in t_values:
            pos, speed, accel = self.controller.get_state_at_time(t)
            pos_values.append(pos)
            speed_values.append(speed)
            accel_values.append(accel)
        
        # 创建子图
        gs = GridSpec(3, 1, figure=self.fig)
        
        # 位置图
        ax1 = self.fig.add_subplot(gs[0, 0])
        ax1.plot(t_values, pos_values, 'b-', linewidth=2)
        ax1.set_ylabel('位置 (rad)')
        ax1.set_title('轨迹规划可视化')
        ax1.grid(True)
        
        # 速度图
        ax2 = self.fig.add_subplot(gs[1, 0])
        ax2.plot(t_values, speed_values, 'g-', linewidth=2)
        ax2.set_ylabel('速度 (rad/s)')
        ax2.grid(True)
        
        # 加速度图
        ax3 = self.fig.add_subplot(gs[2, 0])
        ax3.plot(t_values, accel_values, 'r-', linewidth=2)
        ax3.set_xlabel('时间 (s)')
        ax3.set_ylabel('加速度 (rad/s²)')
        ax3.grid(True)
        
        # 标记各阶段
        if self.controller.accel_time > 0:
            ax1.axvline(x=self.controller.accel_time, color='k', linestyle='--', alpha=0.5)
            ax2.axvline(x=self.controller.accel_time, color='k', linestyle='--', alpha=0.5)
            ax3.axvline(x=self.controller.accel_time, color='k', linestyle='--', alpha=0.5)
        
        if self.controller.cruise_time > 0:
            ax1.axvline(x=self.controller.accel_time + self.controller.cruise_time, color='k', linestyle='--', alpha=0.5)
            ax2.axvline(x=self.controller.accel_time + self.controller.cruise_time, color='k', linestyle='--', alpha=0.5)
            ax3.axvline(x=self.controller.accel_time + self.controller.cruise_time, color='k', linestyle='--', alpha=0.5)
        
        self.fig.tight_layout()
        self.canvas.draw()

# 主函数
if __name__ == '__main__':
    root = tk.Tk()
    app = TrajectoryPlannerApp(root)
    root.mainloop()