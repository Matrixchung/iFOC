import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib

# 配置matplotlib支持中文显示
plt.rcParams['font.sans-serif'] = ['SimHei']  # 用来正常显示中文标签
plt.rcParams['axes.unicode_minus'] = False  # 用来正常显示负号

# 定义常量
SQRT_3div2 = math.sqrt(3) / 2

# 辅助函数
def _constrain(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def FOC_SVPWM_2(Uq, Ud, angle_rad, Udc):
    Udc_divSQRT_3 = Udc / math.sqrt(3)
    Ud = _constrain(Ud, -Udc_divSQRT_3, Udc_divSQRT_3)
    Uq_limited = math.sqrt(Udc_divSQRT_3 * Udc_divSQRT_3 - Ud * Ud)
    Uq = _constrain(Uq, -Uq_limited, Uq_limited)
    
    Ualpha = Ud * math.cos(angle_rad) - Uq * math.sin(angle_rad)
    Ubeta = Uq * math.cos(angle_rad) + Ud * math.sin(angle_rad)
    Ualpha_Pu = Ualpha / Vbus
    Ubeta_Pu = Ubeta / Vbus
    
    Uabc_Pu = [Ualpha_Pu, 0.0, 0.0]
    
    Ualpha_Pu *= -0.5
    Ubeta_Pu *= SQRT_3div2
    
    Uabc_Pu[1] = Ualpha_Pu + Ubeta_Pu
    Uabc_Pu[2] = Ualpha_Pu - Ubeta_Pu
    
    Ucom_Pu = 0.5 * (max(Uabc_Pu) + min(Uabc_Pu))
    
    tX = [
        Uabc_Pu[0] - Ucom_Pu + 0.5,
        Uabc_Pu[1] - Ucom_Pu + 0.5,
        Uabc_Pu[2] - Ucom_Pu + 0.5
    ]

    return tX

# 创建图形
fig = plt.figure(figsize=(15, 10))
fig.suptitle('FOC SVPWM 可视化 - 静止alpha-beta坐标系下的旋转矢量', fontsize=16)

# 创建子图
ax1 = fig.add_subplot(221)  # alpha-beta平面
ax2 = fig.add_subplot(222)  # 三相占空比
ax3 = fig.add_subplot(223)  # 三相电压波形
ax4 = fig.add_subplot(224, projection='polar')  # 极坐标图，显示dq矢量

# 设置参数
Uq = 24.0 / math.sqrt(3)  # q轴电压
Ud = -15.0   # d轴电压（通常设为0，表示最大转矩/电流比控制）
Vbus = 24.0  # 总线电压

Udc_divSQRT_3 = Vbus / math.sqrt(3)
Uq = _constrain(Uq, -Udc_divSQRT_3, Udc_divSQRT_3)
Ud = _constrain(Ud, -Udc_divSQRT_3, Udc_divSQRT_3)
Uq_limited = math.sqrt(Udc_divSQRT_3 * Udc_divSQRT_3 - Ud * Ud)
Uq = _constrain(Uq, -Uq_limited, Uq_limited)
voltage_amplitude = math.sqrt(Uq**2 + Ud**2)  # 电压幅值

# 初始化数据
angle_points = np.linspace(0, 2*np.pi, 360)  # 一个完整的电气周期
alpha_points = []
beta_points = []
duty_a_points = []
duty_b_points = []
duty_c_points = []

# 计算不同角度下的数据
for angle in angle_points:
    # Park逆变换
    alpha = Ud * math.cos(angle) - Uq * math.sin(angle)
    beta = Uq * math.cos(angle) + Ud * math.sin(angle)
    alpha_points.append(alpha / Vbus)  # 归一化
    beta_points.append(beta / Vbus)    # 归一化
    
    # 计算三相占空比
    duties = FOC_SVPWM_2(Uq, Ud, angle, Vbus)
    duty_a_points.append(duties[0])
    duty_b_points.append(duties[1])
    duty_c_points.append(duties[2])

# 绘制alpha-beta平面轨迹
ax1.plot(alpha_points, beta_points, 'b-')
ax1.set_title('Alpha-Beta平面轨迹')
ax1.set_xlabel('Alpha')
ax1.set_ylabel('Beta')
ax1.set_xlim(-1, 1)
ax1.set_ylim(-1, 1)
ax1.grid(True)
ax1.axhline(y=0, color='k', linestyle='-', alpha=0.3)
ax1.axvline(x=0, color='k', linestyle='-', alpha=0.3)

# 绘制六边形边界
hex_angles = np.linspace(0, 2*np.pi, 7)
hex_x = np.cos(hex_angles) * 2/3
hex_y = np.sin(hex_angles) * 2/3
ax1.plot(hex_x, hex_y, 'k--', alpha=0.5)

# 绘制三相占空比
ax2.plot(angle_points * 180/np.pi, duty_a_points, 'r-', label='相A占空比')
ax2.plot(angle_points * 180/np.pi, duty_b_points, 'g-', label='相B占空比')
ax2.plot(angle_points * 180/np.pi, duty_c_points, 'b-', label='相C占空比')
ax2.set_title('三相占空比波形')
ax2.set_xlabel('电角度 (度)')
ax2.set_ylabel('占空比')
ax2.set_xlim(0, 360)
ax2.set_ylim(0, 1)
ax2.grid(True)
ax2.legend()

# 绘制三相电压波形（相对于中点电压）
voltage_a = np.array(duty_a_points) * 2 - 1
voltage_b = np.array(duty_b_points) * 2 - 1
voltage_c = np.array(duty_c_points) * 2 - 1
ax3.plot(angle_points * 180/np.pi, voltage_a, 'r-', label='相A电压')
ax3.plot(angle_points * 180/np.pi, voltage_b, 'g-', label='相B电压')
ax3.plot(angle_points * 180/np.pi, voltage_c, 'b-', label='相C电压')
ax3.set_title('三相电压波形')
ax3.set_xlabel('电角度 (度)')
ax3.set_ylabel('相电压 (标幺值)')
ax3.set_xlim(0, 360)
ax3.set_ylim(-1, 1)
ax3.grid(True)
ax3.legend()

# 绘制dq矢量在极坐标系中的表示
ax4.plot(angle_points, [voltage_amplitude/Vbus] * len(angle_points), 'r-')
ax4.set_title('dq矢量旋转')
ax4.set_rticks([0.2, 0.4, 0.6, 0.8, 1.0])
ax4.set_rlim(0, 1)

# 添加dq分量显示
# 创建一个文本框显示dq分量
dq_text = ax4.text(0.5, 0.05, f'Ud: {Ud:.2f}\nUq: {Uq:.2f}', 
                  transform=ax4.transAxes, fontsize=10, 
                  bbox=dict(facecolor='white', alpha=0.7))

# 添加d轴和q轴的指示线（标准化后）
ud_norm = Ud / Vbus
uq_norm = Uq / Vbus
# 在极坐标系中绘制d轴分量
ax4.plot([0, 0], [0, abs(ud_norm)], 'g-', alpha=0.7, linewidth=2)
# 在极坐标系中绘制q轴分量（q轴与d轴垂直，角度相差π/2）
ax4.plot([np.pi/2, np.pi/2], [0, abs(uq_norm)], 'b-', alpha=0.7, linewidth=2)
# 添加图例
ax4.legend(['合成矢量', 'd轴分量', 'q轴分量'], loc='upper right')

# 添加动态点以显示当前位置
# 创建动画函数
def update(frame):
    # 当前角度
    angle = frame * 2 * np.pi / 100
    
    # 计算当前alpha-beta值
    alpha = Ud * math.cos(angle) - Uq * math.sin(angle)
    beta = Uq * math.cos(angle) + Ud * math.sin(angle)
    alpha_norm = alpha / Vbus
    beta_norm = beta / Vbus
    
    # 计算当前占空比
    duties = FOC_SVPWM_2(Uq, Ud, angle, Vbus)
    
    # 更新alpha-beta平面上的点
    alpha_beta_point.set_data([alpha_norm], [beta_norm])
    
    # 更新三相占空比图上的点
    duty_a_point.set_data([angle * 180/np.pi], [duties[0]])
    duty_b_point.set_data([angle * 180/np.pi], [duties[1]])
    duty_c_point.set_data([angle * 180/np.pi], [duties[2]])
    
    # 更新三相电压图上的点
    voltage_a_val = duties[0] * 2 - 1
    voltage_b_val = duties[1] * 2 - 1
    voltage_c_val = duties[2] * 2 - 1
    voltage_a_point.set_data([angle * 180/np.pi], [voltage_a_val])
    voltage_b_point.set_data([angle * 180/np.pi], [voltage_b_val])
    voltage_c_point.set_data([angle * 180/np.pi], [voltage_c_val])
    
    # 更新dq矢量点
    vector_point.set_data([angle], [voltage_amplitude/Vbus])
    
    # 更新dq分量文本
    # 根据当前角度计算旋转后的dq分量
    dq_text.set_text(f'Ud: {Ud:.2f}\nUq: {Uq:.2f}\n角度: {angle*180/np.pi:.0f}°')
    
    return alpha_beta_point, duty_a_point, duty_b_point, duty_c_point, voltage_a_point, voltage_b_point, voltage_c_point, vector_point, dq_text

# 初始化动态点
alpha_beta_point, = ax1.plot([], [], 'ro', markersize=8)
duty_a_point, = ax2.plot([], [], 'ro', markersize=8)
duty_b_point, = ax2.plot([], [], 'go', markersize=8)
duty_c_point, = ax2.plot([], [], 'bo', markersize=8)
voltage_a_point, = ax3.plot([], [], 'ro', markersize=8)
voltage_b_point, = ax3.plot([], [], 'go', markersize=8)
voltage_c_point, = ax3.plot([], [], 'bo', markersize=8)
vector_point, = ax4.plot([], [], 'ro', markersize=8)

# 创建动画
ani = FuncAnimation(fig, update, frames=100, interval=50, blit=True)

plt.tight_layout()
plt.show()

# 如果需要保存动画
# ani.save('foc_svpwm_visualization.gif', writer='pillow', fps=20)