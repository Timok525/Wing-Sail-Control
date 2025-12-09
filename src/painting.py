import pandas as pd
import matplotlib.pyplot as plt
import numpy as np  # 新增numpy用于生成阈值线

# 解决中文显示问题
plt.rcParams["font.sans-serif"] = ["SimHei"]    # 中文黑体
plt.rcParams["axes.unicode_minus"] = False      # 解决负号显示问题

# -------------------------- 1. 读取CSV数据 --------------------------
csv_path = "D:/fanyi/Wing-Sail-Control-main-v1.0/src/output/simulation_results_alpha_converge.csv"
try:
    df = pd.read_csv(csv_path, encoding="GB2312")
    print("✅ CSV数据读取成功！")
    print(f"📊 数据维度：{df.shape[0]}行 × {df.shape[1]}列")
except FileNotFoundError:
    print(f"❌ 错误：未找到文件 {csv_path}，请检查路径！")
    exit()

# -------------------------- 2. 提取数据列并定义参数 --------------------------
t = df["时间(s)"]                  # 时间轴（对应新代码的t）
phi_deg = df["phi(deg)"]          # Phi角（角度）
alpha_deg = df["alpha(deg)"]      # Alpha角（角度）
beta_deg = df["beta(deg)"]        # Beta角（角度）
delta_actual_deg = df["delta_actual(deg)"]  # 实际舵偏角（含噪声）
delta_delayed_deg = df["delta_delayed(rad)"] * 180 / np.pi  # 延迟舵偏角转角度（用np.pi更精确）

# 定义目标值和阈值参数
phi_target_deg = 0.0  # Phi目标角
alpha_des_deg = 0.0   # Alpha目标角（可根据实际需求调整）
beta_des_deg = 0.0    # Beta目标角（可根据实际需求调整）
delta_max_deg = 30.0  # Delta硬边界阈值（可根据数据范围调整）
delta_soft_deg = 25.0 # Delta软边界阈值

# -------------------------- 3. 绘制4×1子图 --------------------------
fig, axes = plt.subplots(4, 1, figsize=(10, 12), dpi=100)
fig.suptitle("delta主导控制（phi稳定收敛，无持续波动）", fontsize=16, fontweight="bold")

# 子图1：phi角跟踪（新增阈值线）
axes[0].plot(t, phi_deg, 'b-', linewidth=1.5, label='实际phi角')
axes[0].plot(t, phi_target_deg * np.ones_like(t), 'r--', linewidth=1.2, label=f'目标phi角({phi_target_deg}°)')
axes[0].plot(t, 25 * np.ones_like(t), 'g:', linewidth=1.2, label='25°阈值')
axes[0].plot(t, -25 * np.ones_like(t), 'g:')
axes[0].plot(t, 7 * np.ones_like(t), 'b--', linewidth=1.0, label='7°范围')
axes[0].plot(t, -7 * np.ones_like(t), 'b--')
axes[0].set_xlabel('时间(s)')
axes[0].set_ylabel('phi角(°)')
axes[0].set_title(f'phi角跟踪（目标：{phi_target_deg}°，稳定收敛）')
axes[0].legend()
axes[0].grid(True, alpha=0.3)

# 子图2：alpha角变化（新增目标线和边界）
axes[1].plot(t, alpha_deg, 'g-', linewidth=1.2, label='实际alpha角')
axes[1].plot(t, alpha_des_deg * np.ones_like(t), 'm--', linewidth=1.2, label=f'目标alpha角({alpha_des_deg}°)')
axes[1].plot(t, 45 * np.ones_like(t), 'r:', linewidth=1.2, label='±45°边界')
axes[1].plot(t, -45 * np.ones_like(t), 'r:')
axes[1].set_xlabel('时间(s)')
axes[1].set_ylabel('alpha角(°)')
axes[1].set_title('alpha角变化（无干扰）')
axes[1].legend()
axes[1].grid(True, alpha=0.3)

# 子图3：beta角变化（新增目标线和边界）
axes[2].plot(t, beta_deg, color='orange', linewidth=1.2, label='实际beta角')
axes[2].plot(t, beta_des_deg * np.ones_like(t), 'k--', linewidth=1.2, label=f'目标beta角({beta_des_deg}°)')
axes[2].plot(t, 45 * np.ones_like(t), 'r:', linewidth=1.2, label='±45°边界')
axes[2].plot(t, -45 * np.ones_like(t), 'r:')
axes[2].set_xlabel('时间(s)')
axes[2].set_ylabel('beta角(°)')
axes[2].set_title('beta角变化（无干扰）')
axes[2].legend()
axes[2].grid(True, alpha=0.3)

# 子图4：delta舵偏角对比（新增边界线）
axes[3].plot(t, delta_actual_deg, color='#CC3333', linewidth=1.2, label='实际delta（含噪声）')
axes[3].plot(t, delta_delayed_deg, 'c-', linewidth=1.2, label='延迟delta')  # 保留原延迟舵偏角对比
axes[3].plot(t, delta_max_deg * np.ones_like(t), 'b--', linewidth=1.0, label=f'硬边界(±{delta_max_deg}°)')
axes[3].plot(t, delta_soft_deg * np.ones_like(t), 'c--', linewidth=1.0, label=f'软边界(±{delta_soft_deg}°)')
axes[3].plot(t, np.zeros_like(t), 'k:', linewidth=1.0, label='零位')
axes[3].plot(t, -delta_max_deg * np.ones_like(t), 'b--')
axes[3].plot(t, -delta_soft_deg * np.ones_like(t), 'c--')
axes[3].set_xlabel('时间(s)')
axes[3].set_ylabel('delta舵偏角(°)')
axes[3].set_title(f'delta舵偏角（稳定收敛，限制：±{delta_max_deg:.1f}°）')
axes[3].legend()
axes[3].grid(True, alpha=0.3)

# -------------------------- 4. 保存与显示 --------------------------
plt.tight_layout()  # 自动调整子图间距（避免标题重叠）
plt.savefig('simulation_result_4x1.png', dpi=300, bbox_inches='tight')
print("✅ 结果已保存为 simulation_result_4x1.png")
plt.show()