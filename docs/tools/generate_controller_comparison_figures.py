#!/usr/bin/env python3
"""Generate explanatory figures from the current controller equations; no measured data."""
from pathlib import Path
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/lio-controller-comparison-mpl')
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch, Rectangle

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'images' / 'controller_comparison'
FONT = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
if FONT.exists():
    font_manager.fontManager.addfont(str(FONT))
    family = font_manager.FontProperties(fname=str(FONT)).get_name()
else:
    family = 'sans-serif'
plt.rcParams.update({'font.family': family, 'font.size': 12,
                     'axes.unicode_minus': False, 'svg.fonttype': 'path',
                     'savefig.facecolor': '#f8fafc', 'figure.facecolor': '#f8fafc'})
INK, MUTED, OLD, CAR, GRID = '#162333', '#56677c', '#b96519', '#087e8b', '#dce4ed'

def title(fig, number, heading, sub):
    fig.text(.055, .94, f'{number}  |  {heading}', fontsize=22, weight='bold', color=INK)
    fig.text(.055, .89, sub, fontsize=11, color=MUTED)

def footer(fig, text):
    fig.text(.055, .035, text, fontsize=10, color=MUTED)

def save(fig, name):
    OUT.mkdir(parents=True, exist_ok=True)
    fig.savefig(OUT / f'{name}.png', dpi=170)
    fig.savefig(OUT / f'{name}.svg')
    plt.close(fig)

def box(ax, x, y, w, h, text, color, fill='white', size=13):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0.012,rounding_size=0.018',
                               linewidth=1.3, edgecolor=color, facecolor=fill))
    ax.text(x+w/2, y+h/2, text, ha='center', va='center', color=INK, fontsize=size, linespacing=1.55)

def arrow(ax, start, end, color, lw=2):
    ax.annotate('', xy=end, xytext=start,
                arrowprops={'arrowstyle':'-|>', 'color':color, 'lw':lw, 'mutation_scale':17})

def flow():
    fig = plt.figure(figsize=(14,9))
    title(fig, '01', '控制链路：横向偏差在哪里被处理？', '同一条 B-spline 与同一个差速底盘，两个控制器采用不同的误差处理方式。')
    ax = fig.add_axes([.055,.105,.89,.71]); ax.set_xlim(0,1); ax.set_ylim(0,1); ax.axis('off')
    lanes = [(.015, OLD, 'LEGACY｜时间参考跟踪', [
        '按时间取参考位置与速度\np_d(t)、v_d(t)',
        '位置误差 → 二维速度 u_w\n轨迹朝向误差 → 角速度 ω',
        '二维速度转到车体系\n差速配置把横向分量 v_y 截为 0',
        '输出前进速度 v 与角速度 ω\n未暂停时，参考时间继续推进']),
        (.535, CAR, 'CAR｜空间路径跟踪', [
        '实测位置 → 最近投影 s*\n在上一进度附近向前搜索',
        '沿路径选前视点 (x_L, y_L)\n横向偏差直接进入转弯曲率',
        'κ = 2y_L / (x_L² + y_L²)\n按曲率限速，ω = vκ',
        '输出 v、ω 与实测进度\n大角度转向；异常输入停车'])]
    for x,color,heading,items in lanes:
        ax.text(x+.215, .98, heading, color=color, fontsize=17, weight='bold', ha='center')
        for y,text in zip([.74,.52,.30,.08],items):
            box(ax,x,y,.43,.15,text,color)
            if y>.1: arrow(ax,(x+.215,y-.008),(x+.215,y-.065),color)
    footer(fig, '图示对应当前仓库实现。Legacy 的横向截断结论以 max_vy = 0 的差速配置为前提。')
    save(fig,'01_control_flow')

def robot(ax,x,y,color):
    ax.add_patch(Rectangle((x-.09,y-.055),.18,.11,facecolor=color,edgecolor='white',lw=1.5,zorder=7))
    arrow(ax,(x,y),(x+.22,y),color)

def geometry():
    fig=plt.figure(figsize=(14,8.5))
    title(fig,'02','同样偏离路径 0.3 m：为什么 Car 会转回去？','示例设车头沿 +x 方向，前视点在车前方 0.4 m；只展示当前周期的几何关系。')
    for idx,(color,heading) in enumerate([(OLD,'Legacy：侧向修正被截掉'),(CAR,'Car：侧向偏差转换成转向')]):
        ax=fig.add_axes([.07+idx*.49,.27,.40,.49])
        ax.set_facecolor('white'); ax.set_xlim(-.15,1.35); ax.set_ylim(-.25,.65); ax.set_aspect('equal')
        ax.spines[['top','right']].set_visible(False)
        ax.grid(color=GRID,alpha=.55); ax.tick_params(labelsize=10)
        ax.set_xlabel('x / m'); ax.set_ylabel('y / m'); ax.set_title(heading,color=color,fontsize=16,pad=16)
        ax.plot([-.15,1.35],[0,0],color=MUTED,lw=2,ls='--')
        ax.text(.94,.025,'目标路径 y=0',fontsize=11,color=MUTED)
        robot(ax,0,.3,color)
        ax.text(-.10,.43,'车体：θ=0',fontsize=11,color=color)
        ax.plot([0,0],[0,.3],ls=':',color=MUTED,lw=1.5)
        ax.text(.025,-.13,'偏差 0.3 m',fontsize=10,color=MUTED)
        if idx==0:
            ax.plot([.24,1.2],[.3,.3],color=color,lw=3,ls='--')
            arrow(ax,(.95,.3),(1.23,.3),color)
            ax.text(.38,.37,'仍平行前进',color=color,fontsize=12)
            arrow(ax,(.18,.28),(.18,.08),'#d83b39',1.8)
            ax.plot([.12,.24],[.13,.23],color='#d83b39',lw=2.5)
            ax.plot([.12,.24],[.23,.13],color='#d83b39',lw=2.5)
            ax.text(.28,.16,'横移指令被置零',color='#b32929',fontsize=10)
        else:
            ax.scatter([.4],[0],s=65,color=CAR,zorder=6)
            ax.text(.43,-.09,'前视点',color=CAR,fontsize=11)
            ax.plot([0,.4],[.3,0],color=CAR,ls=':',lw=1.5)
            k=-2*.3/(.4**2+.3**2)
            radius=abs(1/k); center_y=.3+1/k
            phi=np.linspace(np.pi/2,np.arctan2(-center_y,.4),100)
            ax.plot(radius*np.cos(phi),center_y+radius*np.sin(phi),color=CAR,lw=3)
            ax.annotate('当前追踪圆弧',xy=(.25,center_y+np.sqrt(radius**2-.25**2)),
                        xytext=(.58,.38),color=CAR,fontsize=11,
                        arrowprops={'arrowstyle':'->','color':CAR})
        label = ('u_y = −0.3 K_p  →  v_y = 0\n航向误差为 0  →  ω = 0' if idx==0 else
                 'κ = 2(−0.3)/(0.4²+0.3²) = −2.4 /m\n若 v = 0.2 m/s，则 ω = −0.48 rad/s')
        fig.text(.075+idx*.49,.155,label,color=color,fontsize=13,linespacing=1.8)
    footer(fig,'概念示意，不是导航日志。Car 会在每个控制周期重新计算前视点；图中的单个圆弧不是完整行驶轨迹。')
    save(fig,'02_lateral_correction')

def speed():
    fig=plt.figure(figsize=(14,8.5))
    title(fig,'03','转得越急，允许的前进速度越低','曲率 κ = 1/R。Car 联合约束 v 与 ω，避免只截断角速度而把实际转弯半径放大。')
    ax=fig.add_axes([.47,.22,.47,.57]); ax.set_facecolor('white')
    k=np.linspace(0,8,500)
    omega_limit=np.divide(1.0,k,out=np.full_like(k,np.inf),where=k>0)
    lateral_limit=np.sqrt(np.divide(.4,k,out=np.full_like(k,np.inf),where=k>0))
    v=np.minimum.reduce([np.full_like(k,.5),omega_limit,lateral_limit])
    ax.plot(k,np.full_like(k,.75),color=OLD,ls='--',lw=2,label='Legacy 配置的线速度上限 0.75')
    ax.plot(k,np.minimum(omega_limit,1.0),color=MUTED,ls=':',lw=2,label='角速度可行边界：v = 1 / |κ|')
    ax.plot(k,v,color=CAR,lw=3,label='Car 曲率相关速度上界')
    ax.fill_between(k,0,v,color=CAR,alpha=.10)
    ax.scatter([2],[np.sqrt(.2)],color=CAR,s=55,zorder=5)
    ax.annotate('R=0.5 m：v ≤ 0.447 m/s',xy=(2,np.sqrt(.2)),xytext=(2.7,.55),
                fontsize=11,color=CAR,arrowprops={'arrowstyle':'->','color':CAR})
    ax.set(xlim=(0,8),ylim=(0,.85),xlabel='转弯曲率 |κ| / (1/m)',ylabel='前进速度 v / (m/s)')
    ax.spines[['top','right']].set_visible(False); ax.grid(color=GRID)
    ax.legend(loc='upper right',fontsize=9,frameon=True)
    fig.text(.055,.75,'先满足两条物理关系',fontsize=17,weight='bold',color=INK)
    fig.text(.065,.65,r'$\omega = v\kappa$',fontsize=25,color=CAR)
    fig.text(.065,.54,r'$a_{\mathrm{lat}} = v^2|\kappa|$',fontsize=23,color=CAR)
    fig.text(.055,.38,'当前 Car 配置\nv_max = 0.5 m/s\nω_max = 1.0 rad/s\na_lat,max = 0.4 m/s²',fontsize=13,color=INK,linespacing=1.7)
    footer(fig,'图中曲线是简化速度上界，未包含参考速度、航向误差、横向误差、终点制动及变化率限制；不是实测速度对比。')
    save(fig,'03_curve_speed_limits')

def progress():
    fig=plt.figure(figsize=(14,8.5))
    title(fig,'04','小车暂时没前进时，参考进度应该怎么办？','假设第 2–4 秒实际未前进，且未触发 Legacy 的大航向误差暂停；路径总参考时长取 8 秒。')
    ax=fig.add_axes([.075,.29,.85,.48]); ax.set_facecolor('white')
    t=np.linspace(0,10,501)
    legacy=np.minimum(t,8)
    measured=np.where(t<2,t,np.where(t<4,2,t-2))
    ax.axvspan(2,4,color='#e6ebf0',alpha=.9)
    ax.plot(t,legacy,color=OLD,lw=2.5,ls='--',label='Legacy：未暂停时按时间推进')
    ax.plot(t,measured,color=CAR,lw=3,label='Car：反馈实测位置对应的进度')
    ax.axvline(8,color=OLD,ls=':',alpha=.5); ax.axvline(10,color=CAR,ls=':',alpha=.5)
    ax.text(3,.6,'实际位置不变',ha='center',color=MUTED,fontsize=11)
    ax.annotate('参考到末端，实际还落后',xy=(8,8),xytext=(5.2,8.6),color=OLD,fontsize=11,
                arrowprops={'arrowstyle':'->','color':OLD})
    ax.annotate('实际进度到末端',xy=(10,8),xytext=(8.5,4.3),color=CAR,fontsize=11,
                arrowprops={'arrowstyle':'->','color':CAR})
    ax.set(xlim=(0,10.4),ylim=(0,9),xlabel='经过时间 / s',ylabel='对应的轨迹参数时间 / s')
    ax.spines[['top','right']].set_visible(False); ax.grid(color=GRID)
    ax.legend(loc='upper left',fontsize=11)
    fig.text(.075,.15,'Legacy：t_ref ← t_ref + Δt（未暂停时）',color=OLD,fontsize=13)
    fig.text(.53,.15,'Car：t_feedback = t(s*)',color=CAR,fontsize=13)
    footer(fig,'概念示意。Car 沿用 B-spline 时间参数表示进度，但由空间投影决定其值；末端还需要满足位置到达条件。')
    save(fig,'04_execution_progress')

if __name__=='__main__':
    flow(); geometry(); speed(); progress()
    print(f'Generated four PNG/SVG figure pairs in {OUT}')
