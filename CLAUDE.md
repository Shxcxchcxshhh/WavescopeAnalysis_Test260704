# CLAUDE.md — I2C Scope Analysis

## 环境

- MATLAB R2025a+ (通过 MCP 调用)
- 项目目录: `D:\GANGBANG\MATLABFiles\WavescopeAnalysis_Trial260629`
- 示波器: ZDS3024 Plus, 2GHz 采样
- 入口脚本: `i2c_full_analysis.m`

## CS V 格式

21行元数据 + 1行列标题 (`Time,CH1,CH2`) → 数据从第23行开始。
读取方式: `readmatrix` + `detectImportOptions('NumHeaderLines',22)`.
数据量级: 百万行级 CSV (本例~2.8M行, 2GHz采样=500ps/点)。

## I2C 解码关键坑

### 核心坑: 极性反转

`double(voltage < threshold)` 返回 1 表示电压低, 但 I2C 协议规定 1=高电平.
**所有后续逻辑必须以此为前提, 否则全盘皆错.**

| 操作 | 正确写法 | 错误写法 |
|------|---------|---------|
| I2C 逻辑值 | `i2c_bit = 1 - sda_d` | `sda_d` 直接当位值 |
| SCL 上升沿 | `diff(scl_d) < -0.5` (scl_d 1→0) | `diff > 0.5` (变成下降沿了) |
| START 条件 | sda_d 0→1 且 scl_d=0 | sda_d 1→0 (变 STOP 了) |
| STOP 条件 | sda_d 1→0 且 scl_d=0 | sda_d 0→1 (变 START 了) |

### 通道自动识别

不要用 START 条件数做评分 (噪声多时伪触发泛滥).
用 `debounced_SCL_edge_count - SDA_transition_count * 0.05` 打分.

### 重复起始条件 (RESTART)

用事件时间线法配对, 不能简单 START/STOP 1:1:
```
events = sort([START_idx; STOP_idx])
walk events: START开段, 再遇START=关旧段+RESTART, STOP关段
```
RESTART 后字节计数归零, 下一字节必为地址字节.

### 毛刺滤波

SCL 边沿间隔 <1μs 的是采样噪声, 必须去抖:
```matlab
clean = raw(1);
for i = 2:length(raw)
    if t(raw(i)) - t(clean(end)) > 1e-6
        clean(end+1) = raw(i);
    end
end
```

### 时钟频率

SCL 周期 = rise→rise (两相邻上升沿间距), 不是半周期.
本例: 周期 10.86μs → 92kHz (标准模式 100kHz 附近).

## 验证清单

- [ ] 运行前确认 `i2c_bit = 1 - sda_d` (不是直接用 sda_d)
- [ ] SCL 上升沿用 `diff < -0.5` (不是 `> 0.5`)
- [ ] RESTART 处字节独立解码, 不跨段拼接
- [ ] 通道识别用边沿计数评分, 不用 START 数
- [ ] 时钟频率用完整周期 (rise→rise)

## Python 环境 (pyqtgraph 方案)

- Python 3.12 (conda: `D:\GANGBANG\PythonFiles\PyltspiceConda`)
- 入口脚本: `i2c_automated_measure.py` (功能与 `i2c_automated_measure.m` 完全对齐)
- MATLAB 备份: `i2c_automated_measure.m.backup`
- 依赖: `pyqtgraph>=0.14`, `PyQt5>=5.15`, `numpy>=2.0`
- 安装: `pip install pyqtgraph PyQt5 numpy` (conda 环境已装)

### pyqtgraph 批量绘图关键坑

#### 1. Windows 下禁用 `QT_QPA_PLATFORM=offscreen`

```python
# 错误: 会导致 Segfault
os.environ["QT_QPA_PLATFORM"] = "offscreen"
# 正确: Windows 有桌面, 不设此变量, 直接用 QApplication([])
```

#### 2. QApplication 必须在任何 pyqtgraph 对象之前创建

```python
app = QApplication(sys.argv)  # 必须第一行 (导入之后)
```

#### 3. GraphicsLayoutWidget 用于多子图

```python
win = pg.GraphicsLayoutWidget()
p1 = win.addPlot(row=0, col=0)  # 类似 subplot(2,1,1)
p2 = win.addPlot(row=1, col=0)  # 类似 subplot(2,1,2)
p2.setXLink(p1)                  # 类似 linkaxes
```

#### 4. 垂直线/水平线 → InfiniteLine (不是 xline/yline)

```python
p.addLine(y=V30, pen=pg.mkPen('k', style=pg.QtCore.Qt.DotLine))     # 水平线
p.addLine(x=t_val, pen=pg.mkPen('r', style=pg.QtCore.Qt.DashLine))  # 垂直线
# 带标签的垂直线:
vline = pg.InfiniteLine(pos=t*1e6, angle=90, pen=pg.mkPen('r', width=1.5, style=pg.QtCore.Qt.DashDotDotLine))
p.addItem(vline)
```

#### 5. pyqtgraph 线型枚举 (pg.QtCore.Qt)

| style | 效果 |
|-------|------|
| `SolidLine` | 实线 |
| `DashLine` | 虚线 `--` |
| `DotLine` | 点线 `:` |
| `DashDotDotLine` | 点划线 `-.` |
| `DashDotLine` | 点划线 |

#### 6. 区域填充 → LinearRegionItem (不是 fill/fill_between)

```python
lr = pg.LinearRegionItem(values=(x0, x1), orientation='vertical',
                         brush=pg.mkBrush(r, g, b, alpha), movable=False)
p.addItem(lr)
```

#### 7. 散点标记 → ScatterPlotItem

```python
scatter = pg.ScatterPlotItem(x=x_arr, y=y_arr, symbol='t', size=5, pen='g', brush='g')
p.addItem(scatter)
# symbol: 'o'圆, 's'方, 't'▲, 't1'▼, 'd'菱形
```

#### 8. 文本标注 → TextItem (不是 text())

```python
txt = pg.TextItem(text='START', color=(255,255,255), anchor=(1, 0.5))
txt.setPos(x, y)
p.addItem(txt)
```

#### 9. 导出 PNG → ImageExporter

```python
exporter = pg.exporters.ImageExporter(win.scene())  # GraphicsLayoutWidget
exporter.parameters()['width'] = 1400                # 宽度(像素)
exporter.export('path/to/output.png')
```

#### 10. 颜色 → QColor 或 (R,G,B) 元组

```python
pg.mkPen(QColor(26, 115, 204), width=1.2)           # QColor 对象
pg.mkPen('r', width=1.5, style=pg.QtCore.Qt.DashLine) # 字符串颜色
pg.mkBrush(174, 217, 255, 45)                         # RGBA 填充
```

#### 11. setXRange/setYRange = xlim/ylim

```python
p.setXRange(xmin, xmax)
p.setYRange(ymin, ymax)
```

### MATLAB → Python 关键差异

| 项 | MATLAB | Python |
|----|--------|--------|
| 数组索引 | 1-based | 0-based |
| diff + 1 | `find(diff(x)>0.5)+1` | `np.where(np.diff(x)>0.5)[0]+1` |
| 切片 diff +1 | `find(diff(x(a:b))~=0)+a` | `np.where(np.diff(x[a:b+1])!=0)[0]+a+1` ← **切片必须额外+1** |
| NaN | `NaN` | `float('nan')` 或 `np.nan` |
| 三元表达式 | `iif(cond,a,b)` | `a if cond else b` |
| CSV编码 | UTF-8 BOM 手动写 | `encoding='utf-8-sig'` 自动写入BOM |
| 文件编码 | `detectImportOptions` 自动 | 需显式指定 `encoding='gbk'` 或 `latin-1` |
| struct数组 | `txn(k).bytes` | `txn[k]['bytes']` (list of dict) |

### numpy 边界检测坑

对数组切片做 `diff` 后, 位偏移量转全局索引时必须加 **+1**:
```python
# 错误: 缺少 +1, 导致检测到前一个采样点而非穿越瞬间
sda_trans_idx = np.where(np.diff(sda_d[wr_start:wr_stop+1]) != 0)[0] + wr_start

# 正确:
sda_trans_idx = np.where(np.diff(sda_d[wr_start:wr_stop+1]) != 0)[0] + wr_start + 1
```
原因: `diff(arr)[i] = arr[i+1] - arr[i]`, 变化点在第 `i+1` 个元素。
对完整数组 `np.where(np.diff(sda_d)!=0)[0]+1` 已正确, 但切片极容易漏加.
