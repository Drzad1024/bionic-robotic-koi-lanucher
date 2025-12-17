# main.py
# 适配 UI: Lanucher_V2.0.py（pyuic6 生成的 Ui_MainWindow）
# 依赖：PyQt6, pyserial(可选), LoraProtocol(必须)
#
# 说明：
# - 本文件按你给的旧版 main.py 的结构/思路重写，适配新 UI 控件命名与页面结构。
# - 新 UI 中旧代码未覆盖的功能，均保留“预调用接口”(TODO)与清晰的扩展点，不影响主流程运行。
# - 串口收包解析仍依赖 LoraProtocol.parse_frame / FishDevice / Command 等协议工具。

from __future__ import annotations

import sys
import time
import re
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox, QInputDialog

# ----------------------------- UI 导入（兼容“文件名带点”的情况） -----------------------------
Ui_MainWindow = None  # type: ignore

def _load_ui_class():
    """
    兼容导入 Ui_MainWindow：
    1) 优先按常规模块名导入（你可自行改成实际文件名，如 Lanucher_V2_0.py）
    2) 若文件名为 Lanucher_V2.0.py（包含点），则用 importlib 从路径加载
    """
    global Ui_MainWindow
    # 你可以把下面第一个名字改成你真实的 UI py 文件名（推荐不含点）
    candidates = [
        "Lanucher_UI",        # 推荐：把 Lanucher_V2.0.py 重命名为 Lanucher_V2_0.py
        "Lanucher_V2",          # 兜底
        "Lanucher_UI_V2_0",     # 兜底
        "Lanucher_UI_V2_1",     # 兜底
    ]
    for mod_name in candidates:
        try:
            mod = __import__(mod_name, fromlist=["Ui_MainWindow"])
            Ui_MainWindow = getattr(mod, "Ui_MainWindow")
            return
        except Exception:
            pass

    # 若常规 import 失败，尝试从同目录的 Lanucher_V2.0.py 加载
    try:
        import importlib.util
        from pathlib import Path
        ui_path = Path(__file__).with_name("Lanucher_V2.0.py")
        if not ui_path.exists():
            raise FileNotFoundError(f"找不到 UI 文件：{ui_path}")
        spec = importlib.util.spec_from_file_location("Lanucher_V2_dot0", str(ui_path))
        if spec is None or spec.loader is None:
            raise ImportError("spec/loader 无法创建")
        mod = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(mod)  # type: ignore
        Ui_MainWindow = getattr(mod, "Ui_MainWindow")
    except Exception as e:
        raise ImportError(
            "无法导入 Ui_MainWindow。请确认 UI 文件名与 main.py 中 _load_ui_class() 的候选名称一致，"
            "或将 UI 文件改名为不含点的模块名（如 Lanucher_V2_0.py）。"
        ) from e

_load_ui_class()

# ----------------------------- 协议与串口依赖 -----------------------------
from LoraProtocol import *  # 保留：FishDevice / parse_frame / Command 等都在此模块中

try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception as e:
    serial = None
    list_ports = None
    print("未安装 pyserial，请先 pip install pyserial：", e)


# ----------------------------- 设备状态数据结构（UI 展示用） -----------------------------
@dataclass
class DeviceState:
    fish_id: int
    paired: bool = False
    mute: bool = False
    voltage_v: Optional[float] = None
    power_mw: Optional[int] = None
    battery: Optional[str] = None
    running: Optional[str] = None


# ----------------------------- 主窗口 -----------------------------
class MainWindow(QMainWindow, Ui_MainWindow):  # type: ignore[misc]
    READ_POLL_MS = 25          # 串口读轮询周期
    CONT_SEND_MS = 50          # 连续发送周期（20Hz）
    MULTI_SEND_GAP_MS = 10     # 多设备分发间隔

    FRAME_HEADER = bytes([0xAA, 0x55])
    FRAME_TAIL = bytes([0x0D])

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)

        # ---------- 串口 ----------
        self.ser: Optional[serial.Serial] = None
        self.current_port: Optional[str] = None
        self.default_baud = 115200  # 新 UI 未明确，沿用常用高速；握手/配置可切到 9600
        self.current_baud = self.default_baud
        self._closing = False

        # ---------- 设备 ----------
        self.tx_channel = int(self.spin_CtrlCh.value())  # UI 默认 23
        self.default_password = 0x0000
        self.devices: Dict[int, FishDevice] = {}
        self.dev_state: Dict[int, DeviceState] = {}
        self.last_reply_enable = True  # 最近一次“消息应答”动作（用于回包更新）

        # ---------- RX 缓冲 ----------
        self._rx_buffer = b""

        # ---------- 定时器 ----------
        self.read_timer = QTimer(self)
        self.read_timer.timeout.connect(self._poll_serial)

        self.timer_gear = QTimer(self)
        self.timer_gear.timeout.connect(lambda: self._send_gear(silent=True))

        self.timer_servo = QTimer(self)
        self.timer_servo.timeout.connect(lambda: self._send_servo_position(silent=True))

        self.timer_cpg = QTimer(self)
        self.timer_cpg.timeout.connect(lambda: self._send_cpg(silent=True))

        # ---------- UI 初始化 ----------
        self._init_log()
        self._wire_ui()
        self._init_shortcuts()
        self._bind_value_pairs()
        self._init_tables()
        self._refresh_ports(preserve=False)

        self._update_ui_state()
        self._status("准备就绪。请选择端口并点击“启动串口(W)”。", 5000)

    # ======================================================================
    # UI：初始化与信号绑定
    # ======================================================================
    def _init_log(self):
        # 防止日志过大导致卡顿
        try:
            self.txt_Log.document().setMaximumBlockCount(1500)
        except Exception:
            pass

    def _wire_ui(self):
        # 串口区
        self.btn_OpenSerial.clicked.connect(self.toggle_serial)
        self.combo_Port.installEventFilter(self)

        # 控制器区
        self.btn_QueryCtrl.clicked.connect(self._query_ctrl_params)
        self.btn_InitCtrl.clicked.connect(self._init_controller)
        self.btn_SetCtrlCh.clicked.connect(self._set_ctrl_channel)

        # 设备列表区
        self.btn_Search.clicked.connect(self._search_devices)
        self.btn_AddDevManual.clicked.connect(self._manual_add_device_dialog)

        self.btn_SelectAll.clicked.connect(self._select_all_targets)
        self.btn_SelectNone.clicked.connect(self._select_none_targets)
        self.btn_SelectNone_2.clicked.connect(self._pair_selected_devices)     # “配对”
        self.btn_SelectNone_3.clicked.connect(self._unpair_selected_devices)  # “取消配对”(预留)

        # 系统监视区
        self.btn_GlobalStop.clicked.connect(self._global_stop_all)
        self.btn_GlobalStop_2.clicked.connect(self._global_stop_selected)

        # 简易运动：Gear
        self.btn_SendGear.clicked.connect(self._send_gear)
        self.chk_Sync_Gear.toggled.connect(self._toggle_gear_cont)

        # 舵机综合管理
        self.btn_SendServo.clicked.connect(self._send_servo_position)
        self.chk_Sync_Servo.toggled.connect(self._toggle_servo_cont)
        self.btn_SetServoPower.clicked.connect(self._set_servo_power)
        self.btn_QueryServoAll.clicked.connect(self._query_servo_all_status)  # 预留：查询全部状态

        # 高阶运动：CPG
        self.btn_SendCPG.clicked.connect(self._send_cpg)
        self.chk_Sync_CPG.toggled.connect(self._toggle_cpg_cont)

        # 表演模式
        self.btn_PlayStart.clicked.connect(self._start_play_mode)

        # 开机参数设置
        self.btn_FlashRead.clicked.connect(self._flash_read_config)   # 预留
        self.btn_FlashSave.clicked.connect(self._flash_save_config)   # 预留

        # 高级设置：自动回传/维护命令（预留）
        self.btn_SetAutoReport.clicked.connect(self._set_auto_report)  # 预留
        self.btn_SetInstallBias.clicked.connect(self._set_install_bias)  # 预留
        self.btn_ResetFault_6.clicked.connect(self._reply_switch_dialog)  # 关闭/开启消息回复
        self.btn_ResetFault.clicked.connect(self._query_volt_power)
        self.btn_ResetFault_4.clicked.connect(self._query_servo_status_single)  # 预留
        self.btn_ResetFault_3.clicked.connect(self._query_mos_temp)  # 预留
        self.btn_ResetFault_5.clicked.connect(self._query_flash)  # 预留
        self.btn_ResetFault_2.clicked.connect(self._reset_faulty_servo)  # 预留
        self.btn_FactoryReset.clicked.connect(self._factory_reset)  # 预留

        # 系统环境检查
        self.btn_CheckCompute.clicked.connect(self._check_compute_env)

        # 日志控制
        self.btn_ClearLog.clicked.connect(self.txt_Log.clear)
        self.chk_AutoScroll.toggled.connect(lambda _: None)

    def _init_shortcuts(self):
        QShortcut(QKeySequence("W"), self, activated=self.toggle_serial)
        QShortcut(QKeySequence("R"), self, activated=self._search_devices)
        QShortcut(QKeySequence("S"), self, activated=self._global_stop_selected)
        QShortcut(QKeySequence("Ctrl+N"), self, activated=self._manual_add_device_dialog)

    def _bind_value_pairs(self):
        # Gear：Dial <-> Spin
        self.dial_Speed.valueChanged.connect(self.spin_GearSpeed.setValue)
        self.spin_GearSpeed.valueChanged.connect(self.dial_Speed.setValue)

        self.dial_Turn.valueChanged.connect(self.spin_GearTurn.setValue)
        self.spin_GearTurn.valueChanged.connect(self.dial_Turn.setValue)

        # Servo：Slider(-900..900) <-> Spin(-90..90)
        self.slider_S1.valueChanged.connect(lambda v: self._servo_slider_to_spin(v, self.spin_S1))
        self.spin_S1.valueChanged.connect(lambda v: self._servo_spin_to_slider(v, self.slider_S1))

        self.slider_S2.valueChanged.connect(lambda v: self._servo_slider_to_spin(v, self.spin_S2))
        self.spin_S2.valueChanged.connect(lambda v: self._servo_spin_to_slider(v, self.slider_S2))

        # CPG：Slider 与 Spin 已在 UI 里设定范围（0..250 对应 0.0..25.0 等）
        self.slider_Amp.valueChanged.connect(lambda v: self.spin_Amp.setValue(v / 10.0))
        self.spin_Amp.valueChanged.connect(lambda v: self.slider_Amp.setValue(int(round(v * 10))))

        self.slider_Freq.valueChanged.connect(lambda v: self.spin_Freq.setValue(v / 10.0))
        self.spin_Freq.valueChanged.connect(lambda v: self.slider_Freq.setValue(int(round(v * 10))))

        self.slider_Bias.valueChanged.connect(lambda v: self.spin_Bias.setValue(v / 10.0))
        self.spin_Bias.valueChanged.connect(lambda v: self.slider_Bias.setValue(int(round(v * 10))))

        # 间歇参数：仅提供接口，不做额外绑定
        # N: spin_N ; coast: spin_Coast

    def _init_tables(self):
        # 设备表
        self.table_Devices.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_Devices.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)

        # 系统监视表：支持多选（并且第一列“选择”可用复选框）
        self.table_SysMonitor.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_SysMonitor.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)

        # selection 变化时更新可用性
        sm = self.table_SysMonitor.selectionModel()
        if sm:
            sm.selectionChanged.connect(lambda *_: self._update_ui_state())

    # ======================================================================
    # 通用小工具
    # ======================================================================
    def _status(self, msg: str, ms: int = 3000):
        try:
            self.statusBar().showMessage(msg, ms)
        except Exception:
            pass

    def _log(self, text: str):
        try:
            self.txt_Log.appendPlainText(text)
            if self.chk_AutoScroll.isChecked():
                cursor = self.txt_Log.textCursor()
                cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
                self.txt_Log.setTextCursor(cursor)
        except Exception:
            pass

    def _todo(self, what: str):
        self._log(f"[TODO] {what}")
        self._status(f"未实现：{what}", 3000)

    def _ensure_serial(self) -> bool:
        if serial is None:
            QMessageBox.warning(self, "缺少依赖", "未安装 pyserial，请先执行：pip install pyserial")
            return False
        if self.ser is None or not self.ser.is_open:
            self._status("请先启动串口(W)。", 4000)
            return False
        return True

    @staticmethod
    def _process_events_ms(ms: int):
        t0 = time.perf_counter()
        app = QtWidgets.QApplication.instance()
        while (time.perf_counter() - t0) < (ms / 1000.0):
            if app:
                app.processEvents(QtCore.QEventLoop.ProcessEventsFlag.AllEvents, 5)
            time.sleep(0.001)

    def _sleep_ms(self, ms: int):
        if self._closing or ms <= 0:
            return
        self._process_events_ms(ms)

    # ======================================================================
    # 端口刷新与事件过滤
    # ======================================================================
    def _refresh_ports(self, preserve: bool = True):
        if list_ports is None:
            # 没装 pyserial 的时候，保底占位
            self.combo_Port.clear()
            self.combo_Port.addItem("点击刷新")
            return

        current = self.combo_Port.currentText() if preserve else None
        ports = [p.device for p in list_ports.comports()]

        self.combo_Port.blockSignals(True)
        self.combo_Port.clear()
        if ports:
            self.combo_Port.addItems(ports)
            if preserve and current in ports:
                self.combo_Port.setCurrentText(current)
            else:
                self.combo_Port.setCurrentIndex(0)
        else:
            self.combo_Port.addItem("无可用端口")
        self.combo_Port.blockSignals(False)

    def eventFilter(self, obj, event):
        # 点击/按键触发端口刷新
        try:
            if obj is self.combo_Port and event.type() in (
                QtCore.QEvent.Type.MouseButtonPress,
                QtCore.QEvent.Type.KeyPress,
            ):
                if self.combo_Port.isEnabled():
                    self._refresh_ports(preserve=True)
                    self._status("已刷新串口列表。", 1500)
        except Exception:
            pass
        return super().eventFilter(obj, event)

    # ======================================================================
    # UI 状态管理（集中启用/禁用）
    # ======================================================================
    def _update_ui_state(self):
        serial_open = bool(self.ser and self.ser.is_open)
        has_any_device = (len(self.devices) > 0)

        # 串口配置区
        self.combo_Port.setEnabled(not serial_open)
        self.btn_OpenSerial.setEnabled(True)

        # 控制器参数区：串口打开即可用
        for w in [self.btn_InitCtrl, self.btn_QueryCtrl, self.btn_SetCtrlCh, self.spin_CtrlCh, self.comboBox]:
            w.setEnabled(serial_open)

        # 设备管理：串口打开即可搜索/手动添加；配对/取消配对需要选中目标
        self.btn_Search.setEnabled(serial_open)
        self.btn_AddDevManual.setEnabled(serial_open)
        self.btn_SelectAll.setEnabled(serial_open and has_any_device)
        self.btn_SelectNone.setEnabled(serial_open and has_any_device)
        self.btn_SelectNone_2.setEnabled(serial_open and self._selected_target_ids() != [])
        self.btn_SelectNone_3.setEnabled(serial_open and self._selected_target_ids() != [])

        # 监视与急停
        self.btn_GlobalStop.setEnabled(serial_open and has_any_device)
        self.btn_GlobalStop_2.setEnabled(serial_open and self._selected_target_ids() != [])

        # 运动控制与舵机控制：串口打开且有选中目标才允许发送
        targets_ok = serial_open and (self._selected_target_ids() != [])
        self.btn_SendGear.setEnabled(targets_ok)
        self.chk_Sync_Gear.setEnabled(targets_ok)

        self.btn_SendServo.setEnabled(targets_ok)
        self.chk_Sync_Servo.setEnabled(targets_ok)
        self.btn_SetServoPower.setEnabled(targets_ok)
        self.btn_QueryServoAll.setEnabled(targets_ok)

        self.btn_SendCPG.setEnabled(targets_ok)
        self.chk_Sync_CPG.setEnabled(targets_ok)

        self.btn_PlayStart.setEnabled(targets_ok)

        # Boot/高级设置/环境检查：串口开即可点（具体功能内部可再限制）
        for w in [self.btn_FlashRead, self.btn_FlashSave, self.btn_SetAutoReport,
                  self.btn_SetInstallBias, self.btn_ResetFault_6, self.btn_ResetFault,
                  self.btn_ResetFault_4, self.btn_ResetFault_3, self.btn_ResetFault_5,
                  self.btn_ResetFault_2, self.btn_FactoryReset, self.btn_CheckCompute]:
            w.setEnabled(serial_open)

        # 若串口关闭，停止所有连续发送
        if not serial_open:
            for chk, timer in [
                (self.chk_Sync_Gear, self.timer_gear),
                (self.chk_Sync_Servo, self.timer_servo),
                (self.chk_Sync_CPG, self.timer_cpg),
            ]:
                try:
                    chk.blockSignals(True)
                    chk.setChecked(False)
                finally:
                    chk.blockSignals(False)
                timer.stop()

    # ======================================================================
    # 串口开关
    # ======================================================================
    def toggle_serial(self):
        if serial is None:
            QMessageBox.warning(self, "缺少依赖", "未安装 pyserial，请先执行：pip install pyserial")
            return

        if self.ser and self.ser.is_open:
            # 关闭
            try:
                self.timer_gear.stop()
                self.timer_servo.stop()
                self.timer_cpg.stop()
                self.read_timer.stop()
                self.ser.close()
                self.ser = None
                self.current_port = None
                self._rx_buffer = b""
                self.btn_OpenSerial.setText("启动串口(&W)")
                self._status("串口已关闭。", 3000)

                # 清空设备
                self._clear_all_devices()
                self._update_ui_state()
            except Exception as e:
                QMessageBox.warning(self, "关闭失败", str(e))
            return

        # 打开
        port = (self.combo_Port.currentText() or "").strip()
        if (not port) or ("无可用端口" in port) or ("点击刷新" in port):
            QMessageBox.warning(self, "无效端口", "请选择一个有效串口。")
            return

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=self.default_baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                rtscts=False, dsrdtr=False, xonxoff=False,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

            self.current_port = port
            self.current_baud = self.default_baud
            self._rx_buffer = b""

            self.read_timer.start(self.READ_POLL_MS)
            self.btn_OpenSerial.setText("关闭串口(&W)")
            self._status(f"串口已打开：{port} @ {self.current_baud}bps", 4000)
            self._log(f"[SER] Open {port} @ {self.current_baud}")

            self._update_ui_state()
        except Exception as e:
            self.ser = None
            QMessageBox.critical(self, "打开失败", f"{port} 打开失败：\n{e}")

    # ======================================================================
    # 串口收包与解析
    # ======================================================================
    def _poll_serial(self):
        if self._closing or (self.ser is None) or (not self.ser.is_open):
            return
        try:
            n = self.ser.in_waiting
            if not n:
                return

            raw = self.ser.read(n)
            if not raw:
                return

            self._log("RX(%d): %s" % (len(raw), " ".join(f"{b:02X}" for b in raw)))
            self._rx_buffer += raw

            # 从缓冲区中切包：AA 55 .... 0D
            while True:
                h = self._rx_buffer.find(self.FRAME_HEADER)
                if h < 0:
                    # 没有包头：丢弃过长垃圾
                    if len(self._rx_buffer) > 4096:
                        self._rx_buffer = b""
                    break
                t = self._rx_buffer.find(self.FRAME_TAIL, h + len(self.FRAME_HEADER))
                if t < 0:
                    # 有头无尾：等待后续
                    # 但若头前有垃圾，丢掉
                    if h > 0:
                        self._rx_buffer = self._rx_buffer[h:]
                    break

                pkt = self._rx_buffer[h:t + 1]
                self._rx_buffer = self._rx_buffer[t + 1:]

                self._handle_frame(pkt)

        except Exception as e:
            self._status(f"串口读取错误：{e}", 5000)

    def _handle_frame(self, pkt: bytes):
        # 解析帧（协议细节交给 LoraProtocol）
        try:
            parsed = parse_frame(pkt)
        except Exception as e:
            self._log(f"[PARSE_ERR] {e} | raw={pkt.hex(' ')}")
            return

        # 统一校验
        if not (parsed.get("head_ok") and parsed.get("tail_ok") and parsed.get("checksum_ok")):
            self._log("[FRAME] 无效帧：头尾或校验失败。")
            return

        cmd = parsed.get("cmd")
        fish_id = int(parsed.get("fish_id", 0))
        data = parsed.get("data", [])

        # 下面按你旧代码的典型分发方式写；具体 cmd 数值以 LoraProtocol.Command 为准
        try:
            if cmd == Command.CMD_REPLY:
                self._handle_cmd_reply(fish_id, data, parsed)
            elif cmd == Command.VOLT_PWR_REPLY:
                self._handle_volt_power_reply(fish_id, data, parsed)
            elif cmd == getattr(Command, "STATE_REPLY_1", None):
                self._handle_state_reply_1(fish_id, data, parsed)  # 预留
            elif cmd == getattr(Command, "STATE_REPLY_2", None):
                self._handle_state_reply_2(fish_id, data, parsed)  # 预留
            elif cmd == getattr(Command, "STATE_REPLY_3", None):
                self._handle_state_reply_3(fish_id, data, parsed)  # 预留
            else:
                self._log(f"[FRAME] 未处理的 CMD=0x{int(cmd):02X} 来自 0x{fish_id:04X}")
        except Exception as e:
            self._log(f"[DISPATCH_ERR] {e}")

    def _handle_cmd_reply(self, fish_id: int, data: list, parsed: dict):
        # 旧逻辑：data[0] 是“被回复的命令”，data[2] 是结果码
        if not data or len(data) < 3:
            self._log("[REPLY] 格式不完整。")
            return

        replied_cmd = data[0]
        result = data[2]

        if replied_cmd == getattr(Command, "SEARCH_DEVICES", None):
            if result == 0:
                self._ensure_device_exists(fish_id)
                self._log(f"[DEV] 搜索到设备：0x{fish_id:04X}")
            else:
                self._log("[DEV] 搜索设备失败，请重试。")
            return

        if replied_cmd == getattr(Command, "REPLY_CTRL", None):
            if result == 0:
                st = self.dev_state.get(fish_id)
                if st:
                    st.mute = (not self.last_reply_enable)
                    self._update_device_tables_row(fish_id)
                self._log(f"[DEV] 0x{fish_id:04X} 消息回复已设置为：{'开' if self.last_reply_enable else '关'}")
            else:
                self._log(f"[DEV] 0x{fish_id:04X} 消息回复设置失败：code={result}")
            return

        self._log(f"[REPLY] fish=0x{fish_id:04X} replied_cmd=0x{replied_cmd:02X} result={result}")

    def _handle_volt_power_reply(self, fish_id: int, data: list, parsed: dict):
        # 旧逻辑示例：voltage = data[0]/10 ; power = (data[1]<<8)|data[2]
        self._ensure_device_exists(fish_id)
        st = self.dev_state[fish_id]

        try:
            if len(data) >= 3:
                st.voltage_v = data[0] / 10.0
                st.power_mw = (data[1] << 8) | data[2]
                # 配对成功（沿用旧逻辑：收到电压功率回复可视为链路正常）
                if not st.paired:
                    st.paired = True
                self._update_device_tables_row(fish_id)
                self._update_monitor_tables_row(fish_id)
                self._log(f"[PWR] 0x{fish_id:04X} V={st.voltage_v:.2f}V P={st.power_mw}mW")
        except Exception as e:
            self._log(f"[PWR_ERR] {e}")

    # 以下三个状态回包处理：新 UI 可按你的协议补齐
    def _handle_state_reply_1(self, fish_id: int, data: list, parsed: dict):
        self._todo("STATE_REPLY_1 解析并更新 UI（例如 CPG 上一次参数）")

    def _handle_state_reply_2(self, fish_id: int, data: list, parsed: dict):
        self._todo("STATE_REPLY_2 解析并更新 UI（例如 舵机位置/电流/温度）")

    def _handle_state_reply_3(self, fish_id: int, data: list, parsed: dict):
        self._todo("STATE_REPLY_3 解析并更新 UI（例如 表演模式上一次参数）")

    # ======================================================================
    # 发送字节
    # ======================================================================
    def _send_bytes(self, data: bytes, tag: str = "") -> bool:
        if not self._ensure_serial():
            return False
        try:
            assert self.ser is not None
            self.ser.write(data)
            self.ser.flush()
            hx = " ".join(f"{b:02X}" for b in data)
            self._log(f"TX{f'[{tag}]' if tag else ''}: {hx}")
            return True
        except Exception as e:
            self._status(f"发送失败：{e}", 5000)
            return False

    # ======================================================================
    # 设备与目标选择
    # ======================================================================
    def _ensure_device_exists(self, fish_id: int):
        if fish_id not in self.devices:
            self.devices[fish_id] = FishDevice(fish_id, self.default_password, self.tx_channel)
            self.dev_state[fish_id] = DeviceState(fish_id=fish_id)
            self._append_device_to_table(fish_id)
            self._append_monitor_row(fish_id)

    def _clear_all_devices(self):
        self.devices.clear()
        self.dev_state.clear()
        try:
            self.table_Devices.setRowCount(0)
        except Exception:
            pass
        try:
            self.table_SysMonitor.setRowCount(0)
        except Exception:
            pass

    def _append_device_to_table(self, fish_id: int):
        row = self.table_Devices.rowCount()
        self.table_Devices.insertRow(row)
        it_id = QtWidgets.QTableWidgetItem(f"0x{fish_id:04X}")
        it_id.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_Devices.setItem(row, 0, it_id)

        it_pair = QtWidgets.QTableWidgetItem("未配对")
        it_pair.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        it_pair.setForeground(QtGui.QBrush(QtGui.QColor("red")))
        self.table_Devices.setItem(row, 1, it_pair)

    def _append_monitor_row(self, fish_id: int):
        row = self.table_SysMonitor.rowCount()
        self.table_SysMonitor.insertRow(row)

        # col0: 选择（复选框）
        it_sel = QtWidgets.QTableWidgetItem("")
        it_sel.setFlags(it_sel.flags() | Qt.ItemFlag.ItemIsUserCheckable)
        it_sel.setCheckState(Qt.CheckState.Unchecked)
        it_sel.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_SysMonitor.setItem(row, 0, it_sel)

        # col1: FishID
        it_id = QtWidgets.QTableWidgetItem(f"0x{fish_id:04X}")
        it_id.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_SysMonitor.setItem(row, 1, it_id)

        # col2..6 初始化占位
        for c, txt in [(2, "-"), (3, "-"), (4, "-"), (5, "-"), (6, "-")]:
            it = QtWidgets.QTableWidgetItem(txt)
            it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table_SysMonitor.setItem(row, c, it)

    def _find_row_by_fish_id(self, table: QtWidgets.QTableWidget, col: int, fish_id: int) -> Optional[int]:
        target = f"0x{fish_id:04X}".upper()
        for r in range(table.rowCount()):
            it = table.item(r, col)
            if it and (it.text() or "").strip().upper() == target:
                return r
        return None

    def _update_device_tables_row(self, fish_id: int):
        st = self.dev_state.get(fish_id)
        if not st:
            return
        r = self._find_row_by_fish_id(self.table_Devices, 0, fish_id)
        if r is None:
            return
        it_pair = self.table_Devices.item(r, 1)
        if it_pair is None:
            it_pair = QtWidgets.QTableWidgetItem("")
            self.table_Devices.setItem(r, 1, it_pair)

        if st.paired:
            it_pair.setText("已配对")
            it_pair.setForeground(QtGui.QBrush(QtGui.QColor(0, 170, 0)))
        else:
            it_pair.setText("未配对")
            it_pair.setForeground(QtGui.QBrush(QtGui.QColor("red")))
        it_pair.setTextAlignment(Qt.AlignmentFlag.AlignCenter)

    def _update_monitor_tables_row(self, fish_id: int):
        st = self.dev_state.get(fish_id)
        if not st:
            return
        r = self._find_row_by_fish_id(self.table_SysMonitor, 1, fish_id)
        if r is None:
            return

        def _set(col: int, txt: str):
            it = self.table_SysMonitor.item(r, col)
            if it is None:
                it = QtWidgets.QTableWidgetItem("")
                it.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table_SysMonitor.setItem(r, col, it)
            it.setText(txt)

        _set(2, "-" if st.voltage_v is None else f"{st.voltage_v:.2f}")
        _set(3, "-" if st.power_mw is None else f"{st.power_mw}")
        _set(4, "-" if st.battery is None else st.battery)
        _set(5, "否" if st.mute else "是")  # 这里“消息回复”列：按你旧代码语义自行微调
        _set(6, "-" if st.running is None else st.running)

    def _selected_target_ids(self) -> List[int]:
        """
        新 UI 提示“点击选中后才可发送指令，支持多选”位于系统监视表，
        因此目标选择以 table_SysMonitor 的“复选框”为准（更直观且可长期保留选择）。
        """
        ids: List[int] = []
        for r in range(self.table_SysMonitor.rowCount()):
            it_sel = self.table_SysMonitor.item(r, 0)
            it_id = self.table_SysMonitor.item(r, 1)
            if not it_sel or not it_id:
                continue
            if it_sel.checkState() == Qt.CheckState.Checked:
                txt = (it_id.text() or "").strip()
                try:
                    fid = int(txt, 16)
                    ids.append(fid)
                except Exception:
                    pass
        return ids

    def _target_fishes(self) -> List[FishDevice]:
        ids = self._selected_target_ids()
        fishes: List[FishDevice] = []
        for fid in ids:
            dev = self.devices.get(fid)
            if dev is None:
                dev = FishDevice(fid, self.default_password, self.tx_channel)
                self.devices[fid] = dev
                self.dev_state[fid] = DeviceState(fish_id=fid)
            dev.channel = self.tx_channel
            fishes.append(dev)
        return fishes

    def _select_all_targets(self):
        for r in range(self.table_SysMonitor.rowCount()):
            it = self.table_SysMonitor.item(r, 0)
            if it:
                it.setCheckState(Qt.CheckState.Checked)
        self._update_ui_state()

    def _select_none_targets(self):
        for r in range(self.table_SysMonitor.rowCount()):
            it = self.table_SysMonitor.item(r, 0)
            if it:
                it.setCheckState(Qt.CheckState.Unchecked)
        self._update_ui_state()

    # ======================================================================
    # 控制器参数区（你可按实际协议补齐）
    # ======================================================================
    def _init_controller(self):
        if not self._ensure_serial():
            return
        # 预留：初始化控制器/发射器（例如旧版 INIT_TRANSMITTER）
        self._todo("初始化控制器：发送对应 LoRa 参数配置帧（如需 9600 握手/RTS 切换）")

    def _query_ctrl_params(self):
        if not self._ensure_serial():
            return
        # 预留：查询控制器参数（版本、ID 等），并更新 lbl_CtrlLinkState / lbl_CtrlID_Val 等
        self._todo("查询控制器参数：握手、解析并更新 UI（连接状态/版本/ID/信道）")

    def _set_ctrl_channel(self):
        if not self._ensure_serial():
            return
        ch = int(self.spin_CtrlCh.value())
        if not (0 <= ch <= 83):
            self._status("信道号无效，应在 0-83 之间。", 4000)
            return
        self.tx_channel = ch
        # 预留：此处应向控制器/发射器下发“设置信道”的配置命令
        self._todo(f"设置信道到 {ch}：发送控制器配置帧，并在成功后清空设备列表/重新搜索")

    # ======================================================================
    # 设备管理
    # ======================================================================
    def _search_devices(self):
        if not self._ensure_serial():
            return
        # 广播搜索：FishDevice(0xFFFF).search_devices()
        try:
            broadcast = FishDevice(0xFFFF, self.default_password, self.tx_channel)
            frame = broadcast.search_devices()
            if self._send_bytes(frame, tag="SearchDevices"):
                self._status("已广播搜索设备。", 2000)
        except Exception as e:
            self._status(f"搜索设备失败：{e}", 5000)

    def _manual_add_device_dialog(self):
        if not self._ensure_serial():
            return
        while True:
            txt, ok = QInputDialog.getText(self, "手动添加设备", "输入 FishID（1~4位十六进制，如 1A2B 或 0x1A2B）：")
            if not ok:
                return
            s = (txt or "").strip().upper()
            if s.startswith("0X"):
                s = s[2:]
            if re.fullmatch(r"[0-9A-F]{1,4}", s):
                fid = int(s, 16)
                break
            QMessageBox.warning(self, "无效输入", "必须是 1~4 位十六进制（0-9/A-F），可带 0x。")

        self._ensure_device_exists(fid)
        self._status(f"已添加设备：0x{fid:04X}", 2000)
        self._update_ui_state()

    def _pair_selected_devices(self):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            self._status("请先在“系统监视”勾选目标设备。", 3000)
            return

        # 新 UI 未提供明文密码框；此处沿用 default_password 或从 BootConfig 的 spin_F_Pwd 取值
        pwd = int(self.spin_F_Pwd.value()) & 0xFFFF

        try:
            for dev in targets:
                # 旧版：dev.set_password(pwd, send=False) + query_volt_power 触发配对
                # 这里按你的 LoraProtocol 实现决定：如果 set_password 会直接发包，则可直接调用。
                if hasattr(dev, "set_password"):
                    dev.set_password(pwd, send=False)  # type: ignore[arg-type]
            self._query_volt_power()
            self._status(f"已向 {len(targets)} 台设备发送配对流程（随后查询电压/功率确认）。", 3000)
        except Exception as e:
            self._status(f"配对失败：{e}", 5000)

    def _unpair_selected_devices(self):
        # 预留：你的协议若支持“取消配对/清密钥”等，可在此实现
        self._todo("取消配对：按协议下发取消配对/恢复默认密码等命令，并更新 UI")

    # ======================================================================
    # 系统监视/急停
    # ======================================================================
    def _global_stop_selected(self):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            self._status("未勾选目标设备，无法急停。", 3000)
            return
        # 预留：按协议实现“急停”指令（可能是 reset / stop motion）
        self._todo(f"急停(选中)：向 {len(targets)} 台设备发送急停/复位指令")

    def _global_stop_all(self):
        if not self._ensure_serial():
            return
        # 预留：广播急停；若协议不允许广播，可遍历已知设备逐个发
        self._todo("急停(全部)：广播或逐设备下发急停/复位指令")

    # ======================================================================
    # Gear：简易运动控制（速度/转向档）
    # ======================================================================
    def _toggle_gear_cont(self, on: bool):
        if on:
            self.timer_gear.start(self.CONT_SEND_MS)
            self._status("速度/转向：实时同步已开启。", 2000)
        else:
            self.timer_gear.stop()
            self._status("速度/转向：实时同步已关闭。", 2000)

    def _send_gear(self, silent: bool = False):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            if not silent:
                self._status("未勾选目标设备。", 2000)
            return

        speed = int(self.spin_GearSpeed.value())  # 0..15
        turn = int(self.spin_GearTurn.value())    # 1..15

        # 预留：这里应调用协议里的“档位模式/速度转向”命令
        try:
            for dev in targets:
                if hasattr(dev, "gear_ctrl"):
                    frame = dev.gear_ctrl(speed, turn)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"GEAR 0x{dev.fish_id:04X} spd={speed} turn={turn}")
                else:
                    # 不存在该接口就先记录，避免程序崩
                    self._todo("LoraProtocol.FishDevice.gear_ctrl(speed, turn) 未实现/未命名")
                    break
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            if not silent:
                self._status(f"已发送档位指令：{len(targets)} 台设备。", 2000)
        except Exception as e:
            self._status(f"发送档位失败：{e}", 5000)

    # ======================================================================
    # Servo：舵机综合管理
    # ======================================================================
    def _servo_slider_to_spin(self, slider_val: int, spin: QtWidgets.QDoubleSpinBox):
        # slider: -900..900 -> spin: -90..90
        spin.blockSignals(True)
        try:
            spin.setValue(slider_val / 10.0)
        finally:
            spin.blockSignals(False)

    def _servo_spin_to_slider(self, spin_val: float, slider: QtWidgets.QSlider):
        slider.blockSignals(True)
        try:
            slider.setValue(int(round(spin_val * 10)))
        finally:
            slider.blockSignals(False)

    def _toggle_servo_cont(self, on: bool):
        if on:
            self.timer_servo.start(self.CONT_SEND_MS)
            self._status("舵机位置：实时同步已开启。", 2000)
        else:
            self.timer_servo.stop()
            self._status("舵机位置：实时同步已关闭。", 2000)

    def _send_servo_position(self, silent: bool = False):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            if not silent:
                self._status("未勾选目标设备。", 2000)
            return

        # UI：radio_S1_Only / radio_S2_Only / radio_Dual_Sync
        mode_s1 = self.radio_S1_Only.isChecked()
        mode_s2 = self.radio_S2_Only.isChecked()
        mode_dual = self.radio_Dual_Sync.isChecked()

        ang1 = float(self.spin_S1.value())  # -90..90
        ang2 = float(self.spin_S2.value())

        try:
            for dev in targets:
                if mode_s1 and hasattr(dev, "servo_ctrl_sng"):
                    frame = dev.servo_ctrl_sng(1, ang1)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"SERVO1 0x{dev.fish_id:04X} {ang1:.1f}")
                elif mode_s2 and hasattr(dev, "servo_ctrl_sng"):
                    frame = dev.servo_ctrl_sng(2, ang2)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"SERVO2 0x{dev.fish_id:04X} {ang2:.1f}")
                elif mode_dual and hasattr(dev, "servo_ctrl_dbl"):
                    # 方向位/符号处理按你协议：此处仅提供典型接口
                    direction = 0x11 if ang1 >= 0 else 0x00
                    frame = dev.servo_ctrl_dbl(direction, ang1, ang2)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"SERVO12 0x{dev.fish_id:04X} {ang1:.1f},{ang2:.1f}")
                else:
                    self._todo("LoraProtocol.FishDevice.servo_ctrl_sng/servo_ctrl_dbl 未实现/未命名，或模式未覆盖")
                    break

                self._sleep_ms(self.MULTI_SEND_GAP_MS)

            if not silent:
                self._status(f"已发送舵机位置：{len(targets)} 台设备。", 2000)
        except Exception as e:
            self._status(f"发送舵机位置失败：{e}", 5000)

    def _set_servo_power(self):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            self._status("未勾选目标设备。", 2000)
            return

        s1_on = self.chk_S1_Pwr.isChecked()
        s2_on = self.chk_S2_Pwr.isChecked()
        # 预留：舵机供电控制命令
        self._todo(f"舵机供电：S1={'ON' if s1_on else 'OFF'} S2={'ON' if s2_on else 'OFF'} (对 {len(targets)} 台设备)")

    def _query_servo_all_status(self):
        # 预留：查询舵机电流/温度等，并更新 bar_S1_Curr/bar_S2_Curr/bar_S1_Temp/bar_S2_Temp
        self._todo("查询全部舵机状态：电流/温度/运行状态，并更新进度条")

    # ======================================================================
    # CPG：高阶运动控制
    # ======================================================================
    def _toggle_cpg_cont(self, on: bool):
        if on:
            self.timer_cpg.start(self.CONT_SEND_MS)
            self._status("CPG：实时同步已开启。", 2000)
        else:
            self.timer_cpg.stop()
            self._status("CPG：实时同步已关闭。", 2000)

    def _send_cpg(self, silent: bool = False):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            if not silent:
                self._status("未勾选目标设备。", 2000)
            return

        amp = float(self.spin_Amp.value())   # °，0..25
        freq = float(self.spin_Freq.value()) # Hz，0..3.8
        bias = float(self.spin_Bias.value()) # °，-25..25

        # 间歇参数：grp_CPG_Inter 可选启用
        inter_enabled = self.grp_CPG_Inter.isChecked()
        N = int(self.spin_N.value())
        coast = float(self.spin_Coast.value())

        try:
            for dev in targets:
                if hasattr(dev, "cpg_ctrl"):
                    frame = dev.cpg_ctrl(amp, bias, freq)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"CPG 0x{dev.fish_id:04X} amp={amp:.1f} bias={bias:.1f} freq={freq:.1f}")
                else:
                    self._todo("LoraProtocol.FishDevice.cpg_ctrl(amp, bias, freq) 未实现/未命名")
                    break

                if inter_enabled:
                    # 预留：间歇参数下发（如果协议支持独立命令或复合命令）
                    self._todo(f"间歇参数：N={N}, coast={coast}（协议命令待补）")

                self._sleep_ms(self.MULTI_SEND_GAP_MS)

            if not silent:
                self._status(f"已发送 CPG 参数：{len(targets)} 台设备。", 2000)
        except Exception as e:
            self._status(f"发送 CPG 失败：{e}", 5000)

    # ======================================================================
    # 表演模式
    # ======================================================================
    def _start_play_mode(self):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            self._status("未勾选目标设备。", 2000)
            return

        play_id = int(self.spin_PlayId.value()) & 0xFF
        try:
            for dev in targets:
                if hasattr(dev, "perf_ctrl"):
                    frame = dev.perf_ctrl(play_id)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"PLAY 0x{dev.fish_id:04X} id={play_id}")
                else:
                    self._todo("LoraProtocol.FishDevice.perf_ctrl(play_id) 未实现/未命名")
                    break
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            self._status(f"已启动表演：{len(targets)} 台设备，序号={play_id}", 3000)
        except Exception as e:
            self._status(f"启动表演失败：{e}", 5000)

    # ======================================================================
    # BootConfig / 高级设置：预留接口
    # ======================================================================
    def _flash_read_config(self):
        self._todo("读取当前设备配置（FlashRead）：需要限定单设备并解析回包填充 BootConfig UI")

    def _flash_save_config(self):
        self._todo("保存配置到设备（FlashSave）：将 BootConfig UI 打包下发并确认回包")

    def _set_auto_report(self):
        self._todo("状态自动回传配置：读取 chk_Rpt_* + spin_Rpt_Ms + radio_Rpt_On/Off 并下发")

    def _set_install_bias(self):
        self._todo("设置安装偏置：通常需要弹窗/或读取当前参数后下发")

    def _query_mos_temp(self):
        self._todo("查询 MOS 温度：下发命令并在回包中更新 UI")

    def _query_flash(self):
        self._todo("查询 Flash：下发命令并显示回包摘要")

    def _reset_faulty_servo(self):
        self._todo("复位损坏舵机：下发维护命令")

    def _factory_reset(self):
        # 高风险操作：建议二次确认
        ok = QMessageBox.question(self, "恢复出厂设置", "该操作可能不可逆，确定继续吗？") == QMessageBox.StandardButton.Yes
        if not ok:
            return
        self._todo("恢复出厂设置：下发 factory reset 命令并处理回包")

    # ======================================================================
    # 高级设置：应答控制 + 电压功率查询（沿用旧思路）
    # ======================================================================
    def _reply_switch_dialog(self):
        if not self._ensure_serial():
            return
        choice, ok = QInputDialog.getItem(self, "消息回复", "请选择：", ["开启应答", "关闭应答"], 0, False)
        if not ok:
            return
        enable = (choice == "开启应答")
        targets = self._target_fishes()
        if not targets:
            self._status("未勾选目标设备。", 2000)
            return

        self.last_reply_enable = enable
        try:
            for dev in targets:
                if hasattr(dev, "replySwitch"):
                    frame = dev.replySwitch(enable)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"Reply={'ON' if enable else 'OFF'} 0x{dev.fish_id:04X}")
                else:
                    self._todo("LoraProtocol.FishDevice.replySwitch(enable) 未实现/未命名")
                    break
                self._sleep_ms(self.MULTI_SEND_GAP_MS)

            self._status(f"已设置消息应答：{'开启' if enable else '关闭'}（{len(targets)} 台）", 3000)
        except Exception as e:
            self._status(f"设置消息应答失败：{e}", 5000)

    def _query_volt_power(self):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            self._status("未勾选目标设备。", 2000)
            return

        try:
            for dev in targets:
                if hasattr(dev, "query_voltage"):
                    frame = dev.query_voltage(auto=False)  # type: ignore[attr-defined]
                    self._send_bytes(frame, tag=f"QueryV/P 0x{dev.fish_id:04X}")
                else:
                    self._todo("LoraProtocol.FishDevice.query_voltage(auto=False) 未实现/未命名")
                    break
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            self._status(f"已发送电压/功率查询：{len(targets)} 台设备。", 2500)
        except Exception as e:
            self._status(f"查询电压/功率失败：{e}", 5000)

    def _query_servo_status_single(self):
        self._todo("查询舵机状态：建议限定单设备，下发命令并更新舵机监视区")

    # ======================================================================
    # 系统环境检查（本地信息，不依赖协议；你也可改成对设备查询）
    # ======================================================================
    def _check_compute_env(self):
        # 这里不做重依赖，给一个“可运行的最小实现”
        import platform
        try:
            self.lbl_OS_Val.setText(f"{platform.system()} {platform.release()}")
            self.lbl_CPU_Val.setText(platform.processor() or "未知")
        except Exception:
            self.lbl_OS_Val.setText("未知")
            self.lbl_CPU_Val.setText("未知")

        # GPU/CUDA/视觉支持：预留（可按你的项目实际检测）
        self.lbl_GPU_Basic_Val.setText("检测未实现")
        self.lbl_RAM_Val.setText("检测未实现")
        self.lbl_GPU_Count_Val.setText("--")
        self.lbl_CUDA_Val.setText("--")
        self.lbl_Vision_Val.setText("--")
        self._status("系统环境检查已更新（部分字段为预留接口）。", 3000)

    # ======================================================================
    # 关闭事件
    # ======================================================================
    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._closing = True
        try:
            if self.ser and self.ser.is_open:
                try:
                    self.read_timer.stop()
                    self.timer_gear.stop()
                    self.timer_servo.stop()
                    self.timer_cpg.stop()
                    self.ser.close()
                except Exception:
                    pass
        finally:
            event.accept()


# ======================================================================
# main
# ======================================================================
if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())
