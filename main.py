# main.py
# 适配 UI: Lanucher_V2.0.py（pyuic6 生成的 Ui_MainWindow）
# 依赖：PyQt6, pyserial(可选), LoraProtocol(必须)

from __future__ import annotations

import sys
import time
import re
import os
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple

from PyQt6 import QtCore, QtGui, QtWidgets
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QKeySequence, QShortcut
from PyQt6.QtWidgets import QApplication, QMainWindow, QMessageBox, QInputDialog

# ----------------------------- UI 导入 -----------------------------
Ui_MainWindow = None  # type: ignore


def _load_ui_class():
    global Ui_MainWindow
    candidates = [
        "Lanucher_UI",
        "Lanucher_V2",
        "Lanucher_UI_V2_0",
        "Lanucher_UI_V2_1",
    ]
    for mod_name in candidates:
        try:
            mod = __import__(mod_name, fromlist=["Ui_MainWindow"])
            Ui_MainWindow = getattr(mod, "Ui_MainWindow")
            return
        except Exception:
            pass

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
            "无法导入 Ui_MainWindow。请检查文件名。"
        ) from e


_load_ui_class()

# ----------------------------- 协议与串口依赖 -----------------------------
from LoraProtocol import *

try:
    import serial
    import serial.tools.list_ports as list_ports
except Exception as e:
    serial = None
    list_ports = None
    print("未安装 pyserial，请先 pip install pyserial：", e)


# ----------------------------- 设备状态数据结构 -----------------------------
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
    READ_POLL_MS = 25  # 串口读轮询周期
    CONT_SEND_MS = 50  # 连续发送周期（20Hz）
    MULTI_SEND_GAP_MS = 10  # 多设备分发间隔

    FRAME_HEADER = bytes([0xAA, 0x55])
    FRAME_TAIL = bytes([0x0D])

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)

        # 设置程序图标
        icon_path = "favicon.ico"
        if os.path.exists(icon_path):
            self.setWindowIcon(QtGui.QIcon(icon_path))

        # ---------- 串口 ----------
        self.ser: Optional[serial.Serial] = None
        self.current_port: Optional[str] = None
        self.default_baud = 115200
        self.current_baud = self.default_baud
        self._closing = False
        self.ctrl_version = None  # 存储握手探测到的版本 "V1.1" 或 "V2.0"

        # ---------- 设备 ----------
        self.tx_channel = int(self.spin_CtrlCh.value())
        self.default_password = 0x0000
        self.devices: Dict[int, FishDevice] = {}
        self.dev_state: Dict[int, DeviceState] = {}
        self.last_reply_enable = True

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

    # ... [保持辅助函数不变] ...
    def _init_log(self):
        try:
            self.txt_Log.document().setMaximumBlockCount(1500)
        except Exception:
            pass

    def _wire_ui(self):
        self.btn_OpenSerial.clicked.connect(self.toggle_serial)
        self.combo_Port.installEventFilter(self)
        self.btn_QueryCtrl.clicked.connect(self._query_ctrl_params)
        self.btn_InitCtrl.clicked.connect(self._init_controller)
        self.btn_SetCtrlCh.clicked.connect(self._set_ctrl_channel)
        self.btn_Search.clicked.connect(self._search_devices)
        self.btn_AddDevManual.clicked.connect(self._manual_add_device_dialog)
        self.btn_SelectAll.clicked.connect(self._select_all_targets)
        self.btn_SelectNone.clicked.connect(self._select_none_targets)
        self.btn_SelectNone_2.clicked.connect(self._pair_selected_devices)
        self.btn_SelectNone_3.clicked.connect(self._unpair_selected_devices)
        self.btn_GlobalStop.clicked.connect(self._global_stop_all)
        self.btn_GlobalStop_2.clicked.connect(self._global_stop_selected)
        self.btn_SendGear.clicked.connect(self._send_gear)
        self.chk_Sync_Gear.toggled.connect(self._toggle_gear_cont)
        self.btn_SendServo.clicked.connect(self._send_servo_position)
        self.chk_Sync_Servo.toggled.connect(self._toggle_servo_cont)
        self.btn_SetServoPower.clicked.connect(self._set_servo_power)
        self.btn_QueryServoAll.clicked.connect(self._query_servo_all_status)
        self.btn_SendCPG.clicked.connect(self._send_cpg)
        self.chk_Sync_CPG.toggled.connect(self._toggle_cpg_cont)
        self.btn_PlayStart.clicked.connect(self._start_play_mode)
        self.btn_FlashRead.clicked.connect(self._flash_read_config)
        self.btn_FlashSave.clicked.connect(self._flash_save_config)
        self.btn_SetAutoReport.clicked.connect(self._set_auto_report)
        self.btn_SetInstallBias.clicked.connect(self._set_install_bias)
        self.btn_ResetFault_6.clicked.connect(self._reply_switch_dialog)
        self.btn_ResetFault.clicked.connect(self._query_volt_power)
        self.btn_ResetFault_4.clicked.connect(self._query_servo_status_single)
        self.btn_ResetFault_3.clicked.connect(self._query_mos_temp)
        self.btn_ResetFault_5.clicked.connect(self._query_flash)
        self.btn_ResetFault_2.clicked.connect(self._reset_faulty_servo)
        self.btn_FactoryReset.clicked.connect(self._factory_reset)
        self.btn_CheckCompute.clicked.connect(self._check_compute_env)
        self.btn_ClearLog.clicked.connect(self.txt_Log.clear)
        self.chk_AutoScroll.toggled.connect(lambda _: None)

    def _init_shortcuts(self):
        QShortcut(QKeySequence("W"), self, activated=self.toggle_serial)
        QShortcut(QKeySequence("R"), self, activated=self._search_devices)
        QShortcut(QKeySequence("S"), self, activated=self._global_stop_selected)
        QShortcut(QKeySequence("Ctrl+N"), self, activated=self._manual_add_device_dialog)

    def _bind_value_pairs(self):
        self.dial_Speed.valueChanged.connect(self.spin_GearSpeed.setValue)
        self.spin_GearSpeed.valueChanged.connect(self.dial_Speed.setValue)
        self.dial_Turn.valueChanged.connect(self.spin_GearTurn.setValue)
        self.spin_GearTurn.valueChanged.connect(self.dial_Turn.setValue)
        self.slider_S1.valueChanged.connect(lambda v: self._servo_slider_to_spin(v, self.spin_S1))
        self.spin_S1.valueChanged.connect(lambda v: self._servo_spin_to_slider(v, self.slider_S1))
        self.slider_S2.valueChanged.connect(lambda v: self._servo_slider_to_spin(v, self.spin_S2))
        self.spin_S2.valueChanged.connect(lambda v: self._servo_spin_to_slider(v, self.slider_S2))
        self.slider_Amp.valueChanged.connect(lambda v: self.spin_Amp.setValue(v / 10.0))
        self.spin_Amp.valueChanged.connect(lambda v: self.slider_Amp.setValue(int(round(v * 10))))
        self.slider_Freq.valueChanged.connect(lambda v: self.spin_Freq.setValue(v / 10.0))
        self.spin_Freq.valueChanged.connect(lambda v: self.slider_Freq.setValue(int(round(v * 10))))
        self.slider_Bias.valueChanged.connect(lambda v: self.spin_Bias.setValue(v / 10.0))
        self.spin_Bias.valueChanged.connect(lambda v: self.slider_Bias.setValue(int(round(v * 10))))

    def _init_tables(self):
        self.table_Devices.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_Devices.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        self.table_SysMonitor.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_SysMonitor.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        sm = self.table_SysMonitor.selectionModel()
        if sm:
            sm.selectionChanged.connect(lambda *_: self._update_ui_state())

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

    def _refresh_ports(self, preserve: bool = True):
        if list_ports is None:
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

    def _update_ui_state(self):
        serial_open = bool(self.ser and self.ser.is_open)
        has_any_device = (len(self.devices) > 0)

        self.combo_Port.setEnabled(not serial_open)
        self.btn_OpenSerial.setEnabled(True)
        # comboBox (版本选择) 在串口连接后由逻辑决定是否禁用

        for w in [self.btn_InitCtrl, self.btn_QueryCtrl, self.btn_SetCtrlCh, self.spin_CtrlCh]:
            w.setEnabled(serial_open)

        self.btn_Search.setEnabled(serial_open)
        self.btn_AddDevManual.setEnabled(serial_open)
        self.btn_SelectAll.setEnabled(serial_open and has_any_device)
        self.btn_SelectNone.setEnabled(serial_open and has_any_device)
        self.btn_SelectNone_2.setEnabled(serial_open and self._selected_target_ids() != [])
        self.btn_SelectNone_3.setEnabled(serial_open and self._selected_target_ids() != [])

        self.btn_GlobalStop.setEnabled(serial_open and has_any_device)
        self.btn_GlobalStop_2.setEnabled(serial_open and self._selected_target_ids() != [])

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

        for w in [self.btn_FlashRead, self.btn_FlashSave, self.btn_SetAutoReport,
                  self.btn_SetInstallBias, self.btn_ResetFault_6, self.btn_ResetFault,
                  self.btn_ResetFault_4, self.btn_ResetFault_3, self.btn_ResetFault_5,
                  self.btn_ResetFault_2, self.btn_FactoryReset, self.btn_CheckCompute]:
            w.setEnabled(serial_open)

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
            self.comboBox.setEnabled(True)
            self.lbl_CtrlLinkState.setText("未连接")
            self.lbl_CtrlLinkState.setStyleSheet("color: red; font-weight: bold;")

    # ======================================================================
    # 串口开关 (含握手探测)
    # ======================================================================
    def toggle_serial(self):
        if serial is None:
            QMessageBox.warning(self, "缺少依赖", "未安装 pyserial，请先执行：pip install pyserial")
            return

        if self.ser and self.ser.is_open:
            # 关闭逻辑
            try:
                self.timer_gear.stop()
                self.timer_servo.stop()
                self.timer_cpg.stop()
                self.read_timer.stop()
                self.ser.close()
                self.ser = None
                self.current_port = None
                self._rx_buffer = b""
                self.ctrl_version = None
                self.btn_OpenSerial.setText("启动串口(&W)")
                self._status("串口已关闭。", 3000)

                self._clear_all_devices()
                self._update_ui_state()
            except Exception as e:
                QMessageBox.warning(self, "关闭失败", str(e))
            return

        # 打开逻辑
        port = (self.combo_Port.currentText() or "").strip()
        if (not port) or ("无可用端口" in port) or ("点击刷新" in port):
            QMessageBox.warning(self, "无效端口", "请选择一个有效串口。")
            return

        try:
            # 1. 打开串口，先设为 9600 用于握手
            self.ser = serial.Serial(
                port=port,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.05,
                rtscts=False, dsrdtr=False, xonxoff=False,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.current_port = port

            self.btn_OpenSerial.setText("关闭串口(&W)")
            self.lbl_CtrlLinkState.setText("连接中...")
            self.lbl_CtrlLinkState.setStyleSheet("color: orange; font-weight: bold;")
            self._update_ui_state()

            # 2. 握手探测
            detected_version = None

            def try_handshake(rts_val: bool) -> bool:
                self.ser.rts = rts_val
                self._sleep_ms(50)
                self.ser.reset_input_buffer()
                cmd = bytes([0xC1, 0x00, 0x07])
                self.ser.write(cmd)
                self.ser.flush()
                self._log(f"[HANDSHAKE] TX (RTS={rts_val}): C1 00 07")

                t0 = time.perf_counter()
                while (time.perf_counter() - t0) < 0.2:
                    if self.ser.in_waiting > 0:
                        raw = self.ser.read(self.ser.in_waiting)
                        self._log(f"[HANDSHAKE] RX: {raw.hex().upper()}")
                        return True
                    self._sleep_ms(10)
                return False

            if try_handshake(True):
                detected_version = "V1.1"
            else:
                if try_handshake(False):
                    detected_version = "V2.0"

            # 3. 设置结果
            if detected_version:
                self.ctrl_version = detected_version
                self._status(f"控制器握手成功：{detected_version}", 4000)
                self.lbl_CtrlLinkState.setText("已连接")
                self.lbl_CtrlLinkState.setStyleSheet("color: green; font-weight: bold;")

                idx = self.comboBox.findText(detected_version)
                if idx >= 0:
                    self.comboBox.setCurrentIndex(idx)
                self.comboBox.setEnabled(False)

                # 切回工作波特率，并设置正常工作时的RTS (V1.1=True, V2.0=False)
                self.ser.baudrate = 115200
                self.current_baud = 115200
                self.ser.rts = (detected_version == "V1.1")
                self._log(f"[SER] 切换波特率至 115200, RTS={self.ser.rts}")

                self.read_timer.start(self.READ_POLL_MS)

            else:
                self._status("控制器连接错误：无响应", 5000)
                self.lbl_CtrlLinkState.setText("连接错误")
                self.lbl_CtrlLinkState.setStyleSheet("color: red; font-weight: bold;")
                # 握手失败也切回波特率，防止卡死
                self.ser.baudrate = 115200
                self.read_timer.start(self.READ_POLL_MS)

        except Exception as e:
            self.ser = None
            QMessageBox.critical(self, "打开失败", f"{port} 打开失败：\n{e}")

    # ======================================================================
    # 控制器初始化与查询（实现部分）
    # ======================================================================
    def _enter_config_mode(self):
        """进入配置模式：9600波特率，根据版本设置RTS"""
        if not self._ensure_serial(): return False

        self.ser.baudrate = 9600
        # 配置模式下：V2.0 RTS=True, V1.1 RTS=False
        is_v2 = (self.comboBox.currentText() == "V2.0")
        self.ser.rts = True if is_v2 else False

        self._sleep_ms(50)
        self.ser.reset_input_buffer()
        self._log(f"[CFG_ENTER] 9600, RTS={self.ser.rts}")
        return True

    def _exit_config_mode(self):
        """退出配置模式：115200波特率，RTS反转"""
        if self.ser and self.ser.is_open:
            self.ser.baudrate = 115200
            # 正常模式下：V2.0 RTS=False, V1.1 RTS=True
            is_v2 = (self.comboBox.currentText() == "V2.0")
            self.ser.rts = False if is_v2 else True
            self._log(f"[CFG_EXIT] 115200, RTS={self.ser.rts}")
            self._sleep_ms(20)

    def _send_config_cmd_and_wait(self, cmd_hex: str, expected_hex: str = None, timeout_s: float = 0.3) -> Optional[
        bytes]:
        """在配置模式下发送指令并等待回包"""
        cmd_bytes = bytes.fromhex(cmd_hex)
        self.ser.write(cmd_bytes)
        self.ser.flush()

        rx_accum = b""
        t0 = time.perf_counter()
        while (time.perf_counter() - t0) < timeout_s:
            if self.ser.in_waiting > 0:
                rx_accum += self.ser.read(self.ser.in_waiting)
                # 如果指定了期望回包，且匹配到了，提前返回
                if expected_hex and bytes.fromhex(expected_hex) in rx_accum:
                    return rx_accum
            self._sleep_ms(5)

        return rx_accum if rx_accum else None

    # 1. 初始化控制器
    def _init_controller(self):
        if not self._ensure_serial(): return

        try:
            self.read_timer.stop()  # 暂停轮询，防止干扰
            self._enter_config_mode()

            # 发送初始化指令
            cmd = "C0 00 07 00 01 00 E7 00 17 43"
            expect = "C1 00 07 00 01 00 E7 00 17 43"

            rx = self._send_config_cmd_and_wait(cmd, expect, 0.4)

            success = False
            if rx and bytes.fromhex(expect) in rx:
                success = True
                self._status("初始化控制器成功！", 3000)
                QMessageBox.information(self, "成功", "控制器初始化成功。")
            else:
                self._status("初始化失败：超时或无正确回复。", 4000)
                QMessageBox.warning(self, "失败", "初始化失败，未收到正确回复。")

        except Exception as e:
            self._log(f"[INIT_ERR] {e}")
        finally:
            self._exit_config_mode()
            self.read_timer.start(self.READ_POLL_MS)

    # 2. 查询控制器参数
    def _query_ctrl_params(self):
        if not self._ensure_serial(): return

        try:
            self.read_timer.stop()
            self._enter_config_mode()

            cmd = "C1 00 07"
            rx = self._send_config_cmd_and_wait(cmd, None, 0.4)

            if rx and len(rx) >= 10:
                # 寻找帧头 C1
                start = rx.find(0xC1)
                if start >= 0 and len(rx[start:]) >= 10:
                    payload = rx[start + 3: start + 10]

                    # 解析
                    mod_addr = f"0x{payload[0]:02X}{payload[1]:02X}"
                    net_id = payload[2]

                    p3 = payload[3]
                    baud_idx = (p3 >> 5) & 0x07
                    parity_idx = (p3 >> 3) & 0x03
                    air_idx = p3 & 0x07

                    p4 = payload[4]
                    pkt_len_idx = (p4 >> 6) & 0x03
                    rssi_en = (p4 >> 5) & 0x01
                    tx_pwr_idx = p4 & 0x03

                    ch = payload[5] & 0x7F
                    trans_mode = (payload[6] >> 6) & 0x01

                    # 更新UI
                    self.tx_channel = ch
                    self.spin_CtrlCh.setValue(ch)

                    # 弹窗显示
                    msg = (
                        f"模块地址: {mod_addr}\n"
                        f"网络ID: {net_id}\n"
                        f"串口波特率: {self._baud_rate_to_str(baud_idx)}\n"
                        f"校验位: {self._parity_to_str(parity_idx)}\n"
                        f"空中速率: {self._air_speed_to_str(air_idx)}\n"
                        f"分包长度: {self._packet_length_to_str(pkt_len_idx)}\n"
                        f"RSSI上报: {'启用' if rssi_en else '禁用'}\n"
                        f"信道号: {ch}\n"
                        f"发射功率: {self._tx_power_to_str(tx_pwr_idx)}\n"
                        f"传输方式: {'定点传输' if trans_mode else '透明传输'}"
                    )
                    QMessageBox.information(self, "查询结果", msg)
                else:
                    QMessageBox.warning(self, "查询结果", f"回复数据长度不足或格式错误: {rx.hex().upper()}")
            else:
                QMessageBox.warning(self, "查询结果", "查询超时，无回复。")

        except Exception as e:
            self._log(f"[QUERY_ERR] {e}")
        finally:
            self._exit_config_mode()
            self.read_timer.start(self.READ_POLL_MS)

    def _set_ctrl_channel(self):
        if not self._ensure_serial(): return
        ch = int(self.spin_CtrlCh.value())
        # 预留：此处应下发配置指令，与_init_controller类似，只是指令内容不同
        self._todo(f"设置信道到 {ch} (需实现对应指令)")

    # 辅助转换函数
    def _baud_rate_to_str(self, idx):
        m = {0: "1200", 1: "2400", 2: "4800", 3: "9600", 4: "19200", 5: "38400", 6: "57600", 7: "115200"}
        return m.get(idx, f"Unknown({idx})")

    def _parity_to_str(self, idx):
        m = {0: "8N1", 1: "8O1", 2: "8E1", 3: "8N1"}
        return m.get(idx, f"Unknown({idx})")

    def _air_speed_to_str(self, idx):
        m = {0: "0.3k", 1: "1.2k", 2: "2.4k", 3: "4.8k", 4: "9.6k", 5: "19.2k", 6: "38.4k", 7: "62.5k"}
        return m.get(idx, f"Unknown({idx})")

    def _packet_length_to_str(self, idx):
        m = {0: "240B", 1: "128B", 2: "64B", 3: "32B"}
        return m.get(idx, f"Unknown({idx})")

    def _tx_power_to_str(self, idx):
        m = {0: "22dBm", 1: "17dBm", 2: "13dBm", 3: "10dBm"}
        return m.get(idx, f"Unknown({idx})")

    # ======================================================================
    # 串口收包与解析 (保持不变)
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

            while True:
                h = self._rx_buffer.find(self.FRAME_HEADER)
                if h < 0:
                    if len(self._rx_buffer) > 4096:
                        self._rx_buffer = b""
                    break
                t = self._rx_buffer.find(self.FRAME_TAIL, h + len(self.FRAME_HEADER))
                if t < 0:
                    if h > 0:
                        self._rx_buffer = self._rx_buffer[h:]
                    break

                pkt = self._rx_buffer[h:t + 1]
                self._rx_buffer = self._rx_buffer[t + 1:]
                self._handle_frame(pkt)

        except Exception as e:
            self._status(f"串口读取错误：{e}", 5000)

    def _handle_frame(self, pkt: bytes):
        try:
            parsed = parse_frame(pkt)
        except Exception as e:
            self._log(f"[PARSE_ERR] {e} | raw={pkt.hex(' ')}")
            return

        if not (parsed.get("head_ok") and parsed.get("tail_ok") and parsed.get("checksum_ok")):
            self._log("[FRAME] 无效帧：头尾或校验失败。")
            return

        cmd = parsed.get("cmd")
        fish_id = int(parsed.get("fish_id", 0))
        data = parsed.get("data", [])

        try:
            if cmd == Command.CMD_REPLY:
                self._handle_cmd_reply(fish_id, data, parsed)
            elif cmd == Command.VOLT_PWR_REPLY:
                self._handle_volt_power_reply(fish_id, data, parsed)
            elif cmd == getattr(Command, "STATE_REPLY_1", None):
                self._handle_state_reply_1(fish_id, data, parsed)
            elif cmd == getattr(Command, "STATE_REPLY_2", None):
                self._handle_state_reply_2(fish_id, data, parsed)
            elif cmd == getattr(Command, "STATE_REPLY_3", None):
                self._handle_state_reply_3(fish_id, data, parsed)
            else:
                self._log(f"[FRAME] 未处理的 CMD=0x{int(cmd):02X} 来自 0x{fish_id:04X}")
        except Exception as e:
            self._log(f"[DISPATCH_ERR] {e}")

    def _handle_cmd_reply(self, fish_id: int, data: list, parsed: dict):
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
        self._ensure_device_exists(fish_id)
        st = self.dev_state[fish_id]

        try:
            if len(data) >= 3:
                st.voltage_v = data[0] / 10.0
                st.power_mw = (data[1] << 8) | data[2]
                if not st.paired:
                    st.paired = True
                self._update_device_tables_row(fish_id)
                self._update_monitor_tables_row(fish_id)
                self._log(f"[PWR] 0x{fish_id:04X} V={st.voltage_v:.2f}V P={st.power_mw}mW")
        except Exception as e:
            self._log(f"[PWR_ERR] {e}")

    def _handle_state_reply_1(self, fish_id: int, data: list, parsed: dict):
        self._todo("STATE_REPLY_1 解析并更新 UI")

    def _handle_state_reply_2(self, fish_id: int, data: list, parsed: dict):
        self._todo("STATE_REPLY_2 解析并更新 UI")

    def _handle_state_reply_3(self, fish_id: int, data: list, parsed: dict):
        self._todo("STATE_REPLY_3 解析并更新 UI")

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
        it_sel = QtWidgets.QTableWidgetItem("")
        it_sel.setFlags(it_sel.flags() | Qt.ItemFlag.ItemIsUserCheckable)
        it_sel.setCheckState(Qt.CheckState.Unchecked)
        it_sel.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_SysMonitor.setItem(row, 0, it_sel)
        it_id = QtWidgets.QTableWidgetItem(f"0x{fish_id:04X}")
        it_id.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
        self.table_SysMonitor.setItem(row, 1, it_id)
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
        _set(5, "否" if st.mute else "是")
        _set(6, "-" if st.running is None else st.running)

    def _selected_target_ids(self) -> List[int]:
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

    def _search_devices(self):
        if not self._ensure_serial():
            return
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
            txt, ok = QInputDialog.getText(self, "手动添加设备", "输入 FishID（1~4位十六进制）：")
            if not ok:
                return
            s = (txt or "").strip().upper()
            if s.startswith("0X"):
                s = s[2:]
            if re.fullmatch(r"[0-9A-F]{1,4}", s):
                fid = int(s, 16)
                break
            QMessageBox.warning(self, "无效输入", "必须是 1~4 位十六进制。")
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
        pwd = int(self.spin_F_Pwd.value()) & 0xFFFF
        try:
            for dev in targets:
                if hasattr(dev, "set_password"):
                    dev.set_password(pwd, send=False)
            self._query_volt_power()
            self._status(f"已向 {len(targets)} 台设备发送配对流程。", 3000)
        except Exception as e:
            self._status(f"配对失败：{e}", 5000)

    def _unpair_selected_devices(self):
        self._todo("取消配对")

    def _global_stop_selected(self):
        if not self._ensure_serial():
            return
        targets = self._target_fishes()
        if not targets:
            self._status("未勾选目标设备。", 3000)
            return
        self._todo(f"急停(选中) {len(targets)} 台")

    def _global_stop_all(self):
        if not self._ensure_serial():
            return
        self._todo("急停(全部)")

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
        speed = int(self.spin_GearSpeed.value())
        turn = int(self.spin_GearTurn.value())
        try:
            for dev in targets:
                if hasattr(dev, "gear_ctrl"):
                    frame = dev.gear_ctrl(speed, turn)
                    self._send_bytes(frame, tag=f"GEAR 0x{dev.fish_id:04X} spd={speed} turn={turn}")
                else:
                    break
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            if not silent:
                self._status(f"已发送档位指令：{len(targets)} 台设备。", 2000)
        except Exception as e:
            self._status(f"发送档位失败：{e}", 5000)

    def _servo_slider_to_spin(self, slider_val: int, spin: QtWidgets.QDoubleSpinBox):
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
        mode_s1 = self.radio_S1_Only.isChecked()
        mode_s2 = self.radio_S2_Only.isChecked()
        mode_dual = self.radio_Dual_Sync.isChecked()
        ang1 = float(self.spin_S1.value())
        ang2 = float(self.spin_S2.value())
        try:
            for dev in targets:
                if mode_s1 and hasattr(dev, "servo_ctrl_sng"):
                    frame = dev.servo_ctrl_sng(1, ang1)
                    self._send_bytes(frame, tag=f"SERVO1 0x{dev.fish_id:04X} {ang1:.1f}")
                elif mode_s2 and hasattr(dev, "servo_ctrl_sng"):
                    frame = dev.servo_ctrl_sng(2, ang2)
                    self._send_bytes(frame, tag=f"SERVO2 0x{dev.fish_id:04X} {ang2:.1f}")
                elif mode_dual and hasattr(dev, "servo_ctrl_dbl"):
                    direction = 0x11 if ang1 >= 0 else 0x00
                    frame = dev.servo_ctrl_dbl(direction, ang1, ang2)
                    self._send_bytes(frame, tag=f"SERVO12 0x{dev.fish_id:04X} {ang1:.1f},{ang2:.1f}")
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
        self._todo(f"舵机供电：S1={'ON' if s1_on else 'OFF'} S2={'ON' if s2_on else 'OFF'}")

    def _query_servo_all_status(self):
        self._todo("查询全部舵机状态")

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
        amp = float(self.spin_Amp.value())
        freq = float(self.spin_Freq.value())
        bias = float(self.spin_Bias.value())
        try:
            for dev in targets:
                if hasattr(dev, "cpg_ctrl"):
                    frame = dev.cpg_ctrl(amp, bias, freq)
                    self._send_bytes(frame,
                                     tag=f"CPG 0x{dev.fish_id:04X} amp={amp:.1f} bias={bias:.1f} freq={freq:.1f}")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            if not silent:
                self._status(f"已发送 CPG 参数：{len(targets)} 台设备。", 2000)
        except Exception as e:
            self._status(f"发送 CPG 失败：{e}", 5000)

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
                    frame = dev.perf_ctrl(play_id)
                    self._send_bytes(frame, tag=f"PLAY 0x{dev.fish_id:04X} id={play_id}")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            self._status(f"已启动表演：{len(targets)} 台设备，序号={play_id}", 3000)
        except Exception as e:
            self._status(f"启动表演失败：{e}", 5000)

    def _flash_read_config(self):
        self._todo("读取配置")

    def _flash_save_config(self):
        self._todo("保存配置")

    def _set_auto_report(self):
        self._todo("自动回传配置")

    def _set_install_bias(self):
        self._todo("设置安装偏置")

    def _query_mos_temp(self):
        self._todo("查询 MOS 温度")

    def _query_flash(self):
        self._todo("查询 Flash")

    def _reset_faulty_servo(self):
        self._todo("复位损坏舵机")

    def _factory_reset(self):
        ok = QMessageBox.question(self, "恢复出厂设置", "确定继续吗？") == QMessageBox.StandardButton.Yes
        if ok:
            self._todo("恢复出厂设置")

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
                    frame = dev.replySwitch(enable)
                    self._send_bytes(frame, tag=f"Reply={'ON' if enable else 'OFF'} 0x{dev.fish_id:04X}")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            self._status(f"已设置消息应答：{'开启' if enable else '关闭'}", 3000)
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
                    frame = dev.query_voltage(auto=False)
                    self._send_bytes(frame, tag=f"QueryV/P 0x{dev.fish_id:04X}")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
            self._status(f"已发送电压/功率查询：{len(targets)} 台设备。", 2500)
        except Exception as e:
            self._status(f"查询电压/功率失败：{e}", 5000)

    def _query_servo_status_single(self):
        self._todo("查询舵机状态")

    def _check_compute_env(self):
        import platform
        try:
            self.lbl_OS_Val.setText(f"{platform.system()} {platform.release()}")
            self.lbl_CPU_Val.setText(platform.processor() or "未知")
        except Exception:
            self.lbl_OS_Val.setText("未知")
            self.lbl_CPU_Val.setText("未知")
        self.lbl_GPU_Basic_Val.setText("检测未实现")
        self.lbl_RAM_Val.setText("检测未实现")
        self.lbl_GPU_Count_Val.setText("--")
        self.lbl_CUDA_Val.setText("--")
        self.lbl_Vision_Val.setText("--")
        self._status("系统环境检查已更新（部分字段为预留接口）。", 3000)

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


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())