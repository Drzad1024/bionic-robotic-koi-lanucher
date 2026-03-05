# main.py

from __future__ import annotations

import sys
import time
import re
import json
import os
import secrets
import platform
import importlib
try:
    import winreg  # type: ignore
except Exception:
    winreg = None  # type: ignore
from dataclasses import dataclass
from typing import Optional, Dict, List, Tuple, Any, TYPE_CHECKING

from PySide6 import QtCore, QtGui, QtWidgets
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QKeySequence, QShortcut
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox, QInputDialog, QFileDialog

# ----------------------------- UI 导入 -----------------------------
Ui_MainWindow = None  # type: ignore


def _load_ui_class():
    global Ui_MainWindow
    candidates = [
        "Lanucher_UI",
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
        ui_path = Path(__file__).with_name("Lanucher_UI.py")
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
from LoraProtocol import LoraProtocol, Cmd, RunMode, LedMode

try:
    from system_monitor import SystemMonitor
except Exception:
    SystemMonitor = None  # type: ignore

if TYPE_CHECKING:
    import serial as serial_typing

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
    battery_pct: Optional[int] = None
    est_minutes: Optional[int] = None
    running: Optional[str] = None


# ----------------------------- 主窗口 -----------------------------
class MainWindow(QMainWindow, Ui_MainWindow):  # type: ignore[misc]
    READ_POLL_MS = 25  # 串口读轮询周期
    CONT_SEND_MS = 50  # 连续发送周期（20Hz）
    MULTI_SEND_GAP_MS = 10  # 多设备分发间隔
    UNPAIR_ACK_TIMEOUT_MS = 300  # 取消配对应答超时（超时按离线取消成功处理）

    FRAME_HEADER = bytes([0xAA, 0x66])

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setAttribute(Qt.WidgetAttribute.WA_DeleteOnClose, True)

        # 设置程序图标
        icon_path = "favicon.ico"
        if os.path.exists(icon_path):
            self.setWindowIcon(QtGui.QIcon(icon_path))

        # ---------- 串口 ----------
        self.ser: Optional["serial_typing.Serial"] = None
        self.current_port: Optional[str] = None
        self.default_baud = 115200
        self.current_baud = self.default_baud
        self._closing = False
        self.ctrl_version = None
        self.ctrl_id = 0x0001
        self._raw_rx_debug = False
        self._last_port_refresh_ts = 0.0
        self._auto_config_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "launcher_autosave.json")

        # ---------- 设备 ----------
        self.tx_channel = int(self.spin_CtrlCh.value())
        self.control_password = 0x0000
        self.pending_ctrl_code: Optional[int] = None
        self.pending_fish_id_change: Dict[int, int] = {}
        self.devices: Dict[int, LoraProtocol] = {}
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
        self._init_hex_fields()
        self._init_tables()
        self._init_env_panels()
        self._refresh_ports(preserve=False)
        self._load_auto_config()

        self._update_ui_state()
        self._status("准备就绪。请选择端口并点击“启动串口(&W)”。", 5000)

    # ======================================================================
    # UI：初始化与信号绑定
    # ======================================================================
    def _init_log(self):
        try:
            self.txt_Log.document().setMaximumBlockCount(2000)
        except Exception:
            pass

    def _wire_ui(self):
        # --- 菜单栏逻辑 ---
        # 文件
        self.action_Load.triggered.connect(self._menu_load_config)
        self.action_Save.triggered.connect(self._menu_save_config)
        self.action_Exit.triggered.connect(self.close)

        # 工具
        # 这里的 action 是 UI 设计里的“串口调试助手”占位
        self.action.triggered.connect(
            lambda: QMessageBox.information(self, "提示", "内置串口监视已启用，请查看下方【通讯日志】区域。"))

        # 帮助
        self.action_OpenSerial.triggered.connect(self.toggle_serial)  # 复用按钮逻辑
        self.action_SearchDev.triggered.connect(self._search_devices)  # 复用按钮逻辑
        self.action_Protocol.triggered.connect(self._menu_show_protocol)
        self.action_About.triggered.connect(self._menu_show_about)

        # --- 串口区 ---
        self.btn_OpenSerial.clicked.connect(self.toggle_serial)
        self.combo_Port.installEventFilter(self)

        # --- 控制器区 ---
        self.btn_QueryCtrl.clicked.connect(self._query_ctrl_params)
        self.btn_InitCtrl.clicked.connect(self._init_controller)
        self.btn_SetCtrlID.clicked.connect(self._set_ctrl_id)
        self.btn_SetCtrlCh.clicked.connect(self._set_ctrl_channel)

        if hasattr(self, "pushButton_SecondPWD_random"):
            self.pushButton_SecondPWD_random.clicked.connect(self._randomize_second_password)
        if hasattr(self, "pushButton_SecondPWD_set"):
            self.pushButton_SecondPWD_set.clicked.connect(self._apply_second_password)

        # --- 设备列表区 ---
        self.btn_Search.clicked.connect(self._search_devices)
        self.btn_AddDevManual.clicked.connect(self._manual_add_device_dialog)

        self.btn_SelectAll.clicked.connect(self._select_all_targets)
        self.btn_SelectNone.clicked.connect(self._select_none_targets)
        self.btn_SelectNone_2.clicked.connect(self._pair_selected_devices)
        self.btn_SelectNone_3.clicked.connect(self._unpair_selected_devices)

        # --- 系统监视区 ---
        self.btn_GlobalStop.clicked.connect(self._global_stop_all)
        self.btn_GlobalStop_2.clicked.connect(self._global_stop_selected)

        # --- 简易运动 ---
        self.btn_SendGear.clicked.connect(self._send_gear)
        self.btn_SetBootFromGear.clicked.connect(self._set_boot_from_gear)
        self.chk_Sync_Gear.toggled.connect(self._toggle_gear_cont)

        # --- 舵机综合管理 ---
        self.btn_SendServo.clicked.connect(self._send_servo_position)
        self.chk_Sync_Servo.toggled.connect(self._toggle_servo_cont)
        self.btn_SetServoPower.clicked.connect(self._set_servo_power)
        self.btn_QueryServoAll.clicked.connect(self._query_servo_all_status)
        self.btn_ServoQuickReset.clicked.connect(self._servo_quick_reset)
        if hasattr(self, "btn_ResetFault_ServoStat"):
            self.btn_ResetFault_ServoStat.clicked.connect(self._reset_faulty_servo)

        # --- 高阶运动 ---
        self.btn_SendCPG.clicked.connect(self._send_cpg)
        self.btn_SetBootFromCPG.clicked.connect(self._set_boot_from_cpg)
        self.chk_Sync_CPG.toggled.connect(self._toggle_cpg_cont)

        # --- 表演模式 ---
        self.btn_PlayStart.clicked.connect(self._start_play_mode)
        self.btn_PlayStop.clicked.connect(self._stop_play_mode)
        self.btn_SetBootFromPlay.clicked.connect(self._set_boot_from_play)

        # --- 开机参数 & 高级设置 ---
        self.btn_FlashRead.clicked.connect(self._flash_read_config)
        self.btn_FlashSave.clicked.connect(self._flash_save_config)
        if hasattr(self, "btn_EditFishID"):
            self.btn_EditFishID.clicked.connect(self._apply_fish_id)
        if hasattr(self, "btn_EditFishCh"):
            self.btn_EditFishCh.clicked.connect(self._apply_fish_channel)
        if hasattr(self, "btn_EditFishPwd"):
            self.btn_EditFishPwd.clicked.connect(self._apply_fish_password)
        self.btn_SetAutoReport.clicked.connect(self._set_auto_report)
        self.btn_ResetFault_6.clicked.connect(self._reply_switch_dialog)
        self.btn_ResetFault.clicked.connect(self._query_volt_power)
        self.btn_ResetFault_5.clicked.connect(self._query_flash)
        self.btn_FactoryReset.clicked.connect(self._factory_reset)

        # --- 系统环境 ---
        self.btn_CheckCompute.clicked.connect(self._check_compute_env)

        # --- 日志 ---
        self.btn_ClearLog.clicked.connect(self.txt_Log.clear)
        self.btn_SaveLogFile.clicked.connect(self._save_log_file)
        self.chk_AutoScroll.toggled.connect(lambda _: None)

        # --- 三维运动 ---
        if hasattr(self, "pushButton_2"):
            self.pushButton_2.clicked.connect(self._query_imu6_axis)
        if hasattr(self, "pushButton"):
            self.pushButton.clicked.connect(self._query_processed_motion)
        if hasattr(self, "pushButton_3"):
            self.pushButton_3.setCheckable(True)
            self.pushButton_3.clicked.connect(self._toggle_imu_auto_report)
        if hasattr(self, "pushButton_4"):
            self.pushButton_4.setCheckable(True)
            self.pushButton_4.clicked.connect(self._toggle_motion_auto_report)
        if hasattr(self, "btn_BLDC_Apply"):
            self.btn_BLDC_Apply.clicked.connect(self._apply_bldc_control)
        if hasattr(self, "btn_PID_Apply"):
            self.btn_PID_Apply.clicked.connect(self._apply_pid)
        if hasattr(self, "btn_HeadPitchApply"):
            self.btn_HeadPitchApply.clicked.connect(self._apply_head_pitch)

    def _init_shortcuts(self):
        QShortcut(QKeySequence("W"), self, activated=self.toggle_serial)
        QShortcut(QKeySequence("R"), self, activated=self._search_devices)
        QShortcut(QKeySequence("S"), self, activated=self._global_stop_selected)
        QShortcut(QKeySequence("T"), self, activated=self._stop_play_mode)
        QShortcut(QKeySequence("Ctrl+N"), self, activated=self._manual_add_device_dialog)
        # Ctrl+S 保存配置
        QShortcut(QKeySequence("Ctrl+S"), self, activated=self._menu_save_config)
        # Ctrl+O 载入配置
        QShortcut(QKeySequence("Ctrl+O"), self, activated=self._menu_load_config)

    def _bind_value_pairs(self):
        # Gear
        self.dial_Speed.valueChanged.connect(self.spin_GearSpeed.setValue)
        self.spin_GearSpeed.valueChanged.connect(self.dial_Speed.setValue)
        self.dial_Turn.valueChanged.connect(self.spin_GearTurn.setValue)
        self.spin_GearTurn.valueChanged.connect(self.dial_Turn.setValue)

        # Servo
        self.slider_S1.valueChanged.connect(lambda v: self._servo_slider_to_spin(v, self.spin_S1))
        self.spin_S1.valueChanged.connect(lambda v: self._servo_spin_to_slider(v, self.slider_S1))
        self.slider_S2.valueChanged.connect(lambda v: self._servo_slider_to_spin(v, self.spin_S2))
        self.spin_S2.valueChanged.connect(lambda v: self._servo_spin_to_slider(v, self.slider_S2))

        # CPG
        self.slider_Amp.valueChanged.connect(lambda v: self.spin_Amp.setValue(v / 10.0))
        self.spin_Amp.valueChanged.connect(lambda v: self.slider_Amp.setValue(int(round(v * 10))))
        self.slider_Freq.valueChanged.connect(lambda v: self.spin_Freq.setValue(v / 10.0))
        self.spin_Freq.valueChanged.connect(lambda v: self.slider_Freq.setValue(int(round(v * 10))))
        self.slider_Bias.valueChanged.connect(lambda v: self.spin_Bias.setValue(v / 10.0))
        self.spin_Bias.valueChanged.connect(lambda v: self.slider_Bias.setValue(int(round(v * 10))))

    def _init_tables(self):
        self.table_Devices.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_Devices.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.SingleSelection)
        dsm = self.table_Devices.selectionModel()
        if dsm:
            dsm.selectionChanged.connect(lambda *_: self._update_ui_state())

        self.table_SysMonitor.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectionBehavior.SelectRows)
        self.table_SysMonitor.setSelectionMode(QtWidgets.QAbstractItemView.SelectionMode.ExtendedSelection)
        self.table_SysMonitor.itemChanged.connect(lambda *_: (self._update_ui_state(), self._sync_fid_from_monitor_selection()))

        sm = self.table_SysMonitor.selectionModel()
        if sm:
            sm.selectionChanged.connect(lambda *_: (self._update_ui_state(), self._sync_fid_from_monitor_selection()))

    def _sync_fid_from_monitor_selection(self):
        target_id: Optional[int] = None

        sm = self.table_SysMonitor.selectionModel()
        if sm:
            rows = sm.selectedRows(1)
            if rows:
                try:
                    target_id = int((rows[0].data() or "").strip(), 16)
                except Exception:
                    target_id = None

        if target_id is None:
            for r in range(self.table_SysMonitor.rowCount()):
                it_sel = self.table_SysMonitor.item(r, 0)
                it_id = self.table_SysMonitor.item(r, 1)
                if (it_sel is None) or (it_id is None):
                    continue
                if it_sel.checkState() == Qt.CheckState.Checked:
                    try:
                        target_id = int((it_id.text() or "").strip(), 16)
                    except Exception:
                        target_id = None
                    break

        if target_id is not None:
            self._set_hex_widget_value(getattr(self, "txt_F_ID", None), target_id)

    def _format_hex16(self, value: int) -> str:
        return f"0x{(int(value) & 0xFFFF):04X}"

    def _format_hex8(self, value: int) -> str:
        return f"0x{(int(value) & 0xFF):02X}"

    def _set_hex_widget_value(self, widget: Any, value: int) -> None:
        if widget is None:
            return
        val = int(value) & 0xFFFF
        if hasattr(widget, "setValue"):
            try:
                widget.setValue(val)
                return
            except Exception:
                pass

        txt = self._format_hex16(val)
        if widget is getattr(self, "lbl_CtrlID_Val", None):
            txt = self._format_hex8(val)
        if hasattr(widget, "setPlainText") and hasattr(widget, "toPlainText"):
            try:
                if widget.toPlainText() == txt:
                    return
                old = widget.blockSignals(True)
                widget.setPlainText(txt)
                widget.blockSignals(old)
                cursor = widget.textCursor()
                cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
                widget.setTextCursor(cursor)
                return
            except Exception:
                pass

        if hasattr(widget, "setText"):
            try:
                widget.setText(txt)
            except Exception:
                pass

    def _get_hex_widget_value(self, widget: Any, default: int = 0x0001) -> int:
        if widget is None:
            return int(default) & 0xFFFF

        if hasattr(widget, "value"):
            try:
                return int(widget.value()) & 0xFFFF
            except Exception:
                return int(default) & 0xFFFF

        text = ""
        if hasattr(widget, "toPlainText"):
            try:
                text = widget.toPlainText().strip()
            except Exception:
                text = ""
        elif hasattr(widget, "text"):
            try:
                text = widget.text().strip()
            except Exception:
                text = ""

        m = re.fullmatch(r"0x([0-9A-Fa-f]{4})", text)
        if m:
            return int(m.group(1), 16)

        hex_chars = ''.join(ch for ch in text if ch in "0123456789abcdefABCDEF")
        if hex_chars:
            return int(hex_chars[-4:], 16)

        return int(default) & 0xFFFF

    def _normalize_hex_widget(self, widget: Any, default: int = 0x0001) -> None:
        self._set_hex_widget_value(widget, self._get_hex_widget_value(widget, default))

    def _bind_hex_plaintext(self, widget: Any, default: int = 0x0001) -> None:
        if widget is None or (not hasattr(widget, "textChanged")):
            return
        self._set_hex_widget_value(widget, default)
        widget.textChanged.connect(lambda: self._normalize_hex_widget(widget, default))

    def _init_hex_fields(self):
        self._bind_hex_plaintext(getattr(self, "txt_F_ID", None), 0x0001)
        self._bind_hex_plaintext(getattr(self, "txt_F_Pwd", None), 0x0000)

        if hasattr(self, "lbl_CtrlID_Val"):
            self.lbl_CtrlID_Val.setMaxLength(6)
            self.lbl_CtrlID_Val.textChanged.connect(self._normalize_ctrl_id_input)
            self._set_hex_widget_value(self.lbl_CtrlID_Val, self.ctrl_id)

        if hasattr(self, "lineEdit_SecondPWD"):
            self.lineEdit_SecondPWD.setMaxLength(4)
            self.lineEdit_SecondPWD.setAlignment(Qt.AlignmentFlag.AlignCenter)
            self.lineEdit_SecondPWD.textChanged.connect(self._normalize_second_password_text)
            self._randomize_second_password(silent=True)

    def _normalize_second_password_text(self):
        if not hasattr(self, "lineEdit_SecondPWD"):
            return
        old = self.lineEdit_SecondPWD.blockSignals(True)
        txt = (self.lineEdit_SecondPWD.text() or "").upper()
        txt = ''.join(ch for ch in txt if ch in "0123456789ABCDEF")[:4]
        self.lineEdit_SecondPWD.setText(txt)
        self.lineEdit_SecondPWD.blockSignals(old)

    def _second_password_value(self) -> Optional[int]:
        if not hasattr(self, "lineEdit_SecondPWD"):
            return 0xBBCC
        txt = (self.lineEdit_SecondPWD.text() or "").strip().upper()
        if not re.fullmatch(r"[0-9A-F]{4}", txt):
            return None
        return int(txt, 16) & 0xFFFF

    def _randomize_second_password(self, silent: bool = False):
        if not hasattr(self, "lineEdit_SecondPWD"):
            return
        value = secrets.randbelow(0x10000)
        self.lineEdit_SecondPWD.setText(f"{value:04X}")
        if not silent:
            self._status("已随机生成控制口令。", 2000)

    def _apply_second_password(self):
        ctrl_code = self._second_password_value()
        if ctrl_code is None:
            QMessageBox.warning(self, "输入错误", "控制口令格式错误，请输入4位16进制（如 A1B2）。")
            return
        self.control_password = ctrl_code
        self._status(f"控制口令已更新：{self._format_hex16(ctrl_code)}", 2500)

    def _normalize_ctrl_id_input(self):
        if not hasattr(self, "lbl_CtrlID_Val"):
            return
        old = self.lbl_CtrlID_Val.blockSignals(True)
        txt = (self.lbl_CtrlID_Val.text() or "").strip().upper()
        hex_chars = ''.join(ch for ch in txt if ch in "0123456789ABCDEF")
        two_hex = (hex_chars[-2:] if hex_chars else "00").rjust(2, "0")
        self.lbl_CtrlID_Val.setText(f"0x{two_hex}")
        self.lbl_CtrlID_Val.blockSignals(old)

    # ======================================================================
    # 菜单逻辑实现
    # ======================================================================
    def _collect_config_data(self) -> Dict[str, Any]:
        known_ids = sorted({int(fid) & 0xFFFF for fid in (list(self.devices.keys()) + list(self.dev_state.keys()))})

        ctrl_pwd = self._second_password_value()
        if ctrl_pwd is None:
            ctrl_pwd = int(self.control_password) & 0xFFFF

        cfg: Dict[str, Any] = {
            "schema": "launcher.config.v2",
            "timestamp": time.time(),
            "serial": {
                "port": (self.current_port or self.combo_Port.currentText() or "").strip(),
                "ctrl_id": int(self.ctrl_id) & 0xFF,
                "ctrl_ch": int(self.spin_CtrlCh.value()) & 0xFF,
            },
            "security": {
                "control_password": int(ctrl_pwd) & 0xFFFF,
            },
            "devices": {
                "known_ids": [f"0x{fid:04X}" for fid in known_ids],
            },
            "ui": {
                "gear_spd": self.spin_GearSpeed.value(),
                "gear_turn": self.spin_GearTurn.value(),
                "cpg_amp": self.spin_Amp.value(),
                "cpg_freq": self.spin_Freq.value(),
                "cpg_bias": self.spin_Bias.value(),
                "boot_id": self._get_hex_widget_value(self.txt_F_ID, 0x0001),
                "boot_ch": self.spin_F_Ch.value(),
                "fish_pwd": self._get_hex_widget_value(self.txt_F_Pwd, 0x0000),
            },
        }
        return cfg

    def _parse_fish_id_value(self, value: Any) -> Optional[int]:
        try:
            if isinstance(value, int):
                fid = int(value)
            elif isinstance(value, str):
                txt = value.strip()
                if not txt:
                    return None
                if txt.lower().startswith("0x"):
                    fid = int(txt, 16)
                else:
                    fid = int(txt, 0)
            else:
                return None
            if 0 <= fid <= 0xFFFF:
                return fid
        except Exception:
            return None
        return None

    def _set_saved_port(self, port: str):
        p = (port or "").strip()
        if not p:
            return
        idx = self.combo_Port.findText(p, Qt.MatchFlag.MatchFixedString)
        if idx < 0:
            self.combo_Port.addItem(p)
            idx = self.combo_Port.findText(p, Qt.MatchFlag.MatchFixedString)
        if idx >= 0:
            self.combo_Port.setCurrentIndex(idx)

    def _apply_config_data(self, cfg: Dict[str, Any], source: str = "manual"):
        serial_cfg = cfg.get("serial") if isinstance(cfg.get("serial"), dict) else {}
        sec_cfg = cfg.get("security") if isinstance(cfg.get("security"), dict) else {}
        dev_cfg = cfg.get("devices") if isinstance(cfg.get("devices"), dict) else {}
        ui_cfg = cfg.get("ui") if isinstance(cfg.get("ui"), dict) else cfg

        saved_port = serial_cfg.get("port")
        if isinstance(saved_port, str):
            self._set_saved_port(saved_port)

        ctrl_ch = serial_cfg.get("ctrl_ch", cfg.get("ctrl_ch"))
        if isinstance(ctrl_ch, (int, float)):
            self.spin_CtrlCh.setValue(int(ctrl_ch) & 0xFF)
            self.tx_channel = int(ctrl_ch) & 0xFF

        ctrl_id = serial_cfg.get("ctrl_id", cfg.get("ctrl_id"))
        if isinstance(ctrl_id, (int, float)):
            self.ctrl_id = int(ctrl_id) & 0xFF
            self._set_hex_widget_value(getattr(self, "lbl_CtrlID_Val", None), self.ctrl_id)

        ctrl_pwd = sec_cfg.get("control_password", cfg.get("control_password"))
        if isinstance(ctrl_pwd, (int, float)):
            pwd = int(ctrl_pwd) & 0xFFFF
            self.control_password = pwd
            if hasattr(self, "lineEdit_SecondPWD"):
                self.lineEdit_SecondPWD.setText(f"{pwd:04X}")

        if "gear_spd" in ui_cfg:
            self.spin_GearSpeed.setValue(int(ui_cfg["gear_spd"]))
        if "gear_turn" in ui_cfg:
            self.spin_GearTurn.setValue(int(ui_cfg["gear_turn"]))
        if "cpg_amp" in ui_cfg:
            self.spin_Amp.setValue(float(ui_cfg["cpg_amp"]))
        if "cpg_freq" in ui_cfg:
            self.spin_Freq.setValue(float(ui_cfg["cpg_freq"]))
        if "cpg_bias" in ui_cfg:
            self.spin_Bias.setValue(float(ui_cfg["cpg_bias"]))
        if "boot_id" in ui_cfg:
            self._set_hex_widget_value(self.txt_F_ID, int(ui_cfg["boot_id"]))
        if "boot_ch" in ui_cfg:
            self.spin_F_Ch.setValue(int(ui_cfg["boot_ch"]))
        if "fish_pwd" in ui_cfg:
            self._set_hex_widget_value(self.txt_F_Pwd, int(ui_cfg["fish_pwd"]))

        known_ids_raw = dev_cfg.get("known_ids", cfg.get("known_devices", []))
        if isinstance(known_ids_raw, list):
            known_ids: List[int] = []
            seen = set()
            for raw in known_ids_raw:
                fid = self._parse_fish_id_value(raw)
                if fid is None:
                    continue
                if fid in seen:
                    continue
                seen.add(fid)
                known_ids.append(fid)

            self._clear_all_devices()
            for fid in sorted(known_ids):
                self._ensure_device_exists(fid)
                st = self.dev_state.get(fid)
                if st is not None:
                    st.paired = False
                self._update_device_tables_row(fid)

            # 新开软件或加载配置后，系统监视保持为空（配对需重新执行）
            self.table_SysMonitor.setRowCount(0)

        self._update_ui_state()
        if source == "auto":
            self._log("[CFG] 已自动加载本地配置。")
        else:
            self._status("配置已导入。", 2500)

    def _save_config_to_path(self, path: str, source: str = "manual"):
        cfg = self._collect_config_data()
        with open(path, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, ensure_ascii=False, indent=4)
        if source == "auto":
            self._log(f"[CFG] 自动保存配置: {path}")
        else:
            self._status(f"配置已保存：{path}")

    def _load_config_from_path(self, path: str, source: str = "manual"):
        with open(path, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        if not isinstance(cfg, dict):
            raise ValueError("配置格式错误：根节点应为对象")
        self._apply_config_data(cfg, source=source)

    def _load_auto_config(self):
        if not os.path.exists(self._auto_config_path):
            return
        try:
            self._load_config_from_path(self._auto_config_path, source="auto")
            self._status("已自动加载上次配置。", 2000)
        except Exception as e:
            self._log(f"[CFG_AUTO_LOAD_ERR] {e}")

    def _save_auto_config(self):
        try:
            self._save_config_to_path(self._auto_config_path, source="auto")
        except Exception as e:
            self._log(f"[CFG_AUTO_SAVE_ERR] {e}")

    def _menu_load_config(self):
        """载入配置文件 (JSON)"""
        path, _ = QFileDialog.getOpenFileName(self, "载入配置", "", "JSON Files (*.json);;All Files (*)")
        if not path:
            return

        try:
            self._load_config_from_path(path, source="manual")
            self._status(f"配置已载入：{os.path.basename(path)}", 2500)
        except Exception as e:
            QMessageBox.warning(self, "载入失败", f"无法解析配置文件：\n{e}")

    def _menu_save_config(self):
        """保存当前 UI 配置到文件"""
        path, _ = QFileDialog.getSaveFileName(self, "保存配置", "launcher_config.json", "JSON Files (*.json)")
        if not path:
            return

        try:
            self._save_config_to_path(path, source="manual")
        except Exception as e:
            QMessageBox.warning(self, "保存失败", f"写入文件错误：\n{e}")

    def _menu_show_protocol(self):
        info = (
            "<b>仿生机器锦鲤通讯协议 V2.1</b><br><br>"
            "1. <b>通讯格式</b>：UART, 115200bps（设置参数时 9600）, 8N1<br>"
            "2. <b>帧结构</b>：Header(AA 66) + SenderID(2) + Password(2) + LEN(1) + CMD(1) + Payload(N) + XOR(1)<br>"
            "3. <b>主要功能</b>：<br>"
            "   - 0x90~0x91: 配对/取消配对<br>"
            "   - 0xA*: 运动控制 (停止/CPG/舵机/表演/档位/电调)<br>"
            "   - 0xB*: 系统配置 (回复/LED/舵机电源/自动回传/搜索)<br>"
            "   - 0xC*: 查询与传感器 (状态/舵机/开机参数/Flash/版本/IMU/姿态/深度)<br>"
            "   - 0xD*: 开机参数写入 (模式/回复/偏置/过流/ID/密码/信道/复位标志)<br>"
            "<br>详细内容请参考开发文档。"
        )
        QMessageBox.information(self, "协议说明", info)

    def _menu_show_about(self):
        QMessageBox.about(self, "关于",
                          "<b>仿生机器锦鲤控制台 V2.5</b><br>"
                          "Bionic Robotic Koi Launcher<br><br>"
                          "基于 PyQt6 & Python<br>"
                          "适配仿生机器锦鲤通讯协议 V2.1<br>"
                          "Copyright © IBD-Lab 2025")

    # ======================================================================
    # 辅助工具
    # ======================================================================
    def _status(self, msg: str, ms: int = 3000):
        try:
            self.statusBar().showMessage(msg, ms)
        except Exception:
            pass

    def _log(self, text: str):
        # 统一日志入口，确保所有收发都经过此处
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

    @staticmethod
    def _xor_checksum(buf: bytes) -> int:
        x = 0
        for b in buf:
            x ^= b
        return x & 0xFF

    @staticmethod
    def _run_mode_text(mode: Optional[int]) -> str:
        mp = {
            0: "停止",
            1: "CPG",
            2: "CPG+间歇",
            3: "舵机位置",
            4: "表演",
            5: "档位",
            7: "保护",
        }
        if mode is None:
            return "-"
        return mp.get(mode, f"未知({mode})")

    def _build_v21_frame(self, sender_id: int, password: int, cmd: int, payload: bytes = b"") -> bytes:
        payload = payload or b""
        body = bytearray()
        body += self.FRAME_HEADER
        body += int(sender_id & 0xFFFF).to_bytes(2, "big")
        body += int(password & 0xFFFF).to_bytes(2, "big")
        body.append((1 + len(payload) + 1) & 0xFF)  # CMD + Payload + XOR
        body.append(cmd & 0xFF)
        body += payload
        body.append(self._xor_checksum(bytes(body)))
        return bytes(body)

    def _wrap_target_channel(self, target_id: int, channel: int, frame: bytes) -> bytes:
        return int(target_id & 0xFFFF).to_bytes(2, "big") + bytes([int(channel) & 0xFF]) + frame

    def _send_v21_cmd(self, target_id: int, cmd: int, payload: bytes = b"", tag: str = "",
                      password: Optional[int] = None) -> bool:
        pwd = self.control_password if password is None else int(password)
        inner = self._build_v21_frame(self.ctrl_id, pwd, cmd, payload)
        frame = self._wrap_target_channel(target_id, self.tx_channel, inner)
        return self._send_bytes(frame, tag=tag or f"CMD 0x{cmd:02X}")

    def _extract_v21_frames(self) -> List[bytes]:
        packets: List[bytes] = []
        while True:
            h = self._rx_buffer.find(self.FRAME_HEADER)
            if h < 0:
                if len(self._rx_buffer) > 2048:
                    self._rx_buffer = b""
                break

            if h > 0:
                self._rx_buffer = self._rx_buffer[h:]

            if len(self._rx_buffer) < 8:
                break

            length = self._rx_buffer[6]
            total_len = 7 + int(length)

            if length < 2 or total_len > 512:
                self._rx_buffer = self._rx_buffer[1:]
                continue

            if len(self._rx_buffer) < total_len:
                break

            pkt = self._rx_buffer[:total_len]
            self._rx_buffer = self._rx_buffer[total_len:]
            packets.append(pkt)

        return packets

    def _parse_v21_frame(self, pkt: bytes) -> Optional[Dict[str, Any]]:
        if len(pkt) < 9:
            return None
        if pkt[0:2] != self.FRAME_HEADER:
            return None

        length = pkt[6]
        expected = 7 + int(length)
        if expected != len(pkt) or length < 2:
            return None

        xor_rx = pkt[-1]
        xor_calc = self._xor_checksum(pkt[:-1])
        if xor_rx != xor_calc:
            return None

        sender_id = int.from_bytes(pkt[2:4], "big")
        password = int.from_bytes(pkt[4:6], "big")
        cmd = pkt[7]
        payload = pkt[8:-1]
        return {
            "sender_id": sender_id,
            "password": password,
            "cmd": cmd,
            "payload": payload,
        }

    def _refresh_ports(self, preserve: bool = True):
        if list_ports is None:
            self.combo_Port.clear()
            self.combo_Port.addItem("点击刷新")
            return

        current = self.combo_Port.currentText() if preserve else None
        try:
            ports_info = list_ports.comports(include_links=True)
        except TypeError:
            ports_info = list_ports.comports()

        ports = []
        seen = set()
        for p in ports_info:
            dev = (getattr(p, "device", "") or "").strip()
            if not dev:
                dev = (getattr(p, "name", "") or "").strip()
            if not dev:
                continue
            key = dev.upper()
            if key in seen:
                continue
            seen.add(key)
            ports.append(dev)

        if os.name == "nt" and winreg is not None:
            reg_ports = self._read_windows_serialcomm_ports()
            for dev in reg_ports:
                key = dev.upper()
                if key in seen:
                    continue
                seen.add(key)
                ports.append(dev)

        def _port_key(name: str):
            m = re.fullmatch(r"COM(\d+)", name.upper())
            return (0, int(m.group(1))) if m else (1, name.upper())

        ports.sort(key=_port_key)

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

    def _read_windows_serialcomm_ports(self) -> List[str]:
        if (os.name != "nt") or (winreg is None):
            return []
        result: List[str] = []
        key_path = r"HARDWARE\DEVICEMAP\SERIALCOMM"
        try:
            with winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE, key_path) as key:
                count = winreg.QueryInfoKey(key)[1]
                for i in range(count):
                    try:
                        _, value, _ = winreg.EnumValue(key, i)
                        name = str(value).strip().upper()
                        if re.fullmatch(r"COM\d+", name):
                            result.append(name)
                    except Exception:
                        continue
        except Exception:
            return []
        return result

    def eventFilter(self, obj, event):
        try:
            if obj is self.combo_Port and event.type() == QtCore.QEvent.Type.MouseButtonPress:
                if self.combo_Port.isEnabled():
                    now = time.monotonic()
                    if (now - self._last_port_refresh_ts) > 0.35:
                        self._last_port_refresh_ts = now
                        self._refresh_ports(preserve=True)
                        self._status(f"已刷新串口列表（{self.combo_Port.count()}个）。", 1500)
        except Exception:
            pass
        return super().eventFilter(obj, event)

    def _update_ui_state(self):
        serial_open = bool(self.ser and self.ser.is_open)
        has_any_device = (self.table_Devices.rowCount() > 0)
        has_any_monitor_device = (self.table_SysMonitor.rowCount() > 0)
        selected_device_ids = self._selected_device_ids_in_devices_table()
        can_pair = serial_open and any((not self.dev_state.get(fid, DeviceState(fid)).paired) for fid in selected_device_ids)
        can_unpair = serial_open and any(self.dev_state.get(fid, DeviceState(fid)).paired for fid in selected_device_ids)

        self.combo_Port.setEnabled(not serial_open)
        self.btn_OpenSerial.setEnabled(True)

        for w in [self.btn_InitCtrl, self.btn_QueryCtrl, self.btn_SetCtrlID, self.btn_SetCtrlCh, self.spin_CtrlCh]:
            w.setEnabled(serial_open)

        self.btn_Search.setEnabled(serial_open)
        self.btn_AddDevManual.setEnabled(True)
        self.btn_SelectAll.setEnabled(serial_open and has_any_monitor_device)
        self.btn_SelectNone.setEnabled(serial_open and has_any_monitor_device)
        self.btn_SelectNone_2.setEnabled(can_pair)
        self.btn_SelectNone_3.setEnabled(can_unpair)

        self.btn_GlobalStop.setEnabled(serial_open and has_any_monitor_device)
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
        self.btn_PlayStop.setEnabled(targets_ok)
        self.btn_SetBootFromGear.setEnabled(targets_ok)
        self.btn_SetBootFromCPG.setEnabled(targets_ok)
        self.btn_SetBootFromPlay.setEnabled(targets_ok)
        self.btn_ServoQuickReset.setEnabled(targets_ok)
        if hasattr(self, "pushButton"):
            self.pushButton.setEnabled(targets_ok)
        if hasattr(self, "pushButton_2"):
            self.pushButton_2.setEnabled(targets_ok)
        if hasattr(self, "pushButton_3"):
            self.pushButton_3.setEnabled(targets_ok)
        if hasattr(self, "pushButton_4"):
            self.pushButton_4.setEnabled(targets_ok)
        if hasattr(self, "btn_BLDC_Apply"):
            self.btn_BLDC_Apply.setEnabled(targets_ok)
        if hasattr(self, "btn_PID_Apply"):
            self.btn_PID_Apply.setEnabled(targets_ok)
        if hasattr(self, "btn_HeadPitchApply"):
            self.btn_HeadPitchApply.setEnabled(targets_ok)

        for w in [self.btn_FlashRead, self.btn_FlashSave, self.btn_SetAutoReport,
                  self.btn_ResetFault_6, self.btn_ResetFault,
                  self.btn_ResetFault_5, self.btn_FactoryReset]:
            w.setEnabled(serial_open)

        self.btn_CheckCompute.setEnabled(True)
        self.btn_ClearLog.setEnabled(True)
        self.btn_SaveLogFile.setEnabled(True)

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
            self._set_link_state("未连接", "red")

    def _set_link_state(self, text: str, color: str):
        self.lbl_CtrlLinkState.setText(text)
        self.lbl_CtrlLinkState.setStyleSheet(f"color: {color}; font-weight: bold;")

    def _close_serial(self, clear_devices: bool = True):
        self.timer_gear.stop()
        self.timer_servo.stop()
        self.timer_cpg.stop()
        self.read_timer.stop()

        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass

        self.ser = None
        self.current_port = None
        self._rx_buffer = b""
        self.ctrl_version = None
        self.current_baud = self.default_baud
        self.btn_OpenSerial.setChecked(False)
        self.btn_OpenSerial.setText("启动串口(&W)")
        if clear_devices:
            self._clear_all_devices()

    # ======================================================================
    # 串口开关 (含握手)
    # ======================================================================
    def toggle_serial(self):
        if serial is None:
            QMessageBox.warning(self, "缺少依赖", "未安装 pyserial，请先执行：pip install pyserial")
            return

        if self.ser and self.ser.is_open:
            try:
                self._close_serial(clear_devices=True)
                self._status("串口已关闭。", 3000)
                self._update_ui_state()
            except Exception as e:
                QMessageBox.warning(self, "关闭失败", str(e))
            return

        port = (self.combo_Port.currentText() or "").strip()
        if (not port) or ("无可用端口" in port) or ("点击刷新" in port):
            QMessageBox.warning(self, "无效端口", "请选择一个有效串口。")
            return

        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=9600,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.03,
                write_timeout=0.2,
                inter_byte_timeout=0.03,
                rtscts=False, dsrdtr=False, xonxoff=False,
            )
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            self.current_port = port

            self.btn_OpenSerial.setChecked(True)
            self.btn_OpenSerial.setText("关闭串口(&W)")
            self._set_link_state("连接中...", "orange")
            self._update_ui_state()

            # 2. 握手探测
            detected_version = None

            def _apply_ctrl_params_from_handshake(rx_bytes: bytes):
                head = bytes([0xC1, 0x00, 0x07])
                start = rx_bytes.find(head)
                if start < 0 or len(rx_bytes[start:]) < 10:
                    return
                payload = rx_bytes[start + 3: start + 10]
                ctrl_id = ((payload[0] << 8) | payload[1]) & 0x00FF
                ch = payload[5] & 0x7F
                self.ctrl_id = ctrl_id
                self.tx_channel = ch
                self.spin_CtrlCh.setValue(ch)
                self._set_hex_widget_value(getattr(self, "lbl_CtrlID_Val", None), ctrl_id)
                self._log(f"[HANDSHAKE] 参数回填: CtrlID={self._format_hex8(ctrl_id)}, CH={ch}")

            def try_handshake(rts_val: bool) -> bool:
                try:
                    self.ser.rts = rts_val
                    self._sleep_ms(50)
                    self.ser.reset_input_buffer()
                    cmd = bytes([0xC1, 0x00, 0x07])
                    self.ser.write(cmd)
                    self.ser.flush()
                    self._log(f"[HANDSHAKE] TX (RTS={rts_val}): C1 00 07")

                    rx_accum = b""
                    t0 = time.perf_counter()
                    while (time.perf_counter() - t0) < 0.2:
                        if self.ser.in_waiting > 0:
                            raw = self.ser.read(min(self.ser.in_waiting, 128))
                            rx_accum += raw
                            self._log(f"[HANDSHAKE] RX: {raw.hex(' ').upper()}")
                            head = bytes([0xC1, 0x00, 0x07])
                            start = rx_accum.find(head)
                            if start >= 0 and len(rx_accum[start:]) >= 10:
                                _apply_ctrl_params_from_handshake(rx_accum)
                                return True
                        self._sleep_ms(10)
                except Exception as e:
                    self._log(f"[HANDSHAKE_ERR] RTS={rts_val} err={e}")
                return False

            if try_handshake(True):
                detected_version = "V2.0"
            else:
                if try_handshake(False):
                    detected_version = "V1.1"

            if detected_version:
                self.ctrl_version = detected_version
                self._status(f"控制器握手成功：{detected_version}", 4000)
                self._set_link_state("已连接", "green")

                idx = self.comboBox.findText(detected_version)
                if idx >= 0:
                    self.comboBox.setCurrentIndex(idx)
                self.comboBox.setEnabled(False)

                # 切换至 115200
                self.ser.baudrate = 115200
                self.current_baud = 115200
                normal_rts = (detected_version == "V1.1")
                self.ser.rts = normal_rts
                self._log(f"[SER] 切换波特率至 115200, 正常模式 RTS={normal_rts}")

                self.read_timer.start(self.READ_POLL_MS)
            else:
                self._set_link_state("连接错误", "red")
                self._close_serial(clear_devices=False)
                self._status("控制器连接错误：无响应，请检查端口是否为目标接收器。", 5000)
                self._update_ui_state()

        except Exception as e:
            self._close_serial(clear_devices=False)
            QMessageBox.critical(self, "打开失败", f"{port} 打开失败：\n{e}")

    # ======================================================================
    # 控制器初始化与查询
    # ======================================================================
    def _enter_config_mode(self):
        if not self._ensure_serial(): return False
        self.ser.baudrate = 9600
        is_v2 = (self.comboBox.currentText() == "V2")
        self.ser.rts = True if is_v2 else False
        self._sleep_ms(50)
        self.ser.reset_input_buffer()
        self._log(f"[CFG_ENTER] 9600, RTS={self.ser.rts}")
        return True

    def _exit_config_mode(self):
        if self.ser and self.ser.is_open:
            self.ser.baudrate = 115200
            is_v2 = (self.comboBox.currentText() == "V2")
            self.ser.rts = False if is_v2 else True
            self._log(f"[CFG_EXIT] 115200, RTS={self.ser.rts}")
            self._sleep_ms(20)

    def _send_config_cmd_and_wait(self, cmd_hex: str, expected_hex: str = None,
                                  timeout_s: float = 0.3, expected_min_len: int = 0) -> Optional[bytes]:
        cmd_bytes = bytes.fromhex(cmd_hex)
        self.ser.write(cmd_bytes)
        self.ser.flush()
        # 显式日志，因为这是在 poll_serial 暂停期间发送的
        self._log(f"[CFG] TX: {cmd_bytes.hex(' ').upper()}")

        expect = bytes.fromhex(expected_hex) if expected_hex else None
        rx_accum = b""
        t0 = time.perf_counter()
        while (time.perf_counter() - t0) < timeout_s:
            if self.ser.in_waiting > 0:
                chunk = self.ser.read(self.ser.in_waiting)
                rx_accum += chunk
                self._log(f"[CFG] RX: {chunk.hex(' ').upper()}")
                if expect and expect in rx_accum:
                    if expected_min_len <= 0:
                        return rx_accum
                    start = rx_accum.find(expect)
                    if start >= 0 and len(rx_accum[start:]) >= expected_min_len:
                        return rx_accum
            self._sleep_ms(5)
        return rx_accum if rx_accum else None

    def _init_controller(self):
        if not self._ensure_serial(): return
        try:
            self.read_timer.stop()
            self._enter_config_mode()
            cmd = "C0 00 07 00 01 00 E7 00 17 43"
            expect = "C1 00 07 00 01 00 E7 00 17 43"
            rx = self._send_config_cmd_and_wait(cmd, expect, 0.4)
            if rx and bytes.fromhex(expect) in rx:
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

    def _query_ctrl_params(self):
        if not self._ensure_serial(): return
        try:
            self.read_timer.stop()
            self._enter_config_mode()
            cmd = "C1 00 07"
            rx = self._send_config_cmd_and_wait(cmd, "C1 00 07", 0.25, 10)
            if rx and len(rx) >= 10:
                start = rx.find(0xC1)
                if start >= 0 and len(rx[start:]) >= 10:
                    payload = rx[start + 3: start + 10]
                    # 解析...
                    mod_addr = f"0x{payload[0]:02X}{payload[1]:02X}"
                    net_id = payload[2]
                    ch = payload[5] & 0x7F
                    # ...省略部分非关键解析，确保逻辑通顺...
                    self.tx_channel = ch
                    self.ctrl_id = ((payload[0] << 8) | payload[1]) & 0x00FF
                    self.spin_CtrlCh.setValue(ch)
                    self._set_hex_widget_value(self.lbl_CtrlID_Val, self.ctrl_id)
                    self._status(f"控制器参数已更新：ID={self._format_hex8(self.ctrl_id)}, CH={ch}", 3000)
                    QMessageBox.information(self, "查询结果", f"模块地址: {mod_addr}\n信道: {ch}\n(完整信息见日志)")
                else:
                    self._status("查询参数失败：返回数据格式错误", 3000)
                    QMessageBox.warning(self, "查询结果", "数据格式错误")
            else:
                self._status("查询参数失败：查询超时", 3000)
                QMessageBox.warning(self, "查询结果", "查询超时")
        except Exception as e:
            self._log(f"[QUERY_ERR] {e}")
        finally:
            self._exit_config_mode()
            self.read_timer.start(self.READ_POLL_MS)

    def _set_ctrl_channel(self):
        if not self._ensure_serial():
            return

        new_ch = int(self.spin_CtrlCh.value()) & 0xFF
        old_ch = int(self.tx_channel) & 0xFF
        keep_id = int(self.ctrl_id) & 0xFF

        cmd = bytes([0xC0, 0x00, 0x07, (keep_id >> 8) & 0xFF, keep_id & 0xFF, 0x00, 0xE7, 0x00, new_ch, 0x43])
        expected = bytes([0xC1, 0x00, 0x07, (keep_id >> 8) & 0xFF, keep_id & 0xFF, 0x00, 0xE7, 0x00, new_ch, 0x43])

        ok = False
        rx_accum = b""

        try:
            self.read_timer.stop()

            if not self._enter_config_mode():
                return

            self.ser.write(cmd)
            self.ser.flush()
            self._log(f"[SET_CH] TX: {cmd.hex(' ').upper()}")

            t0 = time.perf_counter()
            while (time.perf_counter() - t0) < 0.1:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    rx_accum += chunk
                    if expected in rx_accum:
                        ok = True
                        break
                self._sleep_ms(2)
        except Exception as e:
            self._log(f"[SET_CH_ERR] {e}")
            ok = False
        finally:
            try:
                self._exit_config_mode()
            except Exception:
                pass
            self.read_timer.start(self.READ_POLL_MS)

        if ok:
            self.tx_channel = new_ch
            self._log(f"[SET_CH] ACK: {expected.hex(' ').upper()}")
            self._status(f"设置信道成功：CH={new_ch}", 3000)
            QMessageBox.information(self, "设置信道", f"信道设置成功：CH={new_ch}")
            return

        self.spin_CtrlCh.setValue(old_ch)
        self._status("设置信道失败：未收到确认帧", 3500)
        QMessageBox.warning(
            self,
            "设置信道失败",
            f"未在100ms内收到确认帧。\n期望: {expected.hex(' ').upper()}\n接收: {rx_accum.hex(' ').upper() if rx_accum else '(空)'}"
        )

    def _set_ctrl_id(self):
        if not self._ensure_serial():
            return

        new_id = self._get_hex_widget_value(getattr(self, "lbl_CtrlID_Val", None), 0x0001) & 0xFF
        if (new_id < 0x0001) or (new_id > 0x00FF):
            QMessageBox.warning(self, "输入错误", "控制器ID范围应为 0x0001 ~ 0x00FF。")
            self._set_hex_widget_value(getattr(self, "lbl_CtrlID_Val", None), 0x0001)
            return

        old_id = int(self.ctrl_id) & 0xFFFF
        ch = int(self.spin_CtrlCh.value()) & 0xFF

        cmd = bytes([0xC0, 0x00, 0x07, (new_id >> 8) & 0xFF, new_id & 0xFF, 0x00, 0xE7, 0x00, ch, 0x43])
        expected = bytes([0xC1, 0x00, 0x07, (new_id >> 8) & 0xFF, new_id & 0xFF, 0x00, 0xE7, 0x00, ch, 0x43])

        ok = False
        rx_accum = b""
        try:
            self.read_timer.stop()

            if not self._enter_config_mode():
                return

            self.ser.write(cmd)
            self.ser.flush()
            self._log(f"[SET_ID] TX: {cmd.hex(' ').upper()}")

            t0 = time.perf_counter()
            while (time.perf_counter() - t0) < 0.1:
                if self.ser.in_waiting > 0:
                    chunk = self.ser.read(self.ser.in_waiting)
                    rx_accum += chunk
                    if expected in rx_accum:
                        ok = True
                        break
                self._sleep_ms(2)
        except Exception as e:
            self._log(f"[SET_ID_ERR] {e}")
            ok = False
        finally:
            try:
                self._exit_config_mode()
            except Exception:
                pass
            self.read_timer.start(self.READ_POLL_MS)

        if ok:
            self.ctrl_id = new_id & 0xFF
            self._log(f"[SET_ID] ACK: {expected.hex(' ').upper()}")
            self._status(f"控制器ID已设置为 {self._format_hex8(new_id)}", 2500)
            QMessageBox.information(self, "设置ID", f"控制器ID设置成功：{self._format_hex8(new_id)}")
            return

        self._set_hex_widget_value(getattr(self, "lbl_CtrlID_Val", None), old_id)
        self._status("设置ID失败：未收到确认帧", 3500)
        QMessageBox.warning(
            self,
            "设置ID失败",
            f"未在100ms内收到确认帧。\n期望: {expected.hex(' ').upper()}\n接收: {rx_accum.hex(' ').upper() if rx_accum else '(空)'}"
        )

    # ======================================================================
    # 串口收包
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
            if self._raw_rx_debug:
                shown = raw[:80]
                suffix = " ..." if len(raw) > 80 else ""
                self._log("RX(%d): %s%s" % (len(raw), " ".join(f"{b:02X}" for b in shown), suffix))

            self._rx_buffer += raw
            for pkt in self._extract_v21_frames():
                self._handle_frame(pkt)
        except Exception as e:
            self._status(f"串口读取错误：{e}", 5000)

    def _handle_frame(self, pkt: bytes):
        rx_hex = " ".join(f"{b:02X}" for b in pkt)
        self._log(f"RX: {rx_hex}")

        parsed = self._parse_v21_frame(pkt)
        if not parsed:
            self._log(f"[FRAME] 解析失败 | raw={pkt.hex(' ').upper()}")
            return

        fish_id = int(parsed.get("sender_id", 0))
        cmd = int(parsed.get("cmd", 0))
        payload: bytes = parsed.get("payload", b"")

        if cmd == 0xFF:
            self._handle_cmd_reply(fish_id, payload)
        elif cmd == 0xF1:
            self._handle_status_reply_f1(fish_id, payload)
        elif cmd == 0xF2:
            self._handle_servo_reply_f2(fish_id, payload)
        elif cmd == 0xF3:
            self._handle_boot_reply_f3(fish_id, payload)
        elif cmd == 0xF4:
            self._handle_flash_reply_f4(fish_id, payload)
        elif cmd == 0xF5:
            self._handle_version_reply_f5(fish_id, payload)
        elif cmd == 0xF6:
            self._handle_imu_reply_f6(fish_id, payload)
        elif cmd == 0xF7:
            self._handle_attitude_reply_f7(fish_id, payload)
        elif cmd == 0xF8:
            self._handle_depth_reply_f8(fish_id, payload)
        else:
            self._log(f"[FRAME] CMD=0x{cmd:02X} 来自 0x{fish_id:04X} payload={payload.hex(' ').upper()}")

    def _handle_cmd_reply(self, fish_id: int, payload: bytes):
        if len(payload) < 2:
            self._log(f"[FF] 长度不足: {payload.hex(' ').upper()}")
            return
        src_cmd = payload[0]
        status = payload[1]
        ok = (status == 0)
        unpair_ok = (src_cmd == 0x91) and (status in (0, 3))
        msg = "成功" if ok else f"失败(错误码={status})"
        self._log(f"[ACK] 0x{fish_id:04X} CMD=0x{src_cmd:02X} {msg}")

        if src_cmd == 0xBA and ok:
            self._ensure_device_exists(fish_id)
            self._log(f"[DEV] 发现设备 0x{fish_id:04X}")
        elif src_cmd == 0x90 and ok:
            self._ensure_device_exists(fish_id)
            st = self.dev_state.get(fish_id)
            if st:
                st.paired = True
            if self.pending_ctrl_code is not None:
                self.control_password = int(self.pending_ctrl_code) & 0xFFFF
            self._update_device_tables_row(fish_id)
            self._ensure_monitor_row(fish_id)
            self._send_v21_cmd(fish_id, 0xC1, b"", tag="QUERY_STAT")
            self._log(f"[PAIR] 0x{fish_id:04X} 配对成功，已加入系统监视")
            self._update_ui_state()
        elif unpair_ok:
            st = self.dev_state.get(fish_id)
            if st:
                st.paired = False
            self._update_device_tables_row(fish_id)
            self._remove_monitor_row(fish_id)
            if status == 3:
                self._log(f"[UNPAIR] 0x{fish_id:04X} 返回密码错误(3)，可能系统已重启，已移出系统监视")
            else:
                self._log(f"[UNPAIR] 0x{fish_id:04X} 已取消配对，已移出系统监视")
            self._update_ui_state()
        elif src_cmd == 0xD6:
            old_id = fish_id
            new_id = self.pending_fish_id_change.pop(old_id, None)
            if new_id is None:
                for oid, nid in list(self.pending_fish_id_change.items()):
                    if nid == fish_id:
                        old_id, new_id = oid, nid
                        self.pending_fish_id_change.pop(oid, None)
                        break

            if ok and (new_id is not None):
                self._sync_fish_id_change(old_id, new_id)
            elif (not ok) and (new_id is not None):
                self._log(f"[SET_ID] 0x{old_id:04X} -> 0x{new_id:04X} 失败，保持原ID不变")

    def _handle_status_reply_f1(self, fish_id: int, payload: bytes):
        if len(payload) < 7:
            self._log(f"[F1] 长度不足: {payload.hex(' ').upper()}")
            return
        self._ensure_device_exists(fish_id)
        st = self.dev_state[fish_id]
        st.battery_pct = int(payload[0])
        st.voltage_v = int.from_bytes(payload[1:3], "big") / 100.0
        st.power_mw = int.from_bytes(payload[3:5], "big")
        st.est_minutes = int(payload[5]) * 10
        run_mode = int(payload[6])
        st.running = self._run_mode_text(run_mode)
        self._update_device_tables_row(fish_id)
        self._update_monitor_tables_row(fish_id)

    def _handle_servo_reply_f2(self, fish_id: int, payload: bytes):
        if len(payload) < 6:
            self._log(f"[F2] 长度不足: {payload.hex(' ').upper()}")
            return

        servo_out = payload[0] # 0关闭，1开启
        curr1 = int.from_bytes(payload[1:3], "big") # mA
        curr2 = int.from_bytes(payload[3:5], "big")
        servo_bad = payload[5] # 0正常，1异常

        self.bar_S1_Curr.setValue(curr1)
        self.bar_S2_Curr.setValue(curr2)
        self.chk_S1_Pwr.setChecked(bool(servo_out & 0x01))
        self.chk_S2_Pwr.setChecked(bool(servo_out & 0x02))
        self.lbl_S1_FaultFlag_Val.setText("异常" if (servo_bad & 0x01) else "运行正常")
        self.lbl_S2_FaultFlag_Val.setText("异常" if (servo_bad & 0x02) else "运行正常")

    def _handle_boot_reply_f3(self, fish_id: int, payload: bytes):
        if len(payload) < 7:
            self._log(f"[F3] 长度不足: {payload.hex(' ').upper()}")
            return

        boot_mode = int(payload[0])
        reply_en = int(payload[1])
        bias1 = int.from_bytes(payload[2:4], "big", signed=True) / 10.0
        bias2 = int.from_bytes(payload[4:6], "big", signed=True) / 10.0
        oc_time = int(payload[6])

        mode_idx_map = {0: 0, 1: 1, 2: 2, 4: 3, 5: 4, 7: 5}
        self.combo_F_Mode.setCurrentIndex(mode_idx_map.get(boot_mode, 0))
        self.chk_F_Reply.setChecked(reply_en == 0)
        self.spin_F_ServoBiasS1.setValue(bias1)
        self.spin_F_ServoBiasS2.setValue(bias2)
        self.spin_F_OverCurrentTimeout.setValue(oc_time)

        if len(payload) >= 9:
            proto_ver = payload[7]
            fw_ver = payload[8]
            self.lbl_BProtocolVer_Val.setText(f"V{proto_ver >> 4}.{proto_ver & 0x0F}")
            self.lbl_BFirmwareVer_Val.setText(f"V{fw_ver >> 4}.{fw_ver & 0x0F}")
            self._log(
                f"[F3] 0x{fish_id:04X} BootMode={boot_mode}, Reply={'开' if reply_en == 0 else '关'}, "
                f"协议V{proto_ver >> 4}.{proto_ver & 0x0F}, 固件V{fw_ver >> 4}.{fw_ver & 0x0F}"
            )
        else:
            self._log(f"[F3] 0x{fish_id:04X} BootMode={boot_mode}, Reply={'开' if reply_en == 0 else '关'}")

    def _handle_flash_reply_f4(self, fish_id: int, payload: bytes):
        if len(payload) < 3:
            self._log(f"[F4] 长度不足: {payload.hex(' ').upper()}")
            return

        # 新协议: offset(1B) + data(2B)
        # 兼容旧协议: addr(2B) + data(2B)
        if len(payload) >= 4:
            addr = int.from_bytes(payload[0:2], "big")
            data = int.from_bytes(payload[2:4], "big")
        else:
            addr = int(payload[0]) & 0xFF
            data = int.from_bytes(payload[1:3], "big")
        self._log(f"[F4] 0x{fish_id:04X} Flash[0x{addr:04X}] = 0x{data:04X}")

    def _handle_version_reply_f5(self, fish_id: int, payload: bytes):
        if len(payload) < 2:
            self._log(f"[F5] 长度不足: {payload.hex(' ').upper()}")
            return
        proto_ver = payload[0]
        fw_ver = payload[1]
        self.lbl_BProtocolVer_Val.setText(f"V{proto_ver >> 4}.{proto_ver & 0x0F}")
        self.lbl_BFirmwareVer_Val.setText(f"V{fw_ver >> 4}.{fw_ver & 0x0F}")
        self._log(f"[F5] 0x{fish_id:04X} 协议V{proto_ver >> 4}.{proto_ver & 0x0F}, 固件V{fw_ver >> 4}.{fw_ver & 0x0F}")

    def _handle_imu_reply_f6(self, fish_id: int, payload: bytes):
        if len(payload) < 12:
            self._log(f"[F6] 长度不足: {payload.hex(' ').upper()}")
            return

        vals = [int.from_bytes(payload[i:i + 2], "big", signed=True) for i in range(0, 12, 2)]
        ax, ay, az, gx, gy, gz = vals

        self.lbl_IMU_AX_Val.setText(f"{ax / 1000.0:.2f} g")
        self.lbl_IMU_AY_Val.setText(f"{ay / 1000.0:.2f} g")
        self.lbl_IMU_AZ_Val.setText(f"{az / 1000.0:.2f} g")
        self.lbl_IMU_GX_Val.setText(f"{gx / 10.0:.2f} °/s")
        self.lbl_IMU_GY_Val.setText(f"{gy / 10.0:.2f} °/s")
        self.lbl_IMU_GZ_Val.setText(f"{gz / 10.0:.2f} °/s")
        self.lbl_Gyro_Status_Val.setText("正常")
        self._log(f"[F6] 0x{fish_id:04X} IMU已更新")

    def _handle_attitude_reply_f7(self, fish_id: int, payload: bytes):
        if len(payload) < 6:
            self._log(f"[F7] 长度不足: {payload.hex(' ').upper()}")
            return

        pitch = int.from_bytes(payload[0:2], "big", signed=True) / 10.0
        roll = int.from_bytes(payload[2:4], "big", signed=True) / 10.0
        yaw = int.from_bytes(payload[4:6], "big", signed=True) / 10.0

        self.lbl_Attitude_Pitch_Val.setText(f"{pitch:.1f} °")
        self.lbl_Attitude_Roll_Val.setText(f"{roll:.1f} °")
        self.lbl_Attitude_Yaw_Val.setText(f"{yaw:.1f} °")
        self.lbl_YawFiltered_Val.setText(f"{yaw:.1f}°")
        self._log(f"[F7] 0x{fish_id:04X} 姿态已更新")

    def _handle_depth_reply_f8(self, fish_id: int, payload: bytes):
        if len(payload) < 2:
            self._log(f"[F8] 长度不足: {payload.hex(' ').upper()}")
            return

        depth_m = int.from_bytes(payload[0:2], "big") / 10.0
        self.lbl_DepthInfo_Val.setText(f"{depth_m:.2f} m")
        self.lbl_DepthSensor_Status_Val.setText("正常")
        self._log(f"[F8] 0x{fish_id:04X} 深度={depth_m:.2f}m")

    # ======================================================================
    # 发送
    # ======================================================================
    def _send_bytes(self, data: bytes, tag: str = "") -> bool:
        if not self._ensure_serial(): return False
        try:
            self.ser.write(data)
            self.ser.flush()
            hx = " ".join(f"{b:02X}" for b in data)
            # 记录所有发送的原始数据
            self._log(f"TX{f'[{tag}]' if tag else ''}: {hx}")
            return True
        except Exception as e:
            self._status(f"发送失败：{e}", 5000)
            return False

    # ======================================================================
    # 设备与表格管理
    # ======================================================================
    def _ensure_device_exists(self, fish_id: int):
        if fish_id not in self.devices:
            self.devices[fish_id] = LoraProtocol(fish_id, self.control_password, self.tx_channel)
            self.dev_state[fish_id] = DeviceState(fish_id=fish_id)
            self._append_device_to_table(fish_id)

    def _ensure_monitor_row(self, fish_id: int):
        if self._find_row_by_fish_id(self.table_SysMonitor, 1, fish_id) is None:
            self._append_monitor_row(fish_id)

    def _remove_monitor_row(self, fish_id: int):
        row = self._find_row_by_fish_id(self.table_SysMonitor, 1, fish_id)
        if row is not None:
            self.table_SysMonitor.removeRow(row)

    def _clear_all_devices(self):
        self.devices.clear()
        self.dev_state.clear()
        self.table_Devices.setRowCount(0)
        self.table_SysMonitor.setRowCount(0)

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
        for c in range(2, 5):
            item = QtWidgets.QTableWidgetItem("-")
            item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
            self.table_SysMonitor.setItem(row, c, item)

    def _find_row_by_fish_id(self, table: QtWidgets.QTableWidget, col: int, fish_id: int) -> Optional[int]:
        t = f"0x{fish_id:04X}".upper()
        for r in range(table.rowCount()):
            it = table.item(r, col)
            if it and (it.text() or "").strip().upper() == t: return r
        return None

    def _replace_fish_id_text_in_table(self, table: QtWidgets.QTableWidget, col: int, old_id: int, new_id: int):
        row = self._find_row_by_fish_id(table, col, old_id)
        if row is None:
            return
        it = table.item(row, col)
        if it is not None:
            it.setText(f"0x{new_id:04X}")

    def _remove_duplicate_fish_rows(self, table: QtWidgets.QTableWidget, col: int):
        seen = set()
        for r in range(table.rowCount() - 1, -1, -1):
            it = table.item(r, col)
            key = (it.text() or "").strip().upper() if it else ""
            if not key:
                continue
            if key in seen:
                table.removeRow(r)
            else:
                seen.add(key)

    def _sync_fish_id_change(self, old_id: int, new_id: int):
        if old_id == new_id:
            self._log(f"[SET_ID] 新ID与旧ID相同：0x{new_id:04X}")
            return

        old_dev = self.devices.pop(old_id, None)
        old_st = self.dev_state.pop(old_id, None)

        self.devices.pop(new_id, None)
        self.dev_state.pop(new_id, None)

        if old_dev is None:
            old_dev = LoraProtocol(new_id, self.control_password, self.tx_channel)
        old_dev.fish_id = new_id
        self.devices[new_id] = old_dev

        if old_st is None:
            old_st = DeviceState(fish_id=new_id)
        old_st.fish_id = new_id
        self.dev_state[new_id] = old_st

        self._replace_fish_id_text_in_table(self.table_Devices, 0, old_id, new_id)
        self._replace_fish_id_text_in_table(self.table_SysMonitor, 1, old_id, new_id)
        self._remove_duplicate_fish_rows(self.table_Devices, 0)
        self._remove_duplicate_fish_rows(self.table_SysMonitor, 1)

        self._update_device_tables_row(new_id)
        self._update_monitor_tables_row(new_id)
        self._update_ui_state()
        self._status(f"FishID 已同步更新：0x{old_id:04X} -> 0x{new_id:04X}", 3000)
        self._log(f"[SET_ID] 已同步本地设备ID: 0x{old_id:04X} -> 0x{new_id:04X}")

    def _update_device_tables_row(self, fish_id: int):
        st = self.dev_state.get(fish_id)
        if not st: return
        r = self._find_row_by_fish_id(self.table_Devices, 0, fish_id)
        if r is not None:
            it = self.table_Devices.item(r, 1)
            if st.paired:
                it.setText("已配对")
                it.setForeground(QtGui.QBrush(QtGui.QColor(0, 170, 0)))
            else:
                it.setText("未配对")
                it.setForeground(QtGui.QBrush(QtGui.QColor("red")))

    def _update_monitor_tables_row(self, fish_id: int):
        st = self.dev_state.get(fish_id)
        if not st: return
        r = self._find_row_by_fish_id(self.table_SysMonitor, 1, fish_id)
        if r is not None:
            vp_text = "-"
            bt_text = "-"
            run_text = st.running or "-"

            if st.voltage_v is not None or st.power_mw is not None:
                v_str = "-" if st.voltage_v is None else f"{st.voltage_v:.2f}V"
                p_str = "-" if st.power_mw is None else f"{st.power_mw}mW"
                vp_text = f"{v_str} / {p_str}"

            if st.battery_pct is not None or st.est_minutes is not None:
                b_str = "-" if st.battery_pct is None else f"{st.battery_pct}%"
                if st.est_minutes is None:
                    t_str = "-"
                else:
                    h = st.est_minutes // 60
                    m = st.est_minutes % 60
                    t_str = f"{h}h{m:02d}m" if h > 0 else f"{m}m"
                bt_text = f"{b_str} / {t_str}"

            self.table_SysMonitor.item(r, 2).setText(vp_text)
            self.table_SysMonitor.item(r, 3).setText(bt_text)
            self.table_SysMonitor.item(r, 4).setText(run_text)

    def _selected_target_ids(self) -> List[int]:
        ids: List[int] = []
        ids_set = set()
        for r in range(self.table_SysMonitor.rowCount()):
            try:
                it_sel = self.table_SysMonitor.item(r, 0)
                it_id = self.table_SysMonitor.item(r, 1)
                if (it_sel is None) or (it_id is None):
                    continue
                if it_sel.checkState() == Qt.CheckState.Checked:
                    fish_id = int((it_id.text() or "").strip(), 16)
                    if fish_id not in ids_set:
                        ids.append(fish_id)
                        ids_set.add(fish_id)
            except Exception:
                pass

        sm = self.table_SysMonitor.selectionModel()
        if sm:
            for idx in sm.selectedRows(1):
                try:
                    fish_id = int((idx.data() or "").strip(), 16)
                    if fish_id not in ids_set:
                        ids.append(fish_id)
                        ids_set.add(fish_id)
                except Exception:
                    pass
        return ids

    def _selected_device_ids_in_devices_table(self) -> List[int]:
        ids: List[int] = []
        sm = self.table_Devices.selectionModel()
        if not sm:
            return ids
        for idx in sm.selectedRows(0):
            try:
                fid = int((idx.data() or "").strip(), 16)
                ids.append(fid)
            except Exception:
                pass
        return ids

    def _target_fishes(self) -> List[LoraProtocol]:
        fishes = []
        for fid in self._selected_target_ids():
            dev = self.devices.get(fid)
            if not dev:
                dev = LoraProtocol(fid, self.control_password, self.tx_channel)
                self.devices[fid] = dev
                self.dev_state[fid] = DeviceState(fish_id=fid)
            dev.channel = self.tx_channel
            fishes.append(dev)
        return fishes

    def _select_all_targets(self):
        for r in range(self.table_SysMonitor.rowCount()):
            self.table_SysMonitor.item(r, 0).setCheckState(Qt.CheckState.Checked)
        self._update_ui_state()

    def _select_none_targets(self):
        for r in range(self.table_SysMonitor.rowCount()):
            self.table_SysMonitor.item(r, 0).setCheckState(Qt.CheckState.Unchecked)
        self._update_ui_state()

    # ======================================================================
    # 业务功能
    # ======================================================================
    def _search_devices(self):
        if not self._ensure_serial(): return
        try:
            if self._send_v21_cmd(0xFFFF, 0xBA, b"", tag="SearchDevices"):
                self._status("已广播搜索设备。", 2000)
        except Exception as e:
            self._status(f"搜索失败：{e}", 5000)

    def _manual_add_device_dialog(self):
        txt, ok = QInputDialog.getText(self, "手动添加", "FishID (16进制):")
        if ok and txt:
            try:
                fid = int(txt, 16)
                self._ensure_device_exists(fid)
                self._status(f"已添加 0x{fid:04X}", 2000)
                self._update_ui_state()
            except:
                pass

    def _apply_fish_id(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if len(target_ids) != 1:
            QMessageBox.information(self, "提示", "修改 FishID 时请仅勾选 1 台设备。")
            return

        new_id = self._get_hex_widget_value(getattr(self, "txt_F_ID", None), 0x0001)
        old_id = target_ids[0]
        if new_id == old_id:
            self._status("新 FishID 与当前一致，无需修改。", 2500)
            return
        payload = int(new_id & 0xFFFF).to_bytes(2, "big")
        if self._send_v21_cmd(old_id, 0xD6, payload, tag=f"SET_ID 0x{new_id:04X}"):
            self.pending_fish_id_change[old_id] = new_id
            self._status(f"已发送 FishID 修改指令：0x{old_id:04X} -> 0x{new_id:04X}", 3000)

    def _apply_fish_channel(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids:
            self._status("请先勾选目标设备。", 2500)
            return

        new_ch = int(self.spin_F_Ch.value())
        payload = bytes([new_ch & 0xFF])
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xD8, payload, tag=f"SET_CH 0x{fish_id:04X}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

        self.tx_channel = new_ch
        self.spin_CtrlCh.setValue(new_ch)
        self._status(f"已发送信道修改指令：CH={new_ch}", 3000)

    def _apply_fish_password(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids:
            self._status("请先勾选目标设备。", 2500)
            return

        new_pwd = self._get_hex_widget_value(getattr(self, "txt_F_Pwd", None), 0x0000)
        payload = int(new_pwd & 0xFFFF).to_bytes(2, "big")
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xD7, payload, tag=f"SET_PWD 0x{fish_id:04X}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

        self._status(
            f"已发送新密码写入指令：{self._format_hex16(new_pwd)}（仅影响下次配对，当前控制口令不变）",
            3500
        )

    def _pair_selected_devices(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_device_ids_in_devices_table()
        if not target_ids:
            self._status("未选择设备", 2000)
            return

        default_pwd = self._format_hex16(self._get_hex_widget_value(getattr(self, "txt_F_Pwd", None), 0x0000))
        pwd_text, ok = QInputDialog.getText(self, "配对", "请输入配对密码(HEX，如0x0000)：", text=default_pwd)
        if not ok:
            return
        try:
            pwd = int((pwd_text or "").strip(), 16) & 0xFFFF
        except Exception:
            QMessageBox.warning(self, "输入错误", "密码格式错误，请使用 0x0000 这种HEX格式。")
            return

        # 记录“配对密码”作为下次默认值，不影响当前“控制口令”
        self._set_hex_widget_value(getattr(self, "txt_F_Pwd", None), pwd)

        ctrl_code = self._second_password_value()
        if ctrl_code is None:
            QMessageBox.warning(self, "输入错误", "控制口令格式错误，请输入4位16进制（如 A1B2）。")
            return

        payload = int(pwd & 0xFFFF).to_bytes(2, "big") + int(ctrl_code & 0xFFFF).to_bytes(2, "big")
        self.pending_ctrl_code = ctrl_code
        send_count = 0
        for fish_id in target_ids:
            st = self.dev_state.get(fish_id)
            if st and st.paired:
                continue
            self._send_v21_cmd(fish_id, 0x90, payload, tag=f"PAIR 0x{fish_id:04X}", password=pwd)
            send_count += 1
        if send_count == 0:
            self.pending_ctrl_code = None
        self._status(f"已发送配对请求 ({send_count} 台)", 3000)

    def _unpair_selected_devices(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_device_ids_in_devices_table()
        send_count = 0
        timeout_success_count = 0
        response_success_count = 0
        for fish_id in target_ids:
            st = self.dev_state.get(fish_id)
            if not st or (not st.paired):
                continue
            self._send_v21_cmd(fish_id, 0x91, b"", tag=f"UNPAIR 0x{fish_id:04X}")
            send_count += 1

            self._process_events_ms(self.UNPAIR_ACK_TIMEOUT_MS)

            st_after = self.dev_state.get(fish_id)
            if st_after and (not st_after.paired):
                response_success_count += 1
                continue

            if st_after:
                st_after.paired = False
            self._update_device_tables_row(fish_id)
            self._remove_monitor_row(fish_id)
            timeout_success_count += 1
            self._log(
                f"[UNPAIR] 0x{fish_id:04X} 在 {self.UNPAIR_ACK_TIMEOUT_MS}ms 内未收到应答，按离线取消配对成功处理"
            )

        self._update_ui_state()
        if send_count == 0:
            self._status("未找到可取消配对的设备", 2500)
        else:
            self._status(
                f"取消配对完成：应答成功{response_success_count}台，超时判成功{timeout_success_count}台",
                3500
            )

    def _global_stop_selected(self):
        if not self._ensure_serial(): return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xA0, bytes([0]), tag=f"STOP 0x{fish_id:04X}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _global_stop_all(self):
        if not self._ensure_serial(): return
        for fish_id in list(self.devices.keys()):
            self._send_v21_cmd(fish_id, 0xA0, bytes([0]), tag=f"STOP_ALL 0x{fish_id:04X}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _toggle_gear_cont(self, on):
        if on:
            self.timer_gear.start(max(20, int(self.spin_Sync_Gear_Ms.value())))
        else:
            self.timer_gear.stop()

    def _send_gear(self, silent=False):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids: return
        spd = self.spin_GearSpeed.value()
        turn = self.spin_GearTurn.value()
        payload = bytes([spd & 0xFF, turn & 0xFF, 0])
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA5, payload, tag=f"GEAR 0x{fish_id:04X}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _servo_slider_to_spin(self, val, spin):
        spin.blockSignals(True)
        spin.setValue(val / 10.0)
        spin.blockSignals(False)

    def _servo_spin_to_slider(self, val, slider):
        slider.blockSignals(True)
        slider.setValue(int(round(val * 10)))
        slider.blockSignals(False)

    def _toggle_servo_cont(self, on):
        if on:
            self.timer_servo.start(max(20, int(self.spin_Sync_Servo_Ms.value())))
        else:
            self.timer_servo.stop()

    def _send_servo_position(self, silent=False):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids: return
        ang1, ang2 = self.spin_S1.value(), self.spin_S2.value()

        def _to_proto_01deg(v: float) -> int:
            vv = int(round((90.0 + v) * 10.0))
            return max(0, min(1800, vv))

        s1 = _to_proto_01deg(ang1)
        s2 = _to_proto_01deg(ang2)

        if self.radio_S1_Only.isChecked():
            payload = s1.to_bytes(2, "big") + (0xFFFF).to_bytes(2, "big")
        elif self.radio_S2_Only.isChecked():
            payload = (0xFFFF).to_bytes(2, "big") + s2.to_bytes(2, "big")
        else:
            payload = s1.to_bytes(2, "big") + s2.to_bytes(2, "big")

        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA3, payload, tag=f"SERVO 0x{fish_id:04X}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _set_servo_power(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        s1, s2 = self.chk_S1_Pwr.isChecked(), self.chk_S2_Pwr.isChecked()
        flag = (1 if s1 else 0) | (2 if s2 else 0)
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xB5, bytes([flag]), tag="SERVO_PWR")

    def _query_servo_all_status(self):
        if not self._ensure_serial(): return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xC2, b"", tag="QUERY_SERVO")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _toggle_cpg_cont(self, on):
        if on:
            self.timer_cpg.start(max(20, int(self.spin_Sync_CPG_Ms.value())))
        else:
            self.timer_cpg.stop()

    def _send_cpg(self, silent=False):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids: return
        amp, freq, bias = self.spin_Amp.value(), self.spin_Freq.value(), self.spin_Bias.value()

        freq_u8 = max(0, min(255, int(round(freq * 10.0))))
        amp_u16 = max(0, min(65535, int(round(abs(amp) * 10.0))))
        bias_i16 = max(-32768, min(32767, int(round(bias * 10.0))))
        interval = 1 if self.grp_CPG_Inter.isChecked() else 0
        payload = bytes([freq_u8]) + amp_u16.to_bytes(2, "big") + int(bias_i16).to_bytes(2, "big", signed=True) + bytes([interval, 0])

        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA1, payload, tag=f"CPG 0x{fish_id:04X}")
            if self.grp_CPG_Inter.isChecked():
                n_burst = int(self.spin_N.value()) & 0xFF
                coast_unit = max(0, min(65535, int(round(self.spin_Coast.value() * 10.0))))
                payload_int = bytes([n_burst]) + coast_unit.to_bytes(2, "big") + bytes([0])
                self._send_v21_cmd(fish_id, 0xA2, payload_int, tag=f"INTV 0x{fish_id:04X}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _start_play_mode(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        pid = self.spin_PlayId.value()
        payload = bytes([pid & 0xFF, 0])
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA4, payload, tag=f"PLAY {pid}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _stop_play_mode(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        payload = bytes([0, 0])
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA4, payload, tag="PLAY_STOP")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _flash_read_config(self):
        if not self._ensure_serial(): return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xC3, b"", tag="QUERY_BOOT")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _flash_save_config(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()

        mode_map = {0: 0, 1: 1, 2: 2, 3: 4, 4: 5, 5: 7}
        mode = mode_map.get(self.combo_F_Mode.currentIndex(), 0)
        reply_disable = 0 if self.chk_F_Reply.isChecked() else 1
        bias1 = int(round(self.spin_F_ServoBiasS1.value() * 10.0))
        bias2 = int(round(self.spin_F_ServoBiasS2.value() * 10.0))
        oc_time = int(self.spin_F_OverCurrentTimeout.value()) & 0xFF

        payload = (
            bytes([mode, reply_disable])
            + int(bias1).to_bytes(2, "big", signed=True)
            + int(bias2).to_bytes(2, "big", signed=True)
            + bytes([oc_time])
        )

        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xD1, payload, tag="BOOT_CFG")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

        self._status("已发送开机参数保存指令", 3000)

    def _set_auto_report(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        enable = self.radio_Rpt_On.isChecked()
        period_s = max(1, int(round(self.spin_Rpt_Ms.value() / 1000.0)))

        report_cmds: List[int] = []
        if self.chk_Rpt_Motion.isChecked():
            report_cmds.append(0xC1)
        if self.chk_Rpt_Servo.isChecked():
            report_cmds.append(0xC2)
        if not report_cmds:
            report_cmds = [0xC1]

        for fish_id in target_ids:
            if not enable:
                self._send_v21_cmd(fish_id, 0xB6, bytes([0, report_cmds[0], period_s]), tag="AUTO_RPT_OFF")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)
                continue
            for rc in report_cmds:
                self._send_v21_cmd(fish_id, 0xB6, bytes([1, rc, period_s]), tag=f"AUTO_RPT 0x{rc:02X}")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)

        self._status("自动回传设置已下发", 2500)

    def _set_install_bias(self):
        if not self._ensure_serial(): return
        mode_map = {0: 0, 1: 1, 2: 2, 3: 4, 4: 5, 5: 7}
        mode = mode_map.get(self.combo_F_Mode.currentIndex(), 0)
        reply_disable = 0 if self.chk_F_Reply.isChecked() else 1
        bias1 = int(round(self.spin_F_ServoBiasS1.value() * 10.0))
        bias2 = int(round(self.spin_F_ServoBiasS2.value() * 10.0))
        oc_time = int(self.spin_F_OverCurrentTimeout.value()) & 0xFF
        payload = (
            bytes([mode, reply_disable])
            + int(bias1).to_bytes(2, "big", signed=True)
            + int(bias2).to_bytes(2, "big", signed=True)
            + bytes([oc_time])
        )
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xD1, payload, tag="SET_BIAS_D1")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _query_flash(self):
        if not self._ensure_serial(): return
        offset, ok = QInputDialog.getInt(self, "查询 Flash", "Offset (0x00-0x1C, 仅偶数):", 0, 0, 0x1C, 2)
        if not ok:
            return
        if (offset < 0) or (offset > 0x1C) or ((offset & 0x01) != 0):
            QMessageBox.warning(self, "输入错误", "Offset 必须是 0x00~0x1C 范围内的偶数。")
            return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xD4, bytes([offset & 0xFF]), tag=f"FLASH_QUERY_OFFSET {offset}")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _reset_faulty_servo(self):
        if not self._ensure_serial(): return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xD9, b"", tag="RESET_SERVO_BAD")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _factory_reset(self):
        if QMessageBox.question(self, "确认", "确定恢复出厂设置？") == QMessageBox.StandardButton.Yes:
            for fish_id in self._selected_target_ids():
                self._send_v21_cmd(fish_id, 0xD0, b"", tag="FACTORY_RESET")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _reply_switch_dialog(self):
        if not self._ensure_serial(): return
        choice, ok = QInputDialog.getItem(self, "设置", "选项:", ["开启应答", "关闭应答"], 0, False)
        if ok:
            enable = (choice == "开启应答")
            self.last_reply_enable = enable
            timeout_s = int(self.spin_F_OverCurrentTimeout.value()) if hasattr(self, "spin_F_OverCurrentTimeout") else 10
            timeout_s = max(1, min(255, timeout_s))
            payload = timeout_s.to_bytes(2, "big") + bytes([0 if enable else 1])
            for fish_id in self._selected_target_ids():
                self._send_v21_cmd(fish_id, 0xB0, payload, tag="REPLY_CFG")
                self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _query_volt_power(self):
        if not self._ensure_serial(): return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xC1, b"", tag="QUERY_STAT")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _query_servo_status_single(self):
        self._query_servo_all_status()

    def _set_boot_from_gear(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids:
            self._status("请先勾选目标设备。", 2000)
            return
        spd = int(self.spin_GearSpeed.value()) & 0xFF
        turn = int(self.spin_GearTurn.value()) & 0xFF
        payload = bytes([spd, turn, 1])
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA5, payload, tag="BOOT_FROM_GEAR")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _set_boot_from_cpg(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids:
            self._status("请先勾选目标设备。", 2000)
            return

        freq_u8 = max(0, min(255, int(round(self.spin_Freq.value() * 10.0))))
        amp_u16 = max(0, min(65535, int(round(abs(self.spin_Amp.value()) * 10.0))))
        bias_i16 = max(-32768, min(32767, int(round(self.spin_Bias.value() * 10.0))))
        interval = 1 if self.grp_CPG_Inter.isChecked() else 0
        payload = bytes([freq_u8]) + amp_u16.to_bytes(2, "big") + int(bias_i16).to_bytes(2, "big", signed=True) + bytes([interval, 1])

        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA1, payload, tag="BOOT_FROM_CPG")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _set_boot_from_play(self):
        if not self._ensure_serial(): return
        target_ids = self._selected_target_ids()
        if not target_ids:
            self._status("请先勾选目标设备。", 2000)
            return
        pid = int(self.spin_PlayId.value()) & 0xFF
        payload = bytes([pid, 1])
        for fish_id in target_ids:
            self._send_v21_cmd(fish_id, 0xA4, payload, tag="BOOT_FROM_PLAY")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _servo_quick_reset(self):
        self.spin_S1.setValue(0.0)
        self.spin_S2.setValue(0.0)
        self.radio_Dual_Sync.setChecked(True)
        self._send_servo_position(silent=False)

    def _save_log_file(self):
        path, _ = QFileDialog.getSaveFileName(self, "保存日志", "comm_log.txt", "Text Files (*.txt);;All Files (*)")
        if not path:
            return
        try:
            with open(path, "w", encoding="utf-8") as f:
                f.write(self.txt_Log.toPlainText())
            self._status(f"日志已保存：{path}", 3000)
        except Exception as e:
            QMessageBox.warning(self, "保存失败", f"无法保存日志：\n{e}")

    def _query_imu6_axis(self):
        if not self._ensure_serial(): return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xC6, b"", tag="QUERY_IMU")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _query_processed_motion(self):
        if not self._ensure_serial(): return
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xC7, b"", tag="QUERY_ATT")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)
            self._send_v21_cmd(fish_id, 0xC8, b"", tag="QUERY_DEPTH")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _toggle_imu_auto_report(self):
        if not self._ensure_serial(): return
        enabled = self.pushButton_3.isChecked()
        payload = bytes([1 if enabled else 0, 0xC6, 1])
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xB6, payload, tag="AUTO_IMU")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)
        self._status(f"IMU 自动回传已{'开启' if enabled else '关闭'}", 2000)

    def _toggle_motion_auto_report(self):
        if not self._ensure_serial(): return
        enabled = self.pushButton_4.isChecked()
        payload_att = bytes([1 if enabled else 0, 0xC7, 1])
        payload_dep = bytes([1 if enabled else 0, 0xC8, 1])
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xB6, payload_att, tag="AUTO_ATT")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)
            self._send_v21_cmd(fish_id, 0xB6, payload_dep, tag="AUTO_DEPTH")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)
        self._status(f"处理后数据自动回传已{'开启' if enabled else '关闭'}", 2000)

    def _apply_bldc_control(self):
        if not self._ensure_serial(): return
        speed = int(self.spin_BLDC_Speed.value())
        sign = 1 if self.radio_BLDC_Forward.isChecked() else -1
        esc = max(-100, min(100, sign * speed))
        payload = int(esc).to_bytes(1, "big", signed=True) + bytes([0])
        for fish_id in self._selected_target_ids():
            self._send_v21_cmd(fish_id, 0xA6, payload, tag="BLDC")
            self._sleep_ms(self.MULTI_SEND_GAP_MS)

    def _apply_pid(self):
        self._todo(f"应用PID: P={self.spin_PID_P.value():.2f}, I={self.spin_PID_I.value():.2f}, D={self.spin_PID_D.value():.2f}")

    def _apply_head_pitch(self):
        self._todo(f"应用鱼头俯仰目标角: {self.spin_HeadPitchTarget.value():.1f}°")

    def _init_env_panels(self):
        self.lbl_GPU_Count_Val.setText("--")
        self.lbl_CUDA_Val.setText("--")
        self.lbl_Vision_Val.setText("未检测")

        QTimer.singleShot(0, self._fill_system_env_on_startup)

    def _fill_system_env_on_startup(self):
        try:
            if SystemMonitor is not None:
                monitor = SystemMonitor()
                data = monitor.get_all()

                os_text = data.get("os") or f"{platform.system()} {platform.release()}"
                self.lbl_OS_Val.setText(str(os_text))

                cpu = data.get("cpu") or {}
                cpu_model = cpu.get("model") or platform.processor() or "未知"
                c_phy = cpu.get("cores_physical")
                c_log = cpu.get("cores_logical")
                if isinstance(c_phy, int) and isinstance(c_log, int) and c_phy > 0 and c_log > 0:
                    self.lbl_CPU_Val.setText(f"{cpu_model} ({c_phy}C/{c_log}T)")
                else:
                    self.lbl_CPU_Val.setText(str(cpu_model))

                mem = data.get("memory") or {}
                total = mem.get("total")
                avail = mem.get("available")
                used = mem.get("percent_used")
                if total and avail and used:
                    self.lbl_RAM_Val.setText(f"总 {total} / 可用 {avail} / 已用 {used}")
                elif total:
                    self.lbl_RAM_Val.setText(str(total))
                else:
                    self.lbl_RAM_Val.setText("未知")

                g = data.get("graphics") or {}
                gpus = g.get("gpus") or []
                if gpus:
                    names = [str(item.get("name", "")).strip() for item in gpus if item.get("name")]
                    self.lbl_GPU_Basic_Val.setText("; ".join(names) if names else "未知")
                else:
                    self.lbl_GPU_Basic_Val.setText("未检测到")
            else:
                self.lbl_OS_Val.setText(f"{platform.system()} {platform.release()}")
                self.lbl_CPU_Val.setText(platform.processor() or "未知")
                self.lbl_RAM_Val.setText("未知")
                self.lbl_GPU_Basic_Val.setText("未知")

            self._status("系统环境已自动填充", 2000)
        except Exception as e:
            self.lbl_OS_Val.setText(f"{platform.system()} {platform.release()}")
            self.lbl_CPU_Val.setText(platform.processor() or "未知")
            self.lbl_RAM_Val.setText("未知")
            self.lbl_GPU_Basic_Val.setText("未知")
            self._log(f"[ENV_INIT_ERR] {e}")

    def _check_compute_env(self):
        gpu_count_text = "0"
        cuda_text = "不可用"
        vision_text = "不支持"

        try:
            torch = importlib.import_module("torch")

            cuda_ok = bool(torch.cuda.is_available())
            count = int(torch.cuda.device_count()) if cuda_ok else 0
            gpu_count_text = str(count)

            if cuda_ok:
                cuda_ver = torch.version.cuda or "未知"
                cuda_text = f"PyTorch CUDA {cuda_ver}"
                names = []
                for i in range(count):
                    try:
                        names.append(torch.cuda.get_device_name(i))
                    except Exception:
                        names.append(f"GPU{i}")
                vision_text = f"支持 (torch {torch.__version__})"
                if names:
                    self.lbl_GPU_Basic_Val.setText("; ".join(names))
            else:
                cuda_text = f"PyTorch 已安装，CUDA不可用 (torch {torch.__version__})"
                vision_text = "仅CPU"

        except Exception as e:
            try:
                if SystemMonitor is not None:
                    monitor = SystemMonitor()
                    g = monitor.get_gpu_cuda_info()
                    gpus = g.get("gpus") or []
                    gpu_count_text = str(len(gpus)) if gpus else "0"
                    cuda = g.get("cuda") or {}
                    if cuda.get("installed"):
                        cuda_text = f"驱动CUDA {cuda.get('cuda_version', '未知')}"
                vision_text = "PyTorch 未安装"
            except Exception:
                vision_text = "检测失败"
            self._log(f"[COMPUTE_CHECK_ERR] {e}")

        self.lbl_GPU_Count_Val.setText(gpu_count_text)
        self.lbl_CUDA_Val.setText(cuda_text)
        self.lbl_Vision_Val.setText(vision_text)
        self._status("算力环境检测完成", 2500)

    def closeEvent(self, event: QtGui.QCloseEvent) -> None:
        self._closing = True
        self._save_auto_config()
        self._close_serial(clear_devices=False)
        event.accept()


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setQuitOnLastWindowClosed(True)
    w = MainWindow()
    w.show()
    sys.exit(app.exec())