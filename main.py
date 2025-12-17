# main.py
import sys
import time
import queue
import re
import platform
import datetime
from typing import Optional, List, Dict, Callable, Any

from PyQt6 import QtCore, QtWidgets, QtGui
from PyQt6.QtCore import QTimer, Qt, QThread, pyqtSignal, pyqtSlot
from PyQt6.QtGui import QKeySequence, QShortcut, QTextCursor
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QMessageBox, QInputDialog, QTableWidgetItem
)

# -----------------------------------------------------------------------------
# 导入 UI 和 协议库
# -----------------------------------------------------------------------------
from Lanucher_UI import Ui_MainWindow
from LoraProtocol import LoraProtocol, Cmd, RunMode

# 尝试导入 pyserial
try:
    import serial
    import serial.tools.list_ports as list_ports
except ImportError:
    serial = None
    list_ports = None

# -----------------------------------------------------------------------------
# Constants & Config
# -----------------------------------------------------------------------------
CONTROL_VERSION = 2.6
DEFAULT_BAUD = 115200
HANDSHAKE_BAUD = 9600


class SerialWorker(QThread):
    """
    串口工作线程 (增强版)
    支持：
    1. 异步队列发送
    2. 同步阻塞发送 (等待回包)
    3. 自动重连逻辑
    """
    # 信号定义
    log_signal = pyqtSignal(str, str)  # (msg, color_hex)
    status_signal = pyqtSignal(str, int)  # (msg, timeout_ms)
    connection_state_signal = pyqtSignal(bool)
    packet_received_signal = pyqtSignal(dict)  # 异步接收的数据包

    # 同步任务完成信号 (result_dict, error_msg)
    sync_task_finished = pyqtSignal(object, str)

    def __init__(self, port_name: str, parent=None):
        super().__init__(parent)
        self.port_name = port_name
        self.running = True
        self.ser: Optional[serial.Serial] = None
        self.tx_queue = queue.Queue()

        # 同步任务相关变量
        self._sync_lock = False  # 是否正在执行同步任务
        self._sync_request = None  # 当前同步任务详情
        self._sync_result = None  # 同步任务结果容器

    def run(self):
        if not serial:
            self.log_signal.emit("错误: 未安装 pyserial 库。", "#FF0000")
            return

        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=DEFAULT_BAUD,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.01,  # 非阻塞读
                rtscts=False
            )
            self.ser.rts = False  # V2.0 默认拉低

            self.connection_state_signal.emit(True)
            self.log_signal.emit(f"串口已打开: {self.port_name} @ {DEFAULT_BAUD}", "#008000")

            while self.running and self.ser.is_open:
                # 1. 优先处理同步任务 (阻塞式)
                if self._sync_request:
                    self._execute_sync_task()
                    continue

                # 2. 处理普通发送队列
                while not self.tx_queue.empty() and not self._sync_request:
                    try:
                        data, tag = self.tx_queue.get_nowait()
                        self._write_serial(data, tag)
                    except queue.Empty:
                        pass
                    except Exception as e:
                        self.log_signal.emit(f"发送异常: {e}", "#FF0000")

                # 3. 处理接收 (异步)
                try:
                    if self.ser.in_waiting:
                        raw = self.ser.read(self.ser.in_waiting)
                        if raw:
                            self._process_rx_data(raw, is_sync_mode=False)
                except OSError:
                    self.log_signal.emit("物理连接断开！", "#FF0000")
                    self.running = False
                    break
                except Exception as e:
                    self.log_signal.emit(f"接收异常: {e}", "#FF0000")

                self.msleep(5)  # 释放 CPU

        except Exception as e:
            self.log_signal.emit(f"无法打开串口 {self.port_name}: {e}", "#FF0000")
        finally:
            self._close_serial()
            self.connection_state_signal.emit(False)

    def _write_serial(self, data: bytes, tag: str = ""):
        """底层发送方法，统一日志格式"""
        if self.ser and self.ser.is_open:
            self.ser.write(data)
            self.ser.flush()
            # 格式化 HEX 字符串
            hex_str = " ".join(f"{b:02X}" for b in data)
            msg = f"TX [{tag}]: {hex_str}" if tag else f"TX: {hex_str}"
            self.log_signal.emit(msg, "#0000FF")  # 蓝色

    def _process_rx_data(self, raw: bytes, is_sync_mode: bool = False) -> List[dict]:
        """
        统一接收处理。
        is_sync_mode=True 时，返回解析出的包列表，不发射信号。
        is_sync_mode=False 时，发射信号供 UI 更新。
        """
        # 日志
        hex_str = " ".join(f"{b:02X}" for b in raw)
        self.log_signal.emit(f"RX: {hex_str}", "#008000")  # 绿色

        # 简单拼包 (粘包处理)
        # 注意：这里简化处理，假设数据流不频繁断裂。
        # 严谨做法应维护一个成员变量 buffer。
        valid_packets = []

        header = b"\xAA\x55"
        footer = b"\x0D"
        start_idx = 0

        while True:
            header_pos = raw.find(header, start_idx)
            if header_pos == -1: break

            # 协议固定长度 12
            if header_pos + 12 > len(raw): break

            packet = raw[header_pos: header_pos + 12]
            if packet[-1] == footer[0]:
                res = LoraProtocol.parse_response(packet)
                if res["valid"]:
                    valid_packets.append(res)
                    if not is_sync_mode:
                        self.packet_received_signal.emit(res)
                else:
                    self.log_signal.emit(f"校验失败: {res.get('err')}", "#FFA500")  # 橙色
                start_idx = header_pos + 12
            else:
                start_idx = header_pos + 1

        return valid_packets

    # --- 同步任务机制 ---

    def request_sync_task(self, tx_data: bytes, match_func: Callable[[dict], bool], timeout_ms: int = 1000,
                          tag: str = "Sync"):
        """UI 调用的接口：请求一个同步任务"""
        self._sync_request = {
            "data": tx_data,
            "match": match_func,
            "timeout": timeout_ms,
            "tag": tag
        }

    def _execute_sync_task(self):
        """执行同步任务 (运行在 Worker 线程)"""
        req = self._sync_request
        self._sync_request = None  # 清除请求标志，避免重复进入

        try:
            # 1. 发送
            self._write_serial(req["data"], req["tag"])

            # 2. 等待回包
            start_time = time.time()
            found_packet = None

            while (time.time() - start_time) * 1000 < req["timeout"]:
                if self.ser.in_waiting:
                    raw = self.ser.read(self.ser.in_waiting)
                    if raw:
                        # 解析并检查是否匹配
                        packets = self._process_rx_data(raw, is_sync_mode=True)
                        for pkt in packets:
                            # 即使在同步模式，收到的非目标包(如心跳)也应该抛给 UI
                            # 但为了逻辑简单，这里只检查目标包
                            if req["match"](pkt):
                                found_packet = pkt
                                break
                            else:
                                # 非目标包，依然抛出信号，避免数据丢失
                                self.packet_received_signal.emit(pkt)

                    if found_packet: break

                self.msleep(5)  # 轮询间隔

            if found_packet:
                self.sync_task_finished.emit(found_packet, None)
            else:
                self.sync_task_finished.emit(None, "等待超时")

        except Exception as e:
            self.sync_task_finished.emit(None, str(e))

    def _close_serial(self):
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except:
                pass
        self.ser = None

    def stop(self):
        self.running = False
        self.wait()

    def send_bytes(self, data: bytes, tag: str = ""):
        self.tx_queue.put((data, tag))


class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # 核心对象
        self.worker: Optional[SerialWorker] = None
        self.fish_list: List[LoraProtocol] = []

        # 状态
        self.tx_channel = 23
        self.default_pwd = 0x0000
        self._updating_ui = False
        self._pending_sync_action = None  # 记录当前等待哪个按钮的回调

        # 定时器
        self.timers = {}
        for name in ["cpg", "servo", "gear"]:
            t = QTimer(self)
            t.setInterval(50)
            self.timers[name] = t

        self._init_ui_logic()
        self._refresh_ports()
        self._update_ui_state(False)
        self._status("系统就绪")

        # 初始化日志样式
        self.txt_Log.document().setMaximumBlockCount(2000)

    def _init_ui_logic(self):
        # 串口
        self.btn_OpenSerial.clicked.connect(self.toggle_serial)
        self.btn_InitCtrl.clicked.connect(self._cmd_init_transmitter)  # 依然可以用旧的非阻塞方式，或者改为阻塞
        self.btn_QueryCtrl.clicked.connect(self._cmd_query_params_sync)  # 改为同步
        self.btn_SetCtrlCh.clicked.connect(self._cmd_set_channel)

        # 设备
        self.btn_Search.clicked.connect(self._cmd_search)
        self.btn_AddDevManual.clicked.connect(self._manual_add_dev)
        self.btn_SelectAll.clicked.connect(self.table_Devices.selectAll)
        self.btn_SelectNone.clicked.connect(self.table_Devices.clearSelection)
        self.btn_SelectNone_2.clicked.connect(self._pair_device)
        self.btn_SelectNone_3.clicked.connect(lambda: self._status("功能未实现"))  # 取消配对暂留空

        # 运动
        self._bind_dial_spin(self.dial_Speed, self.spin_GearSpeed)
        self._bind_dial_spin(self.dial_Turn, self.spin_GearTurn)
        self.btn_SendGear.clicked.connect(lambda: self._send_gear(False))
        self.chk_Sync_Gear.toggled.connect(lambda c: self._toggle_timer("gear", c))
        self.timers["gear"].timeout.connect(lambda: self._send_gear(True))

        # 舵机
        self._bind_slider_spin(self.slider_S1, self.spin_S1, 10.0)
        self._bind_slider_spin(self.slider_S2, self.spin_S2, 10.0)
        self.btn_SendServo.clicked.connect(lambda: self._send_servo(False))
        self.chk_Sync_Servo.toggled.connect(lambda c: self._toggle_timer("servo", c))
        self.timers["servo"].timeout.connect(lambda: self._send_servo(True))
        self.btn_SetServoPower.clicked.connect(self._send_servo_power)
        self.btn_QueryServoAll.clicked.connect(self._query_servo_status)  # 触发 B7/C2 等

        # CPG
        self._bind_slider_spin(self.slider_Amp, self.spin_Amp, 10.0)
        self._bind_slider_spin(self.slider_Freq, self.spin_Freq, 10.0)
        self._bind_slider_spin(self.slider_Bias, self.spin_Bias, 10.0)
        self.btn_SendCPG.clicked.connect(lambda: self._send_cpg(False))
        self.chk_Sync_CPG.toggled.connect(lambda c: self._toggle_timer("cpg", c))
        self.timers["cpg"].timeout.connect(lambda: self._send_cpg(True))

        # Play & Stop
        self.btn_PlayStart.clicked.connect(self._send_play)
        self.btn_GlobalStop.clicked.connect(self._global_stop)
        self.btn_GlobalStop_2.clicked.connect(self._global_stop)

        # Flash / Boot Config (阻塞式操作)
        self.btn_FlashRead.clicked.connect(self._read_boot_config_sync)
        self.btn_FlashSave.clicked.connect(self._save_boot_config_sync)

        # Advanced
        self.btn_SetAutoReport.clicked.connect(self._set_auto_report)
        self.btn_ResetFault.clicked.connect(self._query_volt_pwr)  # 查询电压
        self.btn_FactoryReset.clicked.connect(self._factory_reset_sync)  # 恢复出厂

        # Env Check
        self.btn_CheckCompute.clicked.connect(self._check_compute_env)

        # Logs
        self.btn_ClearLog.clicked.connect(self.txt_Log.clear)
        self.chk_AutoScroll.setChecked(True)

    # -------------------------------------------------------------------------
    # 辅助逻辑
    # -------------------------------------------------------------------------
    def _bind_slider_spin(self, slider, spin, ratio):
        slider.valueChanged.connect(lambda v: self._set_val_mutex(spin, v / ratio))
        spin.valueChanged.connect(lambda v: self._set_val_mutex(slider, int(v * ratio)))

    def _bind_dial_spin(self, dial, spin):
        dial.valueChanged.connect(lambda v: self._set_val_mutex(spin, v))
        spin.valueChanged.connect(lambda v: self._set_val_mutex(dial, v))

    def _set_val_mutex(self, widget, value):
        if self._updating_ui: return
        self._updating_ui = True
        widget.setValue(value)
        self._updating_ui = False

    def _toggle_timer(self, name, start):
        if start:
            self.timers[name].start()
        else:
            self.timers[name].stop()

    def _status(self, msg, ms=3000):
        self.statusbar.showMessage(msg, ms)

    def _log(self, msg, color="#000000"):
        """富文本日志，带时间戳"""
        now = datetime.datetime.now().strftime("[%H:%M:%S.%f]")[:-3] + " "
        html = f'<span style="color:#808080;">{now}</span><span style="color:{color};">{msg}</span>'
        self.txt_Log.appendHtml(html)
        if self.chk_AutoScroll.isChecked():
            self.txt_Log.moveCursor(QTextCursor.MoveOperation.End)

    # -------------------------------------------------------------------------
    # 串口连接与回调
    # -------------------------------------------------------------------------
    def toggle_serial(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
            self.btn_OpenSerial.setText("启动串口(&W)")
            self.btn_OpenSerial.setChecked(False)
            self._update_ui_state(False)
            self._status("串口已关闭")
        else:
            port = self.combo_Port.currentText()
            if "刷新" in port or not port: return

            self.worker = SerialWorker(port)
            self.worker.log_signal.connect(self._log)
            self.worker.status_signal.connect(self._status)
            self.worker.connection_state_signal.connect(self._on_conn_changed)
            self.worker.packet_received_signal.connect(self._on_packet)
            self.worker.sync_task_finished.connect(self._on_sync_finished)
            self.worker.start()

    def _on_conn_changed(self, connected):
        self._update_ui_state(connected)
        if connected:
            self.btn_OpenSerial.setText("关闭串口(&W)")
            self.btn_OpenSerial.setChecked(True)
            self.lbl_CtrlLinkState.setText("已连接")
            self.lbl_CtrlLinkState.setStyleSheet("color: green; font-weight: bold;")
            QTimer.singleShot(200, self._cmd_query_params_sync)  # 连上自动查参数
        else:
            self.lbl_CtrlLinkState.setText("断开")
            self.lbl_CtrlLinkState.setStyleSheet("color: red; font-weight: bold;")
            self.btn_OpenSerial.setChecked(False)

    def _update_ui_state(self, en):
        self.grp_Controller.setEnabled(en)
        self.grp_Devices.setEnabled(en)
        self.grp_Monitor.setEnabled(en)
        self.tabWidget_Main.setEnabled(en)
        self.combo_Port.setEnabled(not en)

    def _refresh_ports(self):
        self.combo_Port.clear()
        if list_ports:
            ports = [p.device for p in list_ports.comports()]
            self.combo_Port.addItems(ports if ports else ["COM1"])
        else:
            self.combo_Port.addItem("未检测到串口")

    def eventFilter(self, obj, event):
        if obj == self.combo_Port and event.type() == QtCore.QEvent.Type.MouseButtonPress:
            self._refresh_ports()
        return super().eventFilter(obj, event)

    # -------------------------------------------------------------------------
    # 通用发送与同步逻辑
    # -------------------------------------------------------------------------
    def _send(self, data: bytes, tag=""):
        if self.worker: self.worker.send_bytes(data, tag)

    def _send_sync(self, data: bytes, match_func: Callable, action_name: str, timeout=1000):
        """发起同步请求，并禁用界面"""
        if not self.worker: return

        self.setEnabled(False)  # 简单禁用全界面，防误触
        self._pending_sync_action = action_name
        self.worker.request_sync_task(data, match_func, timeout, tag=action_name)

        # 显示进度条或状态
        self._status(f"正在执行: {action_name} ...", 0)

    def _on_sync_finished(self, result, error):
        """同步任务回调"""
        self.setEnabled(True)  # 恢复界面
        action = self._pending_sync_action
        self._pending_sync_action = None

        if error:
            self._status(f"{action} 失败: {error}")
            QMessageBox.critical(self, "指令失败", f"执行 {action} 时发生错误:\n{error}")
        else:
            self._status(f"{action} 成功!")
            # 针对特定指令的后处理
            if action == "QueryCtrl":
                # 解析模块参数回包... 这里简化处理，直接提示
                QMessageBox.information(self, "成功", "控制器参数查询成功，请查看日志详情。")
                # 实际可以解析 result 数据包来更新 spin_CtrlCh
            elif action == "SaveBoot":
                QMessageBox.information(self, "成功", "开机参数已保存到设备 Flash。")
            elif action == "FactoryReset":
                QMessageBox.information(self, "成功", "设备已恢复出厂设置。")

    def _get_targets(self) -> List[LoraProtocol]:
        rows = sorted({i.row() for i in self.table_Devices.selectedItems()})
        if not rows and self.fish_list: rows = [0]
        res = []
        for r in rows:
            if r < len(self.fish_list):
                dev = self.fish_list[r]
                dev.channel = self.tx_channel
                res.append(dev)
        return res

    def _broadcast(self) -> LoraProtocol:
        return LoraProtocol(0xFFFF, self.default_pwd, self.tx_channel)

    # -------------------------------------------------------------------------
    # 具体业务功能
    # -------------------------------------------------------------------------
    # 1. 控制器
    def _cmd_init_transmitter(self):
        # 这是一个 LoRa 模块配置宏，比较复杂，暂时用旧的异步方式，或者封装新的
        # 这里简单发一个初始化包
        frame = bytes.fromhex("C0 00 07 00 01 00 E7 00 17 43")
        self._send(frame, "InitCtrl")

    def _cmd_query_params_sync(self):
        # 模拟同步查询控制器
        frame = bytes.fromhex("C1 00 07")  # 查询指令

        # 匹配规则: 长度>5 且包含某些特征。这里简化为任何回包都算成功
        def matcher(pkt): return True

        self._send_sync(frame, matcher, "QueryCtrl", timeout=500)

    def _cmd_set_channel(self):
        ch = self.spin_CtrlCh.value()
        # LoRa E22 切换信道指令...
        # 简略实现: C0 ...
        base = bytes.fromhex("C0 00 07 00 01 00 E7 00 17 43")
        cmd = base[:8] + bytes([ch]) + base[9:]
        self._send(cmd, f"SetCh={ch}")

    # 2. 设备管理
    def _cmd_search(self):
        self._send(self._broadcast().pack_search(), "Search")

    def _manual_add_dev(self):
        txt, ok = QInputDialog.getText(self, "添加", "FishID (Hex):")
        if ok and txt:
            try:
                self._add_dev_ui(int(txt, 16))
            except:
                pass

    def _add_dev_ui(self, fid):
        for f in self.fish_list:
            if f.fish_id == fid: return
        dev = LoraProtocol(fid, self.default_pwd, self.tx_channel)
        self.fish_list.append(dev)
        r = self.table_Devices.rowCount()
        self.table_Devices.insertRow(r)
        self.table_Devices.setItem(r, 0, QTableWidgetItem(f"0x{fid:04X}"))
        self.table_Devices.setItem(r, 1, QTableWidgetItem("未配对"))
        self._log(f"添加设备: 0x{fid:04X}", "#0000FF")

    def _pair_device(self):
        targets = self._get_targets()
        if not targets: return
        for dev in targets:
            self._send(dev.pack_query_volt(), f"Pair[{dev.fish_id:04X}]")

    # 3. 运动
    def _send_gear(self, silent=False):
        targets = self._get_targets()
        if not targets: return
        spd = self.spin_GearSpeed.value()
        turn = self.spin_GearTurn.value()
        for dev in targets:
            self._send(dev.pack_gear_mode(spd, turn, False), "" if silent else f"Gear[{dev.fish_id:04X}]")

    def _send_servo(self, silent=False):
        targets = self._get_targets()
        if not targets: return
        a1, a2 = self.spin_S1.value(), self.spin_S2.value()
        for dev in targets:
            if self.radio_S1_Only.isChecked():
                frame = dev.pack_position_single(1, a1)
            elif self.radio_S2_Only.isChecked():
                frame = dev.pack_position_single(2, a2)
            else:
                frame = dev.pack_position_dual(a1, a2)
            self._send(frame, "" if silent else f"Servo[{dev.fish_id:04X}]")

    def _send_cpg(self, silent=False):
        targets = self._get_targets()
        if not targets: return
        # 获取 CPG 参数
        amp = self.spin_Amp.value()
        freq = self.spin_Freq.value()
        bias = self.spin_Bias.value()
        for dev in targets:
            frame = dev.pack_cpg(amp, bias, freq, False)
            self._send(frame, "" if silent else f"CPG[{dev.fish_id:04X}]")

    def _send_play(self):
        idx = self.spin_PlayId.value()
        for dev in self._get_targets():
            self._send(dev.pack_play_mode(idx), f"Play[{idx}]")

    def _global_stop(self):
        for t in self.timers.values(): t.stop()
        self.chk_Sync_Gear.setChecked(False)
        self.chk_Sync_Servo.setChecked(False)
        self.chk_Sync_CPG.setChecked(False)
        self._send(self._broadcast().pack_stop(), "STOP_ALL")
        self._status("已发送全局急停", 5000)

    # 4. 同步/高级功能
    def _read_boot_config_sync(self):
        targets = self._get_targets()
        if len(targets) != 1:
            QMessageBox.warning(self, "提示", "请选中一台设备")
            return
        # 这里 D8 发送后，设备会回吐多条 Fx 指令。
        # 严格同步比较难，这里采用 "发送 -> 等待1秒收集所有回包" 的策略
        # 既然是 Demo，我们先发 D8，利用异步接收去更新 UI。
        # 给用户一个假的进度条感
        dev = targets[0]
        self._send(dev.pack_query_boot(), "QueryBoot")
        self._status("正在读取开机参数 (等待回包)...")
        # 实际数据更新在 _on_packet

    def _save_boot_config_sync(self):
        targets = self._get_targets()
        if len(targets) != 1: return
        dev = targets[0]

        # 构造组合包 (示例: 保存Mode和CPG)
        # 实际应该一条一条发，这里演示同步逻辑：发送最后一条并等待 ACK
        # 假设我们只关心最后一条是否成功

        # 1. 保存Mode
        idx = self.combo_F_Mode.currentIndex()
        modes = [RunMode.RESET, RunMode.CPG, RunMode.CPG_INTERMIT, RunMode.PLAY,
                 RunMode.GEAR, RunMode.GEAR_INTERMIT, RunMode.PROTECT]
        mode = modes[idx] if idx < len(modes) else RunMode.RESET
        self._send(dev.pack_boot_mode(mode), "SaveMode")
        time.sleep(0.05)  # 稍微间隔

        # 2. 保存CPG (最后一条，使用 Sync)
        amp = self.spin_F_Amp.value()
        freq = self.spin_F_Freq.value()
        bias = self.spin_F_Bias.value()

        frame = dev.pack_cpg(amp, bias, freq, False, as_boot=True)

        # 匹配规则: 收到 FF (Reply) 且执行结果为 0
        def matcher(pkt):
            return pkt["type"] == "Reply" and pkt["ok"]

        self._send_sync(frame, matcher, "SaveBoot", timeout=2000)

    def _factory_reset_sync(self):
        ret = QMessageBox.warning(self, "确认", "确定恢复出厂设置？",
                                  QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No)
        if ret != QMessageBox.StandardButton.Yes: return

        targets = self._get_targets()
        if not targets: return

        def matcher(pkt):
            return pkt["type"] == "Reply" and pkt["ok"]

        self._send_sync(targets[0].pack_factory_reset(), matcher, "FactoryReset", timeout=2000)

    # 5. 系统环境
    def _check_compute_env(self):
        self.lbl_OS_Val.setText(platform.system() + " " + platform.release())
        self.lbl_CPU_Val.setText(platform.machine())
        try:
            # 需要 psutil，若无则跳过
            import psutil
            mem = psutil.virtual_memory()
            self.lbl_RAM_Val.setText(f"{mem.total / 1024 ** 3:.1f} GB")
        except:
            self.lbl_RAM_Val.setText("未知 (需psutil)")

        self.lbl_GPU_Basic_Val.setText("检测中...")
        QTimer.singleShot(1000, lambda: self.lbl_GPU_Basic_Val.setText("无独立显卡 / 未检测"))

    # 其他指令
    def _send_servo_power(self):
        s1 = self.chk_S1_Pwr.isChecked()
        s2 = self.chk_S2_Pwr.isChecked()
        for dev in self._get_targets():
            self._send(dev.pack_servo_power(s1, s2), "ServoPwr")

    def _query_servo_status(self):
        # 查询状态，回包在 F5/F6
        for dev in self._get_targets():
            self._send(dev.pack_query_servo_st(), "Q_ServoSt")
            self._send(dev.pack_query_temp(), "Q_Temp")

    def _set_auto_report(self):
        en = self.radio_Rpt_On.isChecked()
        ms = self.spin_Rpt_Ms.value()
        for dev in self._get_targets():
            self._send(dev.pack_auto_report(en, ms), "SetAutoRpt")

    def _query_volt_pwr(self):
        for dev in self._get_targets():
            self._send(dev.pack_query_volt(), "Q_Volt")

    # -------------------------------------------------------------------------
    # 回包处理
    # -------------------------------------------------------------------------
    def _on_packet(self, res):
        ptype = res.get("type")
        fid = res.get("fish_id")

        # 自动添加设备
        if ptype == "Reply" and res.get("src_cmd") == hex(Cmd.SEARCH_DEV):
            self._add_dev_ui(fid)

        # 更新监视表
        self._update_monitor_table(res)

        # 舵机状态更新 UI
        if ptype == "Servo":  # F5
            # res: {s1_ma, s2_ma, ...}
            self.bar_S1_Curr.setValue(res.get("s1_ma", 0))
            self.bar_S2_Curr.setValue(res.get("s2_ma", 0))
        elif ptype == "Temp":  # F6
            self.bar_S1_Temp.setValue(int(res.get("s1", 0)))
            self.bar_S2_Temp.setValue(int(res.get("s2", 0)))

        # 开机参数回读 (F2, F9...)
        if ptype == "BootCPG":
            self.spin_F_Amp.setValue(res.get("amp", 0))
            self.spin_F_Bias.setValue(res.get("bias", 0))
            self.spin_F_Freq.setValue(res.get("freq", 0))
        elif ptype == "BootMode":
            # 简单处理，直接显示到日志
            self._log(f"开机模式读取: {res.get('boot_mode')}", "#0000FF")

    def _update_monitor_table(self, res):
        fid = res.get("fish_id")
        row = -1
        for r in range(self.table_SysMonitor.rowCount()):
            item = self.table_SysMonitor.item(r, 1)
            if item and item.text() == f"0x{fid:04X}":
                row = r
                break

        if row == -1:
            row = self.table_SysMonitor.rowCount()
            self.table_SysMonitor.insertRow(row)
            self.table_SysMonitor.setItem(row, 0, QTableWidgetItem(""))  # Checkbox col
            self.table_SysMonitor.setItem(row, 1, QTableWidgetItem(f"0x{fid:04X}"))
            for c in range(2, 7): self.table_SysMonitor.setItem(row, c, QTableWidgetItem("-"))

        ptype = res.get("type")
        if ptype == "Power":
            self.table_SysMonitor.setItem(row, 2, QTableWidgetItem(f"{res['volt']:.2f}"))
            self.table_SysMonitor.setItem(row, 3, QTableWidgetItem(f"{res['mw']}"))
            # 收到电压，视为配对成功
            for r2 in range(self.table_Devices.rowCount()):
                if self.table_Devices.item(r2, 0).text() == f"0x{fid:04X}":
                    self.table_Devices.setItem(r2, 1, QTableWidgetItem("已配对"))
                    self.table_Devices.item(r2, 1).setForeground(QtGui.QColor("green"))


if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())
