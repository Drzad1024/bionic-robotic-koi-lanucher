"""
LoraProtocol.py
===================
基于 **新锦鲤通讯协议 V1.5** 的完整 Python 实现 (STM32 C代码对齐版)

特性：
1. 严格对齐 STM32 C 代码逻辑 (以 protocol.c 为准)
2. 覆盖 A0~FF 全指令集
3. 内置详细的单元测试

Author  : Embedded Architect
Version : 1.5.1
Target  : STM32F103 (New Koi Robot)
"""

import struct
import enum
import math
from typing import Tuple, Dict, Any, Union, Optional

# ==========================================
# 常量定义
# ==========================================

HEAD_BYTES = b"\xAA\x55"
TAIL_BYTE = 0x0D
FRAME_LEN = 12

class Cmd(enum.IntEnum):
    # --- 0xA* 运动控制 ---
    RESET_STOP      = 0xA0  # 停止并复位
    CPG_MODE        = 0xA1  # CPG 模式
    INTERMITTENT    = 0xA2  # 间歇模式
    POSITION        = 0xA3  # 舵机位置
    PLAY_MODE       = 0xA4  # 表演模式
    GEAR_MODE       = 0xA5  # 档位模式

    # --- 0xB* 系统配置 ---
    MSG_REPLY_CFG   = 0xB0  # 消息回复开关/超时
    LED_CTRL        = 0xB1  # LED 灯效
    SET_PWD         = 0xB2  # 修改密码
    SET_ID          = 0xB3  # 修改 ID
    SET_CHANNEL     = 0xB4  # 修改信道
    QUERY_VER       = 0xB5  # 查询版本
    SERVO_PWR       = 0xB6  # 舵机电源
    QUERY_RUN_STAT  = 0xB7  # 查询运行状态
    QUERY_BATTERY   = 0xB8  # 查询电量
    RESET_SERVO     = 0xB9  # 复位舵机
    SEARCH_DEV      = 0xBA  # 搜索设备

    # --- 0xC* 传感器 ---
    QUERY_VOLT_PWR  = 0xC1  # 查询电压功率
    QUERY_SERVO_ST  = 0xC2  # 查询舵机状态
    QUERY_TEMP      = 0xC3  # 查询温度
    AUTO_REPORT_CFG = 0xC4  # 设置自动回报

    # --- 0xD* 开机参数 (EEPROM) ---
    BOOT_MODE       = 0xD0  # 开机模式
    BOOT_CPG        = 0xD1  # 开机 CPG
    BOOT_INTERMIT   = 0xD2  # 开机间歇
    BOOT_PLAY       = 0xD3  # 开机表演
    BOOT_GEAR       = 0xD4  # 开机档位
    BOOT_LED        = 0xD5  # 开机 LED
    BOOT_REPLY      = 0xD6  # 开机回复
    FACTORY_RESET   = 0xD7  # 恢复出厂
    QUERY_BOOT      = 0xD8  # 查询开机参数
    INSTALL_OFFSET  = 0xD9  # 安装偏置
    QUERY_FLASH     = 0xDF  # 查询 Flash (调试用)

    # --- 0xF* 返回帧 ---
    RET_STATUS      = 0xF1
    RET_BOOT_CPG    = 0xF2
    RET_BOOT_GEN    = 0xF3
    RET_VOLT_PWR    = 0xF4
    RET_SERVO_ST    = 0xF5
    RET_TEMP        = 0xF6
    RET_BATTERY     = 0xF7
    RET_BOOT_INT    = 0xF8
    RET_BOOT_GEAR   = 0xF9
    RET_OFFSET      = 0xFA
    RET_CMD_REPLY   = 0xFF

class LedMode(enum.IntEnum):
    OFF             = 0
    ON              = 1
    SLOW_BLINK      = 2
    FAST_BLINK      = 3
    HEARTBEAT       = 4

class RunMode(enum.IntEnum):
    RESET           = 0
    CPG             = 1
    CPG_INTERMIT    = 2
    POSITION        = 3
    PLAY            = 4
    GEAR            = 5
    GEAR_INTERMIT   = 6
    PROTECT         = 7

# ==========================================
# 辅助函数
# ==========================================

def _xor_checksum(buf: bytes) -> int:
    chk = 0
    for b in buf[:10]:
        chk ^= b
    return chk & 0xFF

def _clamp(val, min_v, max_v):
    return max(min(val, max_v), min_v)

def _pack_u16_be(val: int) -> bytes:
    return struct.pack(">H", val & 0xFFFF)

# ==========================================
# 核心协议类
# ==========================================

class LoraProtocol:
    def __init__(self, fish_id: int, password: int = 0x0000, target_channel: int = 0x17):
        self.fish_id = fish_id
        self.password = password
        self.channel = target_channel

    def _build_frame(self, cmd: int, data: Tuple[int, int, int], with_lora_header: bool = True) -> bytes:
        """构建协议帧 (Frame = 12 Bytes)"""
        payload = bytearray(FRAME_LEN)
        payload[0:2] = HEAD_BYTES
        payload[2:4] = _pack_u16_be(self.fish_id)
        payload[4:6] = _pack_u16_be(self.password)
        payload[6] = cmd
        payload[7] = data[0] & 0xFF
        payload[8] = data[1] & 0xFF
        payload[9] = data[2] & 0xFF
        payload[10] = _xor_checksum(payload)
        payload[11] = TAIL_BYTE

        if with_lora_header:
            # LoRa E22 定点头: AddrH + AddrL + Channel
            header = struct.pack(">HB", self.fish_id, self.channel)
            return header + payload
        else:
            return bytes(payload)

    # ================================================================
    # PACK 方法 (上位机 -> 下位机)
    # ================================================================

    # --- 0xA* 运动控制 ---

    def pack_stop(self) -> bytes:
        """A0: 停止并复位"""
        return self._build_frame(Cmd.RESET_STOP, (0, 0, 0))

    def pack_cpg(self, amp_deg: float, bias_deg: float, freq_hz: float,
                 intermittent_en: bool = False, as_boot: bool = False) -> bytes:
        """A1/D1: CPG 控制"""
        # C 代码逻辑:
        # AA = amp * 10
        # BBCC: [15:Dir] [14-7:Bias*10] [6-1:Freq*10] [0:Int]
        aa = int(round(_clamp(abs(amp_deg), 0, 30.0) * 10))

        bias_dir = 1 if bias_deg >= 0 else 0 # 协议: 1为右/逆(正), 0为左/顺(负)
        bias_val = int(round(_clamp(abs(bias_deg), 0, 30.0) * 10))
        freq_val = int(round(_clamp(freq_hz, 0, 3.8) * 10))
        int_bit = 1 if intermittent_en else 0

        bbcc = (bias_dir << 15) | ((bias_val & 0xFF) << 7) | ((freq_val & 0x3F) << 1) | int_bit

        cmd = Cmd.BOOT_CPG if as_boot else Cmd.CPG_MODE
        return self._build_frame(cmd, (aa, (bbcc >> 8), bbcc & 0xFF))

    def pack_intermittent(self, n_burst: int, t_coast_ms: int, enable: bool = True, as_boot: bool = False) -> bytes:
        """A2/D2: 间歇参数 (Tcoast精度100ms)"""
        # C 代码逻辑: Intermit_Pack -> AABB = (N<<12) | T_coast_unit
        t_unit = int(t_coast_ms / 100)
        combined = ((n_burst & 0x0F) << 12) | (t_unit & 0x0FFF)

        cmd = Cmd.BOOT_INTERMIT if as_boot else Cmd.INTERMITTENT
        return self._build_frame(cmd, ((combined >> 8), combined & 0xFF, 1 if enable else 0))

    def pack_position_single(self, servo_idx: int, angle: float) -> bytes:
        """A3: 单舵机位置 (angle: -90~90, 0为中点)"""
        # C 代码逻辑: Position_Pack(single=1)
        # AA[7]=0, AA[6]=Idx, AA[5]=Dir
        # BBCC = Mag (0.1 deg)
        idx_bit = 1 if servo_idx == 2 else 0

        # 协议定义：90度为中位。输入0对应协议90。
        proto_ang = 90.0 + angle
        dir_bit = 1 if proto_ang >= 90 else 0
        mag = int(round(abs(angle) * 10))
        mag = _clamp(mag, 0, 900)

        aa = (0 << 7) | (idx_bit << 6) | (dir_bit << 5)
        return self._build_frame(Cmd.POSITION, (aa, (mag >> 8), mag & 0xFF))

    def pack_position_dual(self, angle1: float, angle2: float) -> bytes:
        """A3: 双舵机位置"""
        # C 代码逻辑: Position_Pack(single=0)
        # 20bit mag combined
        mag1 = _clamp(int(round(abs(angle1) * 10)), 0, 900)
        mag2 = _clamp(int(round(abs(angle2) * 10)), 0, 900)

        proto_a1 = 90.0 + angle1
        proto_a2 = 90.0 + angle2
        d1 = 1 if proto_a1 >= 90 else 0
        d2 = 1 if proto_a2 >= 90 else 0

        # S1(10bit) << 10 | S2(10bit)
        combined = (mag1 << 10) | mag2

        aa_val = (combined >> 16) & 0x0F
        aa = (1 << 7) | (d1 << 6) | (d2 << 5) | aa_val

        return self._build_frame(Cmd.POSITION, (aa, (combined >> 8), combined & 0xFF))

    def pack_play_mode(self, mode_idx: int, as_boot: bool = False) -> bytes:
        """A4/D3: 表演模式"""
        cmd = Cmd.BOOT_PLAY if as_boot else Cmd.PLAY_MODE
        # C 代码: Protocol_ParseCommand case 0xA4 -> uses cc
        return self._build_frame(cmd, (0, 0, mode_idx))

    def pack_gear_mode(self, speed: int, turn: int, intermit: bool = False, as_boot: bool = False) -> bytes:
        """A5/D4: 档位模式"""
        # 修正：依据 C 代码 Protocol_ParseCommand case 0xA5
        # bool intermit_en = (aa == 1);
        # uint8_t gear_v = (bb >> 4) & 0x0F;
        # uint8_t gear_w = bb & 0x0F;
        # Byte 7=Enable, Byte 8=Gear, Byte 9=Ignored(CC)
        aa = 1 if intermit else 0
        bb = ((speed & 0x0F) << 4) | (turn & 0x0F)

        cmd = Cmd.BOOT_GEAR if as_boot else Cmd.GEAR_MODE
        if as_boot:
             # D4逻辑: cc >> 4 = v, cc & 0x0F = w. (Wait, C code D4 uses CC for Gear?)
             # Check C code case 0xD4:
             # g_robot_config.boot_gear_v = (cc >> 4) & 0x0F;
             # g_robot_config.boot_gear_w = cc & 0x0F;
             # BB maps to enable logic? "if (bb == 1 ...)"
             # 所以 D4 和 A5 的打包格式不同！
             # D4: AA=0, BB=Enable, CC=Gear
             return self._build_frame(cmd, (0, aa, bb)) # BB=Enable, CC=Gear
        else:
             # A5: AA=Enable, BB=Gear, CC=0
             return self._build_frame(cmd, (aa, bb, 0))

    # --- 0xB* 系统配置 ---

    def pack_reply_config(self, timeout_s: int, enable_reply: bool) -> bytes:
        """B0: 消息回复设置 (Enable=True表示开启回复)"""
        # C 代码: msg_reply_stat = (cc == 0) ? 0 : 1; (0 is ON)
        # AA BB = Timeout
        cc = 0 if enable_reply else 1
        return self._build_frame(Cmd.MSG_REPLY_CFG, ((timeout_s >> 8), timeout_s & 0xFF, cc))

    def pack_led(self, mode: LedMode, as_boot: bool = False) -> bytes:
        """B1/D5: LED 设置"""
        cmd = Cmd.BOOT_LED if as_boot else Cmd.LED_CTRL
        # C 代码: uses CC
        return self._build_frame(cmd, (0, 0, mode))

    def pack_set_password(self, new_pwd: int) -> bytes:
        """B2: 修改密码"""
        # C 代码: uses AA BB
        return self._build_frame(Cmd.SET_PWD, ((new_pwd >> 8), new_pwd & 0xFF, 0))

    def pack_set_id(self, new_id: int) -> bytes:
        """B3: 修改 ID"""
        return self._build_frame(Cmd.SET_ID, ((new_id >> 8), new_id & 0xFF, 0))

    def pack_set_channel(self, new_ch: int) -> bytes:
        """B4: 修改信道"""
        # C 代码: uses CC
        return self._build_frame(Cmd.SET_CHANNEL, (0, 0, new_ch))

    def pack_query_version(self) -> bytes:
        """B5: 查询版本"""
        return self._build_frame(Cmd.QUERY_VER, (0, 0, 0))

    def pack_servo_power(self, s1_on: bool, s2_on: bool) -> bytes:
        """B6: 舵机电源"""
        # C 代码: s1_on = (aa & 0x02), s2_on = (aa & 0x01)
        val = 0
        if s1_on: val |= 0x02
        if s2_on: val |= 0x01
        return self._build_frame(Cmd.SERVO_PWR, (val, 0, 0))

    def pack_query_status(self) -> bytes:
        """B7: 查询状态"""
        return self._build_frame(Cmd.QUERY_RUN_STAT, (0, 0, 0))

    def pack_query_battery(self) -> bytes:
        """B8: 查询电量"""
        return self._build_frame(Cmd.QUERY_BATTERY, (0, 0, 0))

    def pack_reset_servo(self) -> bytes:
        """B9: 复位舵机"""
        return self._build_frame(Cmd.RESET_SERVO, (0, 0, 0))

    def pack_search(self) -> bytes:
        """BA: 搜索设备 (广播)"""
        # 这里应该把 self.fish_id 临时改为 0xFFFF 发送，但在 frame build 内部处理比较好
        # 为保持 clean，调用者应实例化一个 ID=0xFFFF 的对象来搜索，或者这里不做特殊处理
        return self._build_frame(Cmd.SEARCH_DEV, (0, 0, 0))

    # --- 0xC* 传感器 ---

    def pack_query_volt(self) -> bytes:
        """C1: 查询电压"""
        return self._build_frame(Cmd.QUERY_VOLT_PWR, (0, 0, 0))

    def pack_query_servo_st(self) -> bytes:
        """C2: 查询舵机状态"""
        return self._build_frame(Cmd.QUERY_SERVO_ST, (0, 0, 0))

    def pack_query_temp(self) -> bytes:
        """C3: 查询温度"""
        return self._build_frame(Cmd.QUERY_TEMP, (0, 0, 0))

    def pack_auto_report(self, enable: bool, interval_ms: int) -> bytes:
        """C4: 设置自动回报"""
        # C 代码 Callback_Pack: AABB [15:En] [10-5:Min] [4-2:SecCode] [1-0:MSecCode]
        if not enable:
            return self._build_frame(Cmd.AUTO_REPORT_CFG, (0, 0, 0))

        mins = interval_ms // 60000
        rem = interval_ms % 60000
        secs = rem // 1000
        msecs = rem % 1000

        # Sec Table
        s_map = {0:0, 1:1, 2:2, 5:3, 10:4, 20:5, 30:6, 45:7}
        s_code = 0
        # 寻找最近的键
        s_code = min(s_map, key=lambda x: abs(x - secs))
        s_val = s_map[s_code]

        # MSec Table
        ms_map = {0:0, 100:1, 200:2, 500:3}
        ms_code = min(ms_map, key=lambda x: abs(x - msecs))
        ms_val = ms_map[ms_code]

        combined = (1 << 15) | ((mins & 0x3F) << 5) | ((s_val & 0x07) << 2) | (ms_val & 0x03)
        return self._build_frame(Cmd.AUTO_REPORT_CFG, ((combined >> 8), combined & 0xFF, 0))

    # --- 0xD* 开机参数 ---

    def pack_boot_mode(self, mode: RunMode) -> bytes:
        """D0: 开机模式"""
        # C 代码: uses CC
        return self._build_frame(Cmd.BOOT_MODE, (0, 0, mode))

    def pack_boot_reply(self, enable: bool) -> bytes:
        """D6: 开机回复"""
        # C 代码: cc=0开启, 1关闭
        return self._build_frame(Cmd.BOOT_REPLY, (0, 0, 0 if enable else 1))

    def pack_factory_reset(self) -> bytes:
        """D7: 恢复出厂"""
        return self._build_frame(Cmd.FACTORY_RESET, (0, 0, 0))

    def pack_query_boot(self) -> bytes:
        """D8: 查询开机参数"""
        return self._build_frame(Cmd.QUERY_BOOT, (0, 0, 0))

    def pack_install_offset(self, s1: int, s2: int) -> bytes:
        """D9: 安装偏置"""
        # C 代码: AA[7]=D1, [6:4]=V1, [3]=D2, [2:0]=V2
        def enc(v):
            d = 1 if v >= 0 else 0
            val = _clamp(abs(v), 0, 7)
            return (d << 3) | val
        aa = (enc(s1) << 4) | enc(s2)
        return self._build_frame(Cmd.INSTALL_OFFSET, (aa, 0, 0))

    def pack_query_flash(self, addr: int) -> bytes:
        """DF: 查询 Flash"""
        # C 代码: uses CC for addr
        return self._build_frame(Cmd.QUERY_FLASH, (0, 0, addr))

    # ================================================================
    # UNPACK 方法 (下位机 -> 上位机)
    # ================================================================

    @staticmethod
    def parse_response(frame: bytes) -> Dict[str, Any]:
        """解析 F* 返回帧"""
        if len(frame) < FRAME_LEN:
            return {"valid": False, "err": "len"}
        if frame[0:2] != HEAD_BYTES:
            return {"valid": False, "err": "head"}
        if _xor_checksum(frame) != frame[10]:
            return {"valid": False, "err": "chk"}

        cmd = frame[6]
        aa, bb, cc = frame[7], frame[8], frame[9]
        fid = (frame[2] << 8) | frame[3]

        res = {"valid": True, "cmd_hex": hex(cmd), "fish_id": fid}

        if cmd == 0xF1: # 状态
            res["type"] = "Status"
            res["mode"] = (aa >> 5) & 0x07
            res["led"] = (aa >> 2) & 0x07
            res["reply"] = (aa & 0x02) == 0
            res["play_idx"] = bb
            res["err"] = cc
        elif cmd == 0xF2: # 开机 CPG
            res["type"] = "BootCPG"
            res["amp"] = aa / 10.0
            bias_val = (bb << 8 | cc)
            b_dir = (bias_val >> 15) & 1
            b_mag = (bias_val >> 7) & 0xFF
            res["bias"] = (b_mag / 5.0) * (1 if b_dir else -1)
            res["freq"] = ((bias_val >> 1) & 0x3F) / 10.0
        elif cmd == 0xF3: # 开机通用
            res["type"] = "BootGen"
            res["boot_mode"] = (aa >> 5) & 0x07
            res["boot_led"] = (aa >> 2) & 0x07
            res["auto_rep_raw"] = (bb << 8) | cc
        elif cmd == 0xF4: # 电压
            res["type"] = "Power"
            res["volt"] = (aa / 100.0) + 3.5
            res["mw"] = (bb << 8) | cc
        elif cmd == 0xF5: # 舵机
            res["type"] = "Servo"
            res["s1_on"] = (aa & 0x80) != 0
            res["s2_on"] = (aa & 0x40) != 0
            res["s1_bad"] = (aa & 0x20) != 0
            res["s2_bad"] = (aa & 0x10) != 0
            res["s1_ma"] = bb * 20
            res["s2_ma"] = cc * 20
        elif cmd == 0xF6: # 温度
            val = (aa << 16) | (bb << 8) | cc
            res["type"] = "Temp"
            res["s1"] = ((val >> 12) & 0xFFF) / 10.0
            res["s2"] = (val & 0xFFF) / 10.0
        elif cmd == 0xF7: # 电量
            val = (aa << 16) | (bb << 8) | cc
            res["type"] = "Battery"
            res["run_s"] = ((val >> 8) & 0xFFFF) * 0.5
            res["rem_h"] = ((val >> 5) & 0x07) * 0.5
            res["pct"] = (val & 0x1F) * 5
        elif cmd == 0xF8: # 开机间歇
            val = (aa << 8) | bb
            res["type"] = "BootInt"
            res["n"] = (val >> 12) & 0x0F
            res["t_coast"] = (val & 0x0FFF) * 100
        elif cmd == 0xF9: # 开机档位
            res["type"] = "BootGear"
            res["int_en"] = (aa == 1)
            res["speed"] = (bb >> 4) & 0x0F
            res["turn"] = bb & 0x0F
            res["play_idx"] = cc
        elif cmd == 0xFA: # 安装偏置
            res["type"] = "InstallOff"
            res["s1"] = (aa >> 4) & 0x07
            if not (aa & 0x80): res["s1"] = -res["s1"] # Bit 7 Dir
            res["s2"] = aa & 0x07
            if not (aa & 0x08): res["s2"] = -res["s2"] # Bit 3 Dir
        elif cmd == 0xFF: # 回复
            res["type"] = "Reply"
            res["src_cmd"] = hex(aa)
            res["err"] = bb
            res["ok"] = (cc == 0)
        else:
            res["type"] = "Unknown"
            res["raw"] = f"{aa:02X} {bb:02X} {cc:02X}"

        return res

# ==========================================
# 完整测试用例 (Main)
# ==========================================

if __name__ == "__main__":
    def hex_str(b): return " ".join(f"{x:02X}" for x in b)

    koi = LoraProtocol(0x0001, 0x0000, 0x17)
    print(f"{'CMD Name':<15} | {'Function':<20} | {'Hex Output':<40} | {'Check'}")
    print("-" * 90)

    # 定义测试用例：(PackFunc, Args, Expected_Cmd_Byte, Expected_Key_Bytes_Indices, Expected_Key_Values)
    # Key Indices 基于帧体 (Payload)，Payload[0]=AA, [6]=Cmd, [7]=AA, [8]=BB, [9]=CC
    # 这里的 Expected Indices 是相对于 Payload 的偏移 (7, 8, 9)

    tests = [
        # --- A系列 ---
        ("Stop", koi.pack_stop, (), 0xA0, [7,8,9], [0,0,0]),
        ("CPG", koi.pack_cpg, (20, -5, 1.5, False), 0xA1, [7,8,9], [0xC8, 0x19, 0x1E]),

        # N=5, T=2000ms(20 units). Combined = (5<<12)|20 = 0x5014. AA=50, BB=14, CC=1
        ("Intermit", koi.pack_intermittent, (5, 2000, True), 0xA2, [7,8,9], [0x50, 0x14, 0x01]),

        # Single S2(Idx=1), -45deg. 90-45=45(0x2D). Dir=0. Mag=450(0x1C2).
        # AA = (0<<7)|(1<<6)|(0<<5) = 0x40. BB=01, CC=C2
        ("Pos Single", koi.pack_position_single, (2, -45), 0xA3, [7,8,9], [0x40, 0x01, 0xC2]),

        # Dual S1=30, S2=-30.
        # S1: 120(>90,D=1), Mag=300(12C). S2: 60(<90,D=0), Mag=300(12C).
        # Combined = (12C<<10)|12C = 0x4B12C.
        # AA = (1<<7)|(1<<6)|(0<<5)|(4) = C0|4 = C4. BB=B1, CC=2C.
        ("Pos Dual", koi.pack_position_dual, (30, -30), 0xA3, [7,8,9], [0xC4, 0xB1, 0x2C]),

        ("Play", koi.pack_play_mode, (3,), 0xA4, [9], [0x03]),

        # Gear: Spd=10(A), Turn=7(Straight). Int=True.
        # A5: AA=1, BB=(A<<4)|7 = A7.
        ("Gear", koi.pack_gear_mode, (10, 7, True), 0xA5, [7,8], [0x01, 0xA7]),

        # --- B系列 ---
        # Reply: Timeout=10s(000A), En=True(CC=0).
        ("ReplyCfg", koi.pack_reply_config, (10, True), 0xB0, [7,8,9], [0x00, 0x0A, 0x00]),
        ("LED", koi.pack_led, (LedMode.HEARTBEAT,), 0xB1, [9], [0x04]),
        ("SetPwd", koi.pack_set_password, (0x1234,), 0xB2, [7,8], [0x12, 0x34]),
        ("SetID", koi.pack_set_id, (0x0002,), 0xB3, [7,8], [0x00, 0x02]),
        ("SetCh", koi.pack_set_channel, (80,), 0xB4, [9], [0x50]),
        ("Ver", koi.pack_query_version, (), 0xB5, [], []),
        ("ServoPwr", koi.pack_servo_power, (True, False), 0xB6, [7], [0x02]),
        ("RunStat", koi.pack_query_status, (), 0xB7, [], []),
        ("Bat", koi.pack_query_battery, (), 0xB8, [], []),
        ("RstServo", koi.pack_reset_servo, (), 0xB9, [], []),
        ("Search", koi.pack_search, (), 0xBA, [], []),

        # --- C系列 ---
        ("Volt", koi.pack_query_volt, (), 0xC1, [], []),
        ("ServoSt", koi.pack_query_servo_st, (), 0xC2, [], []),
        ("Temp", koi.pack_query_temp, (), 0xC3, [], []),

        # AutoRep: En=True, 2500ms. Min=0. Sec=2(Code2). MSec=500(Code3).
        # Combined = (1<<15) | (0) | (2<<2) | 3 = 8000 | 8 | 3 = 800B.
        ("AutoRep", koi.pack_auto_report, (True, 2500), 0xC4, [7,8], [0x80, 0x0B]),

        # --- D系列 ---
        ("BootMode", koi.pack_boot_mode, (RunMode.PLAY,), 0xD0, [9], [0x04]),
        # BootCPG same as A1 logic.
        ("BootCPG", koi.pack_cpg, (20, -5, 1.5, False, True), 0xD1, [7,8,9], [0xC8, 0x19, 0x1E]),
        # BootInt same as A2 logic.
        ("BootInt", koi.pack_intermittent, (5, 2000, True, True), 0xD2, [7,8,9], [0x50, 0x14, 0x01]),
        ("BootPlay", koi.pack_play_mode, (5, True), 0xD3, [9], [0x05]),

        # BootGear: D4. Spd=10, Turn=7, Int=True.
        # D4 logic: AA=0, BB=En(1), CC=Gear(A7).
        ("BootGear", koi.pack_gear_mode, (10, 7, True, True), 0xD4, [7,8,9], [0x00, 0x01, 0xA7]),

        ("BootLED", koi.pack_led, (LedMode.OFF, True), 0xD5, [9], [0x00]),
        ("BootReply", koi.pack_boot_reply, (True,), 0xD6, [9], [0x00]),
        ("Factory", koi.pack_factory_reset, (), 0xD7, [], []),
        ("QryBoot", koi.pack_query_boot, (), 0xD8, [], []),

        # InstallOff: S1=3(Pos), S2=-2(Neg).
        # S1: D=1, V=3 -> 1011(B). S2: D=0, V=2 -> 0010(2).
        # AA = (B<<4)|2 = B2.
        ("InstOff", koi.pack_install_offset, (3, -2), 0xD9, [7], [0xB2]),
        ("Flash", koi.pack_query_flash, (0x10,), 0xDF, [9], [0x10]),
    ]

    for name, func, args, exp_cmd, exp_idxs, exp_vals in tests:
        raw = func(*args) # 包含 LoRa 头 (3B) + 帧 (12B)
        payload = raw[3:] # 去掉 LoRa 头

        # Check Cmd
        cmd_byte = payload[6]
        status = "OK" if cmd_byte == exp_cmd else f"FAIL(Cmd {cmd_byte:02X}!={exp_cmd:02X})"

        # Check Data
        if status == "OK":
            for idx, val in zip(exp_idxs, exp_vals):
                if payload[idx] != val:
                    status = f"FAIL(Byte{idx} {payload[idx]:02X}!={val:02X})"
                    break

        print(f"{name:<15} | {str(func.__name__):<20} | {hex_str(raw):<40} | {status}")

    print("\n=== Unpack Test ===")
    # 模拟 F4 电压返回: AA=50(0.5V+3.5=4.0), BB=03, CC=E8 (1000mW)
    sim_payload = bytearray([0xAA, 0x55, 0x00, 0x01, 0x00, 0x00, 0xF4, 50, 0x03, 0xE8, 0x00, 0x0D])
    sim_payload[10] = _xor_checksum(sim_payload)

    res = LoraProtocol.parse_response(sim_payload)
    print(f"F4 Response: {res}")
    if res["valid"] and res["volt"] == 4.0 and res["mw"] == 1000:
        print("Unpack Check: OK")
    else:
        print("Unpack Check: FAIL")
