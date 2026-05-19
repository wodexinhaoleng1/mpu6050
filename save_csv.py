"""
save_csv.py - PC 端控制 MCU 采集并保存 CSV

用法:
    python save_csv.py [COM端口]  (不填则自动扫描)

工作流程:
    1. 脚本启动后占用串口，MCU 等待指令
    2. 在本脚本终端按 Enter -> 向 MCU 发送 Enter，开始采集
    3. 摇动设备
    4. 再次按 Enter -> 停止记录，CSV 自动保存到 ./data/
    5. 重复步骤 2-4，Ctrl+C 退出
"""

import sys
import os
import serial
import serial.tools.list_ports
import threading
import time
from datetime import datetime

BAUD = 115200
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")


def pick_port(arg=None):
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("[ERROR] 未找到串口设备，请检查连接。")
        sys.exit(1)
    if arg:
        return arg
    if len(ports) == 1:
        print(f"[INFO] 自动选择: {ports[0].device} — {ports[0].description}")
        return ports[0].device
    print("可用串口:")
    for i, p in enumerate(ports):
        print(f"  {i}: {p.device}  {p.description}")
    idx = int(input("请输入端口编号: ").strip())
    return ports[idx].device


class SerialReader(threading.Thread):
    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lines = []
        self.lock = threading.Lock()
        self._stop = False

    def run(self):
        while not self._stop:
            try:
                raw = self.ser.readline()
            except Exception:
                break
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
            if line:
                print(f"  {line}")
                with self.lock:
                    self.lines.append(line)

    def flush_lines(self):
        with self.lock:
            snap = list(self.lines)
            self.lines.clear()
        return snap

    def stop(self):
        self._stop = True


def save_csv(lines, session_num):
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    fname = os.path.join(OUTPUT_DIR, f"session_{session_num:03d}_{ts}.csv")
    header = None
    rows = []
    for line in lines:
        if line.startswith("session_id,"):
            header = line
        elif not line.startswith("#") and header:
            rows.append(line)
    if not header or not rows:
        print("[PC] 本次无有效数据，未保存。")
        return
    with open(fname, "w", encoding="utf-8", newline="") as f:
        f.write(header + "\n")
        for r in rows:
            f.write(r + "\n")
    print(f"[PC] 已保存: {fname}  ({len(rows)} 行数据)")


def main():
    port = pick_port(sys.argv[1] if len(sys.argv) > 1 else None)
    print(f"[INFO] 连接 {port} @ {BAUD} baud ...")
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.5)

    reader = SerialReader(ser)
    reader.start()

    session_num = 0
    print("\n=========================================")
    print(" 按 Enter 开始采集，再按 Enter 停止并保存，Ctrl+C 退出")
    print("=========================================\n")

    try:
        while True:
            input(f">> 按 Enter 开始第 {session_num + 1} 条记录...")
            session_num += 1
            reader.flush_lines()

            ser.write(b"\n")
            print("[PC] 采集中... 摇动完成后按 Enter 停止")

            input("")
            print("[PC] 正在保存...")

            ser.write(b"\n")
            time.sleep(0.3)

            lines = reader.flush_lines()
            save_csv(lines, session_num)
            print()

    except KeyboardInterrupt:
        print("\n[INFO] 用户退出。")
    finally:
        reader.stop()
        ser.close()
        print("[INFO] 串口已关闭。")


if __name__ == "__main__":
    main()
