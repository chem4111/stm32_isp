import serial
import time
import struct
import sys

ACK = 0x79
NACK = 0x1F

FLASH_BASE = 0x08000000
FLASH_END  = 0x080FFFFF   # 够覆盖 F1
WRITE_BLOCK = 256
TIMEOUT = 5
DEBUG = False


def xor_checksum(data: bytes):
    c = 0
    for b in data:
        c ^= b
    return c & 0xFF


def wait_ack(ser, name="", timeout=TIMEOUT):
    start = time.time()
    while time.time() - start < timeout:
        r = ser.read(1)
        if not r:
            continue
        if r[0] == ACK:
            return
        if r[0] == NACK:
            raise RuntimeError(f"NACK after {name}")
    raise RuntimeError(f"Timeout waiting ACK after {name}")


# -------- ISP 控制 --------

def enter_isp(ser):
    print("[*] Enter ISP")
    ser.dtr = False   # NRST low
    time.sleep(0.05)
    ser.rts = True    # BOOT0 high
    time.sleep(0.05)
    ser.dtr = True    # NRST release
    time.sleep(0.3)


def exit_isp(ser):
    """退出 ISP 模式（保持 BootLoader 模式）"""
    print("[*] 退出 ISP")
    ser.rts = False
    time.sleep(0.05)
    ser.dtr = False
    time.sleep(0.05)
    ser.dtr = True


def reset_and_run(ser):
    """复位芯片并运行程序（模仿 MCUISP 流程）"""
    print("[*] 复位并运行程序")
    # MCUISP 的复位时序：
    # 1. RTS电平置高(+3-+12V), DTR置高(+3-+12V) - 选择BootLoader并保持复位
    ser.rts = True    # BOOT0 高
    ser.dtr = True    # NRST 高（保持复位）
    time.sleep(0.1)   # 延时100毫秒
    
    # 2. RTS电平变低(-3--12V)释放复位
    ser.dtr = False   # NRST 低（释放复位）
    time.sleep(0.05)
    ser.rts = False   # BOOT0 低（从 Flash 启动）


def sync(ser):
    print("[*] Sync 0x7F")
    ser.reset_input_buffer()
    ser.write(b'\x7F')
    wait_ack(ser, "sync")


# -------- Bootloader 指令 --------

def send_cmd(ser, cmd):
    ser.write(bytes([cmd, cmd ^ 0xFF]))
    wait_ack(ser, f"cmd 0x{cmd:02X}")


def send_addr(ser, addr):
    b = struct.pack(">I", addr)
    ser.write(b + bytes([xor_checksum(b)]))
    wait_ack(ser, "address")


def write_mem(ser, addr, data):
    send_cmd(ser, 0x31)
    send_addr(ser, addr)
    ln = len(data) - 1
    pkt = bytes([ln]) + data
    ser.write(pkt + bytes([xor_checksum(pkt)]))
    wait_ack(ser, "write")


def erase_all(ser):
    print("[*] Global erase")
    send_cmd(ser, 0x43)
    ser.write(b'\xFF\x00')
    wait_ack(ser, "erase", timeout=10)
    print("[+] Erase done")


# -------- HEX 解析（工程级）--------

def parse_hex_segments(path):
    segments = []
    base = 0
    mem = {}

    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line.startswith(":"):
                continue

            ln = int(line[1:3], 16)
            addr = int(line[3:7], 16)
            typ = int(line[7:9], 16)
            data = bytes.fromhex(line[9:9 + ln * 2])

            if typ == 0x00:
                for i, b in enumerate(data):
                    mem[base + addr + i] = b

            elif typ == 0x04:
                base = int(line[9:13], 16) << 16

            elif typ == 0x01:
                break

    if not mem:
        raise RuntimeError("HEX has no data")

    addrs = sorted(mem.keys())
    start = addrs[0]
    buf = bytearray()
    last = start

    for a in addrs:
        if a != last:
            segments.append((start, bytes(buf)))
            start = a
            buf = bytearray()
        buf.append(mem[a])
        last = a + 1

    segments.append((start, bytes(buf)))
    return segments


# -------- 写入流程 --------

def write_segments(ser, segments):
    total = sum(len(d) for _, d in segments)
    done = 0

    print("[*] Writing firmware")
    for addr, data in segments:
        if addr < FLASH_BASE or addr + len(data) > FLASH_END:
            raise RuntimeError(f"HEX address out of flash: 0x{addr:08X}")

        for off in range(0, len(data), WRITE_BLOCK):
            chunk = data[off:off + WRITE_BLOCK]
            if len(chunk) < WRITE_BLOCK:
                chunk += b'\xFF' * (WRITE_BLOCK - len(chunk))
            write_mem(ser, addr + off, chunk)
            done += len(chunk)
            percent = min(100, int(done * 100 / total))  # 确保不超过100%
            sys.stdout.write(f"\rDownloading... {percent}%")
            sys.stdout.flush()

    print("\n[+] Download complete")


# -------- main --------

def main():
    if len(sys.argv) < 4:
        print("Usage:")
        print("  python stm32_isp.py COMx 115200 clear")
        print("  python stm32_isp.py COMx 115200 download app.bin/app.hex")
        return

    port = sys.argv[1]
    baud = int(sys.argv[2])
    cmd  = sys.argv[3]

    ser = serial.Serial(
        port=port,
        baudrate=baud,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_EVEN,
        stopbits=serial.STOPBITS_ONE,
        timeout=TIMEOUT
    )

    print(f"[*] Open {port} @ {baud}")
    enter_isp(ser)
    sync(ser)

    if cmd == "clear":
        erase_all(ser)

    elif cmd == "download":
        fw = sys.argv[4]
        erase_all(ser)

        if fw.lower().endswith(".hex"):
            segments = parse_hex_segments(fw)
        else:
            with open(fw, "rb") as f:
                data = f.read()
            segments = [(FLASH_BASE, data)]

        write_segments(ser, segments)
        print("[+] 下载完成，自动复位运行程序")
        # 下载完成后执行完整的复位流程
        reset_and_run(ser)

    else:
        print("Unknown command")

    exit_isp(ser)
    ser.close()


if __name__ == "__main__":
    main()
