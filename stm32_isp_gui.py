import serial
import serial.tools.list_ports
import time
import struct
import threading
import tkinter as tk
from tkinter import ttk, filedialog, messagebox

ACK = 0x79
NACK = 0x1F

FLASH_BASE = 0x08000000
FLASH_END  = 0x080FFFFF
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
    ser.dtr = False
    time.sleep(0.05)
    ser.rts = True
    time.sleep(0.05)
    ser.dtr = True
    time.sleep(0.3)


def exit_isp(ser):
    ser.rts = False
    time.sleep(0.05)
    ser.dtr = False
    time.sleep(0.05)
    ser.dtr = True


def reset_and_run(ser):
    ser.rts = True
    ser.dtr = True
    time.sleep(0.1)
    ser.dtr = False
    time.sleep(0.05)
    ser.rts = False


def sync(ser):
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
    send_cmd(ser, 0x43)
    ser.write(b'\xFF\x00')
    wait_ack(ser, "erase", timeout=10)


# -------- HEX 解析 --------

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


# -------- GUI 应用 --------

class STM32ISPApp:
    def __init__(self, root):
        self.root = root
        self.root.title("STM32 ISP 烧录工具")
        self.root.geometry("600x450")
        self.root.resizable(False, False)
        
        self.firmware_path = ""
        self.is_flashing = False
        
        self.create_widgets()
        self.refresh_ports()
    
    def create_widgets(self):
        # 主框架
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)
        
        # 串口选择
        port_frame = ttk.Frame(main_frame)
        port_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_frame, text="串口:").pack(side=tk.LEFT)
        self.port_combo = ttk.Combobox(port_frame, width=15, state="readonly")
        self.port_combo.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(port_frame, text="刷新", command=self.refresh_ports).pack(side=tk.LEFT)
        
        # 波特率
        ttk.Label(port_frame, text="波特率:").pack(side=tk.LEFT, padx=(20, 0))
        self.baud_combo = ttk.Combobox(port_frame, width=10, state="readonly")
        self.baud_combo['values'] = ['115200', '57600', '38400', '19200', '9600']
        self.baud_combo.set('115200')
        self.baud_combo.pack(side=tk.LEFT, padx=5)
        
        # 固件文件选择
        fw_frame = ttk.Frame(main_frame)
        fw_frame.pack(fill=tk.X, pady=10)
        
        ttk.Label(fw_frame, text="固件:").pack(side=tk.LEFT)
        self.fw_entry = ttk.Entry(fw_frame, width=40)
        self.fw_entry.pack(side=tk.LEFT, padx=5)
        self.fw_entry.config(state='readonly')
        
        ttk.Button(fw_frame, text="浏览...", command=self.browse_firmware).pack(side=tk.LEFT)
        
        # 按钮
        btn_frame = ttk.Frame(main_frame)
        btn_frame.pack(fill=tk.X, pady=10)
        
        self.flash_btn = ttk.Button(btn_frame, text="烧录", command=self.flash_firmware, width=15)
        self.flash_btn.pack(side=tk.LEFT, padx=5)
        
        self.clear_btn = ttk.Button(btn_frame, text="擦除", command=self.clear_flash, width=15)
        self.clear_btn.pack(side=tk.LEFT, padx=5)
        
        # 进度条
        self.progress = ttk.Progressbar(main_frame, mode='determinate', length=580)
        self.progress.pack(fill=tk.X, pady=5)
        
        # 日志输出
        log_frame = ttk.LabelFrame(main_frame, text="日志", padding="5")
        log_frame.pack(fill=tk.BOTH, expand=True)
        
        self.log_text = tk.Text(log_frame, height=12, width=70)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(log_frame, command=self.log_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text.config(yscrollcommand=scrollbar.set)
    
    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.set(ports[0])
    
    def browse_firmware(self):
        file_path = filedialog.askopenfilename(
            title="选择固件文件",
            filetypes=[("固件文件", "*.bin *.hex"), ("BIN文件", "*.bin"), ("HEX文件", "*.hex"), ("所有文件", "*.*")]
        )
        if file_path:
            self.firmware_path = file_path
            self.fw_entry.config(state='normal')
            self.fw_entry.delete(0, tk.END)
            self.fw_entry.insert(0, file_path)
            self.fw_entry.config(state='readonly')
    
    def log(self, message):
        self.log_text.insert(tk.END, message + "\n")
        self.log_text.see(tk.END)
    
    def set_ui_state(self, enabled):
        state = 'normal' if enabled else 'disabled'
        self.flash_btn.config(state=state)
        self.clear_btn.config(state=state)
        self.port_combo.config(state='readonly' if enabled else 'disabled')
        self.baud_combo.config(state='readonly' if enabled else 'disabled')
        self.is_flashing = not enabled
    
    def flash_firmware(self):
        if self.is_flashing:
            return
        
        if not self.firmware_path:
            messagebox.showerror("错误", "请先选择固件文件!")
            return
        
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("错误", "请选择串口!")
            return
        
        baud = int(self.baud_combo.get())
        
        self.progress['value'] = 0
        self.log("=" * 50)
        self.log(f"开始烧录: {self.firmware_path}")
        
        thread = threading.Thread(target=self._flash_thread, args=(port, baud), daemon=True)
        thread.start()
    
    def _flash_thread(self, port, baud):
        self.set_ui_state(False)
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT
            )
            
            self.log(f"打开串口 {port} @ {baud}")
            
            try:
                self.log("进入 ISP 模式...")
                enter_isp(ser)
                
                self.log("同步...")
                sync(ser)
                
                self.log("擦除 Flash...")
                erase_all(ser)
                self.log("擦除完成")
                
                # 解析固件
                if self.firmware_path.lower().endswith(".hex"):
                    segments = parse_hex_segments(self.firmware_path)
                else:
                    with open(self.firmware_path, "rb") as f:
                        data = f.read()
                    segments = [(FLASH_BASE, data)]
                
                # 写入固件
                total = sum(len(d) for _, d in segments)
                done = 0
                
                self.log(f"开始写入固件 ({total} 字节)...")
                
                for addr, data in segments:
                    if addr < FLASH_BASE or addr + len(data) > FLASH_END:
                        raise RuntimeError(f"HEX地址超出Flash范围: 0x{addr:08X}")
                    
                    for off in range(0, len(data), WRITE_BLOCK):
                        chunk = data[off:off + WRITE_BLOCK]
                        if len(chunk) < WRITE_BLOCK:
                            chunk += b'\xFF' * (WRITE_BLOCK - len(chunk))
                        write_mem(ser, addr + off, chunk)
                        done += len(chunk)
                        percent = min(100, int(done * 100 / total))
                        self.progress['value'] = percent
                        self.root.update_idletasks()
                
                self.log("下载完成!")
                self.log("复位并运行程序...")
                reset_and_run(ser)
                self.log("烧录成功!")
                
                messagebox.showinfo("成功", "固件烧录完成!")
                
            finally:
                exit_isp(ser)
                ser.close()
                
        except Exception as e:
            self.log(f"错误: {str(e)}")
            messagebox.showerror("错误", f"烧录失败:\n{str(e)}")
        finally:
            self.set_ui_state(True)
    
    def clear_flash(self):
        if self.is_flashing:
            return
        
        port = self.port_combo.get()
        if not port:
            messagebox.showerror("错误", "请选择串口!")
            return
        
        baud = int(self.baud_combo.get())
        
        self.progress['value'] = 0
        self.log("=" * 50)
        self.log("开始擦除 Flash...")
        
        thread = threading.Thread(target=self._clear_thread, args=(port, baud), daemon=True)
        thread.start()
    
    def _clear_thread(self, port, baud):
        self.set_ui_state(False)
        try:
            ser = serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_EVEN,
                stopbits=serial.STOPBITS_ONE,
                timeout=TIMEOUT
            )
            
            self.log(f"打开串口 {port} @ {baud}")
            
            try:
                self.log("进入 ISP 模式...")
                enter_isp(ser)
                
                self.log("同步...")
                sync(ser)
                
                self.log("擦除 Flash...")
                erase_all(ser)
                self.log("擦除完成!")
                
                messagebox.showinfo("成功", "Flash擦除完成!")
                
            finally:
                exit_isp(ser)
                ser.close()
                
        except Exception as e:
            self.log(f"错误: {str(e)}")
            messagebox.showerror("错误", f"擦除失败:\n{str(e)}")
        finally:
            self.set_ui_state(True)


def main():
    root = tk.Tk()
    app = STM32ISPApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
