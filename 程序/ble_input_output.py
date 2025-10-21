import asyncio
import sys
import struct
import time
import csv
import os
import hashlib
from typing import Dict, List, Optional, Tuple
from collections import deque
from bleak import BleakClient, BleakScanner

# Windows: 非阻塞键盘检测
try:
    import msvcrt
    HAS_MSVCRT = True
except ImportError:
    HAS_MSVCRT = False

# 监听文件默认路径
DEFAULT_WATCH_FILE = os.path.join(os.path.dirname(__file__), "targets.csv")

# 与 ESP32 相同的 UUID
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID_RX = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # ESP32接收
CHARACTERISTIC_UUID_TX = "6d68efe5-04b6-4a85-abc4-c2670b7bf7fd"  # ESP32发送

# 包类型定义
# 顶部常量处
PACKET_TYPE_SINGLE = 0x01
PACKET_TYPE_MULTI = 0x02
PACKET_TYPE_MULTI_STRUCT = 0x03  # 新增：结构体化MULTI

class MultiBLEInputOutput:
    def __init__(self, max_devices: int = 50, watch_file_path: Optional[str] = DEFAULT_WATCH_FILE, poll_interval: float = 0.5):
        self.max_devices = max_devices
        self.watch_file_path = watch_file_path
        self.poll_interval = poll_interval

        self.clients: Dict[str, BleakClient] = {}
        self.device_responses: Dict[str, List[dict]] = {}
        self.device_status: Dict[str, dict] = {}
        self.id_to_address: Dict[int, str] = {}
        self.connected_count = 0

        self.shutting_down = False
        self.device_buffers: Dict[int, dict] = {}
        self.max_device_id: int = 0

        self._last_file_hash: Optional[str] = None

    def notification_handler(self, device_address):
        def handler(sender, data):
            try:
                if self.shutting_down:
                    return
                message = data.decode('utf-8', errors='ignore')
                print(f"📨 [{device_address}] 响应: {message}")

                # 从 "id:..." 响应中建立 ID 映射
                if ":" in message:
                    head = message.split(':')[0]
                    if head.isdigit():
                        dev_id = int(head)
                        self.id_to_address[dev_id] = device_address

                # 更新在线状态与最后活动时间
                if device_address in self.device_status:
                    self.device_status[device_address]['last_activity'] = time.time()
                    self.device_status[device_address]['is_online'] = True

                # 存储响应
                self.device_responses.setdefault(device_address, []).append({
                    'timestamp': time.time(),
                    'message': message
                })
            except Exception as e:
                print(f"❌ [{device_address}] 解码错误: {e}")
        return handler

    async def scan_esp32_devices(self, timeout: int = 10) -> List[dict]:
        print(f"🔍 扫描ESP32设备，超时时间: {timeout}秒")
        devices: List[dict] = []
        try:
            scanned_devices = await BleakScanner.discover(timeout=timeout)
            for device in scanned_devices:
                if device.name and any(k in device.name for k in ["ESP32", "DengFOC", "DFOC", "Motor", "Controller"]):
                    print(f"🎯 发现ESP32设备: {device.name} - {device.address}")
                    devices.append({'address': device.address, 'name': device.name})
            if not devices:
                print("⚠️ 未发现ESP32设备，展示可见设备：")
                for device in scanned_devices:
                    if device.name:
                        print(f"   {device.name} - {device.address}")
        except Exception as e:
            print(f"❌ 扫描失败: {e}")
        return devices

    async def connect_to_devices(self, devices: List[dict]) -> List[str]:
        connected: List[str] = []
        for info in devices[:self.max_devices]:
            addr = info['address']
            try:
                client = BleakClient(addr)
                await client.connect()
                await client.start_notify(CHARACTERISTIC_UUID_TX, self.notification_handler(addr))
                self.clients[addr] = client
                self.device_status[addr] = {
                    'is_online': True,
                    'last_activity': time.time(),
                    'name': info.get('name', 'Unknown')
                }
                self.device_responses[addr] = []
                self.connected_count += 1
                connected.append(addr)
                print(f"✅ 已连接: {addr}")
            except Exception as e:
                print(f"❌ 连接 {addr} 失败: {e}")
        return connected

    async def disconnect_all(self):
        tasks = []
        for addr, client in list(self.clients.items()):
            try:
                tasks.append(asyncio.create_task(client.disconnect()))
            except Exception:
                pass
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)
        self.clients.clear()
        self.device_responses.clear()
        self.device_status.clear()
        print("🔌 已断开所有设备连接")

    def create_multi_slice_packet(self, start_id: int, values: List[float], data_type: int) -> bytearray:
        packet = bytearray()
        packet.extend([0xAA, 0x55])
        packet.append(PACKET_TYPE_MULTI)
        packet.append(data_type)
        packet.append(start_id)
        packet.append(len(values))
        for v in values:
            scaled = int(v * 10.0)
            packet.extend(struct.pack('>h', scaled))
        return packet

    def create_multi_struct_packet(self, items: List[Tuple[int, float]], data_type: int) -> bytearray:
        packet = bytearray()
        packet.extend([0xAA, 0x55])
        packet.append(PACKET_TYPE_MULTI_STRUCT)
        packet.append(data_type)
        packet.append(len(items))
        for dev_id, v in items:
            scaled = int(v * 10.0)
            packet.append(dev_id & 0xFF)
            packet.extend(struct.pack('>h', scaled))
        return packet

    async def send_broadcast_data(self, packet_data: bytes):
        tasks = []
        for addr, client in self.clients.items():
            tasks.append(asyncio.create_task(client.write_gatt_char(CHARACTERISTIC_UUID_RX, packet_data)))
        await asyncio.gather(*tasks, return_exceptions=True)
        print(f"📤 广播到 {len(self.clients)} 台设备")

    def ensure_buffers(self, max_id: int):
        if max_id > self.max_device_id:
            for i in range(self.max_device_id + 1, max_id + 1):
                self.device_buffers[i] = {"device_id": i, "queue": deque(), "last_value": 0.0}
            self.max_device_id = max_id

    def load_id_values_dict(self, id_values: Dict[int, float]):
        if not id_values:
            print("❌ 输入字典为空")
            return
        self.ensure_buffers(max(id_values.keys()))
        loaded = 0
        for device_id, val in id_values.items():
            if device_id < 1:
                continue
            buf = self.device_buffers[device_id]
            buf["queue"].append(val)
            buf["last_value"] = val
            loaded += 1
        print(f"📥 已加载 {loaded} 条(ID→值)，最大ID={self.max_device_id}")

    async def broadcast_buffers_multi_rounds(
        self,
        group_size: int = 5,
        data_type: int = 0x01,
        per_device_hz: Optional[float] = None,
        max_rounds: Optional[int] = None,
        use_struct: bool = False
    ):
        if self.max_device_id == 0:
            print("❌ 尚未加载任何设备数据")
            return

        group_count = (self.max_device_id + group_size - 1) // group_size
        slot_seconds = 0.0
        if per_device_hz and per_device_hz > 0:
            slot_seconds = 1.0 / (per_device_hz * group_count)
            print(f"⏱️ 分组轮询: max_id={self.max_device_id}, group_size={group_size}, groups={group_count}, per_device_hz={per_device_hz}, slot={slot_seconds*1000:.2f}ms")
        else:
            print(f"📤 分组发送（按队列耗尽）：max_id={self.max_device_id}, group_size={group_size}, groups={group_count}")

        rounds = 0
        while True:
            any_data = False
            for g in range(group_count):
                start_id = g * group_size + 1
                end_id = min(start_id + group_size - 1, self.max_device_id)
                slice_vals: List[float] = []
                items: List[Tuple[int, float]] = []
                for dev_id in range(start_id, end_id + 1):
                    buf = self.device_buffers.get(dev_id)
                    if not buf:
                        val = 0.0
                    else:
                        q = buf["queue"]
                        if q:
                            val = q.popleft()
                            buf["last_value"] = val
                            any_data = True
                        else:
                            val = buf["last_value"]
                    slice_vals.append(val)
                    items.append((dev_id, val))
                packet = (
                    self.create_multi_struct_packet(items, data_type)
                    if use_struct
                    else self.create_multi_slice_packet(start_id, slice_vals, data_type)
                )
                await self.send_broadcast_data(packet)
                if slot_seconds > 0:
                    await asyncio.sleep(slot_seconds)

            rounds += 1
            if max_rounds is not None and rounds >= max_rounds:
                print(f"✅ 达到最大轮次 {max_rounds}，停止")
                break
            if not any_data:
                print("✅ 所有队列已耗尽，停止")
                break

    async def wait_for_responses(self, timeout: float = 5):
        start = time.time()
        print(f"⏳ 等待响应，最多 {timeout}s")
        while time.time() - start < timeout:
            await asyncio.sleep(0.2)

    def _compute_file_hash(self, path: str) -> Optional[str]:
        try:
            with open(path, 'rb') as f:
                data = f.read()
            return hashlib.sha256(data).hexdigest()
        except Exception:
            return None

    def _parse_id_values_and_config(self, path: str) -> Tuple[Dict[int, float], int, Optional[float], Optional[int], int, bool]:
        id_values: Dict[int, float] = {}
        group_size = 5
        per_device_hz: Optional[float] = None
        max_rounds: Optional[int] = None
        data_type = 0x01
        use_struct = False
        ext = os.path.splitext(path)[1].lower()
        try:
            if ext == ".csv":
                with open(path, newline='', encoding='utf-8') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if not row or len(row) < 2:
                            continue
                        key = row[0].strip()
                        val = row[1].strip()
                        if key.isdigit():
                            try:
                                id_values[int(key)] = float(val)
                            except ValueError:
                                pass
                        else:
                            k = key.lower()
                            if k == "group_size":
                                try: group_size = int(val)
                                except: pass
                            elif k == "per_device_hz":
                                try: per_device_hz = float(val)
                                except: per_device_hz = None
                            elif k == "max_rounds":
                                try: max_rounds = int(val)
                                except: max_rounds = None
                            elif k == "data_type":
                                vs = val.strip().lower()
                                if vs == "velocity":
                                    data_type = 0x02
                                elif vs == "current":
                                    data_type = 0x03
                                else:
                                    data_type = 0x01
                            elif k in ("packet_mode", "packet_type"):
                                pt = val.strip().lower()
                                use_struct = pt in ("struct", "multi_struct", "03", "0x03")
            else:
                with open(path, encoding='utf-8') as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        parts = [p.strip() for p in (line.split(',') if ',' in line else line.split())]
                        if len(parts) < 2:
                            continue
                        key, val = parts[0], parts[1]
                        if key.isdigit():
                            try:
                                id_values[int(key)] = float(val)
                            except ValueError:
                                pass
                        else:
                            k = key.lower()
                            if k == "group_size":
                                try: group_size = int(val)
                                except: pass
                            elif k == "per_device_hz":
                                try: per_device_hz = float(val)
                                except: per_device_hz = None
                            elif k == "max_rounds":
                                try: max_rounds = int(val)
                                except: max_rounds = None
                            elif k == "data_type":
                                vs = val.strip().lower()
                                if vs == "velocity":
                                    data_type = 0x02
                                elif vs == "current":
                                    data_type = 0x03
                                else:
                                    data_type = 0x01
                            elif k in ("packet_mode", "packet_type"):
                                pt = val.strip().lower()
                                use_struct = pt in ("struct", "multi_struct", "03", "0x03")
        except Exception as e:
            print(f"❌ 读取文件失败: {e}")
        return id_values, group_size, per_device_hz, max_rounds, data_type, use_struct

    async def watch_file_and_broadcast_loop(self):
        if not self.watch_file_path:
            print("❌ 未设置监听文件路径")
            return
        print(f"👀 开始监听文件: {self.watch_file_path}")
        while not self.shutting_down:
            # ESC 退出
            if HAS_MSVCRT and msvcrt.kbhit():
                ch = msvcrt.getch()
                if ch in (b'\x1b',):  # ESC
                    print("🛑 检测到 ESC，退出监听")
                    break

            if not os.path.isfile(self.watch_file_path):
                await asyncio.sleep(self.poll_interval)
                continue

            current_hash = self._compute_file_hash(self.watch_file_path)
            if current_hash and current_hash != self._last_file_hash:
                self._last_file_hash = current_hash
                print("📄 检测到文件更新，读取并发送...")
                id_values, group_size, per_device_hz, max_rounds, data_type, use_struct = self._parse_id_values_and_config(self.watch_file_path)
                self.load_id_values_dict(id_values)
                await self.broadcast_buffers_multi_rounds(
                    group_size=group_size,
                    data_type=data_type,
                    per_device_hz=per_device_hz,
                    max_rounds=max_rounds,
                    use_struct=use_struct
                )
                await self.wait_for_responses()
            await asyncio.sleep(self.poll_interval)

    async def run(self):
        devices = await self.scan_esp32_devices()
        if not devices:
            print("❌ 未找到ESP32设备")
            return
        connected = await self.connect_to_devices(devices)
        if not connected:
            print("❌ 没有成功连接的设备")
            return

        print("\n" + "="*50)
        print("自动文件监听与广播（按 ESC 退出）")
        print("="*50)

        try:
            await self.watch_file_and_broadcast_loop()
        finally:
            await self.disconnect_all()


async def main():
    # Python 版本检查
    if sys.version_info < (3, 7):
        print("❌ 需要Python 3.7或更高版本")
        return
    io = MultiBLEInputOutput(max_devices=50, watch_file_path=DEFAULT_WATCH_FILE, poll_interval=0.5)
    await io.run()


if __name__ == "__main__":
    asyncio.run(main())