import asyncio
import sys
from bleak import BleakClient, BleakScanner
import struct
import time
from typing import Dict, List, Optional, Callable, Tuple
from collections import deque
import csv
import os
DEFAULT_WATCH_FILE = os.path.join(os.path.dirname(__file__), "targets.csv")

# 使用与ESP32相同的UUID
SERVICE_UUID = "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
CHARACTERISTIC_UUID_RX = "beb5483e-36e1-4688-b7f5-ea07361b26a8"  # ESP32接收
CHARACTERISTIC_UUID_TX = "6d68efe5-04b6-4a85-abc4-c2670b7bf7fd"  # ESP32发送

# 多电机控制包格式
PACKET_TYPE_SINGLE = 0x01    # 单电机控制包
PACKET_TYPE_MULTI = 0x02     # 多电机批量控制包
PACKET_TYPE_MULTI_STRUCT = 0x03  # 新增：结构体化MULTI

class MultiBLECommunicator:
    def __init__(self, max_devices=20):
        self.clients: Dict[str, BleakClient] = {}
        self.device_responses: Dict[str, List[dict]] = {}
        self.device_status: Dict[str, dict] = {}
        self.max_devices = max_devices
        self.connected_count = 0
        self.heartbeat_task = None
        self.is_monitoring = False
        # 新增：设备ID到地址的映射
        self.id_to_address: Dict[int, str] = {}
        self.shutting_down: bool = False  # 退出静音标志
        from collections import deque
        self.device_buffers: Dict[int, dict] = {}
        self.max_device_id: int = 0
        self.watch_file_path: Optional[str] = DEFAULT_WATCH_FILE
        self._watch_task: Optional[asyncio.Task] = None
    
    def notification_handler(self, device_address):
        def handler(sender, data):
            try:
                if self.shutting_down:
                    return
                message = data.decode('utf-8')
                print(f"📨 [{device_address}] 收到响应: {message}")
                # 解析"<id>:..."建立ID映射
                if ":" in message:
                    head = message.split(':')[0]
                    if head.isdigit():
                        dev_id = int(head)
                        self.id_to_address[dev_id] = device_address
                
                # 更新设备最后活动时间
                if device_address in self.device_status:
                    self.device_status[device_address]['last_activity'] = time.time()
                    self.device_status[device_address]['is_online'] = True
                
                # 存储响应消息
                if device_address not in self.device_responses:
                    self.device_responses[device_address] = []
                self.device_responses[device_address].append({
                    'timestamp': time.time(),
                    'message': message
                })
                
            except Exception as e:
                print(f"❌ [{device_address}] 解码错误: {e}")
        return handler
    
    async def scan_esp32_devices(self, timeout=10):
        """扫描ESP32设备"""
        print(f"🔍 扫描ESP32设备，超时时间: {timeout}秒")
        devices = []
        
        try:
            # 扫描BLE设备
            scanned_devices = await BleakScanner.discover(timeout=timeout)
            
            for device in scanned_devices:
                # 检查设备名称是否包含ESP32相关标识
                if device.name and any(keyword in device.name for keyword in ["ESP32", "DengFOC", "DFOC", "Motor", "Controller"]):
                    print(f"🎯 发现ESP32设备: {device.name} - {device.address}")
                    devices.append({
                        'address': device.address,
                        'name': device.name
                    })
            
            if not devices:
                print("⚠️ 未发现ESP32设备，尝试显示所有发现的设备:")
                for device in scanned_devices:
                    if device.name:
                        print(f"   {device.name} - {device.address}")
        
        except Exception as e:
            print(f"❌ 扫描设备失败: {e}")
        
        return devices
    
    async def connect_to_devices(self, devices):
        """连接多个设备"""
        connected_devices = []
        
        for device_info in devices[:self.max_devices]:
            device_address = device_info['address']
            
            try:
                # 连接设备
                client = BleakClient(device_address)
                await client.connect()
                
                # 启用通知
                await client.start_notify(CHARACTERISTIC_UUID_TX, self.notification_handler(device_address))
                
                # 存储客户端和状态信息
                self.clients[device_address] = client
                self.device_status[device_address] = {
                    'is_online': True,
                    'last_activity': time.time(),
                    'name': device_info.get('name', 'Unknown')
                }
                self.device_responses[device_address] = []
                
                connected_devices.append(device_address)
                self.connected_count += 1
                print(f"✅ 已连接设备: {device_address}")
                
            except Exception as e:
                print(f"❌ 连接设备 {device_address} 失败: {e}")
        
        # 启动心跳检测
        if connected_devices:
            self.is_monitoring = True
            self.heartbeat_task = asyncio.create_task(self.heartbeat_monitor())
        
        return connected_devices
    
    async def heartbeat_monitor(self):
        """心跳检测任务"""
        while self.is_monitoring:
            await asyncio.sleep(5)  # 每5秒检查一次
            
            current_time = time.time()
            offline_devices = []
            
            for device_address, status in self.device_status.items():
                if status.get('is_online', False):
                    last_activity = status.get('last_activity', 0)
                    if current_time - last_activity > 30:  # 30秒无响应认为离线
                        status['is_online'] = False
                        offline_devices.append(device_address)
                        print(f"⚠️ 设备 {device_address} 离线")
            
            # 处理离线设备
            for device_address in offline_devices:
                await self.handle_device_offline(device_address)
    
    async def handle_device_offline(self, device_address):
        """处理设备离线"""
        if device_address in self.clients:
            try:
                await self.clients[device_address].disconnect()
                del self.clients[device_address]
                self.connected_count -= 1
                print(f"🔌 已断开离线设备: {device_address}")
            except Exception as e:
                print(f"❌ 断开设备 {device_address} 失败: {e}")
    
    async def send_heartbeat(self):
        """发送心跳包"""
        heartbeat_data = b"HEARTBEAT"
        for device_address, client in self.clients.items():
            try:
                await client.write_gatt_char(CHARACTERISTIC_UUID_RX, heartbeat_data)
            except Exception as e:
                print(f"❌ 向设备 {device_address} 发送心跳包失败: {e}")
    
    async def check_connection_status(self):
        """检查连接状态"""
        print("\n" + "="*50)
        print("设备连接状态:")
        
        for device_address, status in self.device_status.items():
            online_status = "🟢 在线" if status.get('is_online', False) else "🔴 离线"
            last_activity = status.get('last_activity', 0)
            time_diff = time.time() - last_activity
            
            print(f"{online_status} [{device_address}] - {status.get('name', 'Unknown')}")
            if status.get('is_online', False):
                print(f"   最后活动: {time_diff:.1f}秒前")
        
        print(f"总计: {self.connected_count} 个设备连接")
        print("="*50)
    
    def create_single_packet(self, device_id, data_type, target_value):
        """创建单电机控制包"""
        # 包格式: AA 55 01 DT ID VH VL
        packet = bytearray()
        packet.extend([0xAA, 0x55])  # 帧头
        packet.append(PACKET_TYPE_SINGLE)  # 包类型
        packet.append(data_type)  # 数据类型
        packet.append(device_id)  # 设备ID
        
        # 目标值转换 (乘以10.0缩放系数)
        scaled_value = int(target_value * 10.0)
        packet.extend(struct.pack('>h', scaled_value))  # 2字节大端序
        
        return packet
    
    def create_multi_slice_packet(self, start_id: int, values: List[float], data_type: int):
        """创建新版切片式多电机控制包: AA 55 02 DT START_ID COUNT V(start)..V(end)"""
        packet = bytearray()
        packet.extend([0xAA, 0x55])
        packet.append(PACKET_TYPE_MULTI)
        packet.append(data_type)
        packet.append(start_id)            # START_ID (1-based)
        packet.append(len(values))         # COUNT
        for v in values:
            scaled_value = int(v * 10.0)   # 与ESP32端 ANGLE_SCALE 保持一致（10.0）
            packet.extend(struct.pack('>h', scaled_value))
        return packet

    def create_multi_struct_packet(self, items: List[Tuple[int, float]], data_type: int) -> bytearray:
        """结构体化多电机包: AA 55 03 DT COUNT | (ID,VALUE)*COUNT"""
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

    async def run_multi_slice_scheduler(
        self,
        total_devices: int = 20,
        group_size: int = 5,
        data_type: int = 0x01,
        per_device_hz: float = 50.0,
        value_fn: Optional[Callable[[int, float], float]] = None,
        runtime_seconds: Optional[float] = None
    ):
        """
        自动按分组和时隙轮询，广播切片式MULTI包，确保每台设备达到 per_device_hz 刷新频率。
        - total_devices: 总设备数（ID从1开始）
        - group_size: 每组设备数量（默认5）
        - data_type: 数据类型（0x01角度等）
        - per_device_hz: 每台设备的目标刷新频率（如20或50Hz）
        - value_fn(device_id, now): 返回该设备当前时刻的目标值（float）；默认0.0
        - runtime_seconds: 运行时长（秒）；None表示一直运行直到外部停止或shutting_down=True
        """
        if total_devices <= 0 or group_size <= 0 or per_device_hz <= 0:
            print("❌ 参数错误: total_devices/group_size/per_device_hz 必须为正数")
            return

        group_count = (total_devices + group_size - 1) // group_size
        slot_seconds = 1.0 / (per_device_hz * group_count)
        print(f"⏱️ 自动轮询: total={total_devices}, group_size={group_size}, groups={group_count}, "
              f"per_device_hz={per_device_hz}, slot={slot_seconds*1000:.2f}ms")

        if value_fn is None:
            def value_fn(device_id: int, now: float) -> float:
                return 0.0

        start_ts = time.time()
        try:
            while not self.shutting_down:
                now = time.time()
                if runtime_seconds is not None and (now - start_ts) >= runtime_seconds:
                    print("⏹️ 自动轮询到达设定运行时长，停止")
                    break

                for g in range(group_count):
                    if self.shutting_down:
                        break
                    start_id = g * group_size + 1
                    end_id = min(start_id + group_size - 1, total_devices)
                    values: List[float] = [value_fn(dev_id, time.time()) for dev_id in range(start_id, end_id + 1)]
                    packet = self.create_multi_slice_packet(start_id, values, data_type)
                    await self.send_broadcast_data(packet)
                    await asyncio.sleep(slot_seconds)
        except asyncio.CancelledError:
            print("⏹️ 自动轮询被取消")
        except Exception as e:
            print(f"❌ 自动轮询异常: {e}")
        finally:
            print("✅ 自动轮询结束")
    
    async def send_broadcast_data(self, packet_data):
        """向所有连接设备发送数据（广播）"""
        tasks = []
        for device_address, client in self.clients.items():
            task = asyncio.create_task(client.write_gatt_char(CHARACTERISTIC_UUID_RX, packet_data))
            tasks.append(task)
        
        await asyncio.gather(*tasks, return_exceptions=True)
        print(f"📤 向 {len(self.clients)} 个设备广播数据")
    
    def ensure_buffers(self, max_id: int):
        """确保为1..max_id创建设备缓冲结构体。"""
        from collections import deque
        if max_id > self.max_device_id:
            for i in range(self.max_device_id + 1, max_id + 1):
                self.device_buffers[i] = {
                    "device_id": i,
                    "queue": deque(),
                    "last_value": 0.0,
                }
            self.max_device_id = max_id

    def load_id_values_dict(self, id_values: Dict[int, float]):
        """将字典{id: value}存入对应ID队列，并更新last_value。"""
        if not id_values:
            print("❌ 输入字典为空")
            return
        max_id = max(id_values.keys())
        self.ensure_buffers(max_id)
        loaded = 0
        for device_id, val in id_values.items():
            if device_id < 1:
                continue
            buf = self.device_buffers[device_id]
            buf["queue"].append(val)
            buf["last_value"] = val
            loaded += 1
        print(f"📥 已加载 {loaded} 条(ID→值)，最大ID={self.max_device_id}")

    def load_id_values_from_file(self, path: str, delimiter: Optional[str] = None):
        """从CSV或TXT读取: 每行 'id,value' 或 'id value'，跳过非数字行。"""
        if not path:
            print("❌ 未设置文件路径")
            return
        if not os.path.isfile(path):
            print(f"❌ 文件不存在: {path}")
            return
        ext = os.path.splitext(path)[1].lower()
        id_values: Dict[int, float] = {}
        try:
            if ext == ".csv":
                with open(path, newline='', encoding='utf-8') as f:
                    reader = csv.reader(f)
                    for row in reader:
                        if not row or len(row) < 2:
                            continue
                        id_str = row[0].strip()
                        val_str = row[1].strip()
                        if not id_str.isdigit():
                            continue
                        device_id = int(id_str)
                        try:
                            val = float(val_str)
                        except ValueError:
                            continue
                        id_values[device_id] = val
            else:
                with open(path, encoding='utf-8') as f:
                    for line in f:
                        line = line.strip()
                        if not line:
                            continue
                        parts = []
                        if delimiter:
                            parts = [p.strip() for p in line.split(delimiter)]
                        else:
                            parts = [p.strip() for p in (line.split(',') if ',' in line else line.split())]
                        if len(parts) < 2 or not parts[0].isdigit():
                            continue
                        device_id = int(parts[0])
                        try:
                            val = float(parts[1])
                        except ValueError:
                            continue
                        id_values[device_id] = val
        except Exception as e:
            print(f"❌ 读取文件失败: {e}")
            return
        self.load_id_values_dict(id_values)

    async def broadcast_buffers_multi_rounds(
        self,
        group_size: int = 5,
        data_type: int = 0x01,
        per_device_hz: Optional[float] = None,
        max_rounds: Optional[int] = None,
        use_struct: bool = False
    ):
        """分多次按缓冲内容广播：切片式或结构体化 MULTI。"""
        if self.max_device_id == 0:
            print("❌ 尚未加载任何设备数据")
            return

        group_count = (self.max_device_id + group_size - 1) // group_size
        slot_seconds = 0.0
        if per_device_hz and per_device_hz > 0:
            slot_seconds = 1.0 / (per_device_hz * group_count)
            print(f"⏱️ 计划分组轮询：max_id={self.max_device_id}, group_size={group_size}, groups={group_count}, "
                  f"per_device_hz={per_device_hz}, slot={slot_seconds*1000:.2f}ms")
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
                for device_id in range(start_id, end_id + 1):
                    buf = self.device_buffers.get(device_id)
                    if buf is None:
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
                    items.append((device_id, val))

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

    async def watch_file_and_broadcast(
        self,
        path: Optional[str] = None,
        group_size: int = 5,
        data_type: int = 0x01,
        per_device_hz: Optional[float] = None,
        poll_interval: float = 0.5,
        delimiter: Optional[str] = None,
        use_struct: bool = False
    ):
        """监听文件变更，自动读取ID→值并分组广播（可切片/结构体化）。"""
        target_path = path or self.watch_file_path
        if not target_path or not os.path.isfile(target_path):
            print(f"❌ 文件不存在: {target_path}")
            return
        print(f"👀 开始监听文件: {target_path}")
        last_mtime = 0.0
        while not self.shutting_down:
            try:
                mtime = os.path.getmtime(target_path)
                if mtime != last_mtime:
                    last_mtime = mtime
                    print("📄 检测到文件更新，准备读取并发送...")
                    self.load_id_values_from_file(target_path, delimiter=delimiter)
                    await self.broadcast_buffers_multi_rounds(
                        group_size=group_size,
                        data_type=data_type,
                        per_device_hz=per_device_hz,
                        max_rounds=1 if per_device_hz is None else None,
                        use_struct=use_struct
                    )
                    await self.wait_for_responses()
                await asyncio.sleep(poll_interval)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"❌ 监听错误: {e}")
                await asyncio.sleep(poll_interval)
        print("🛑 文件监听已停止")

    def start_auto_watch(self, group_size: int = 5, data_type: int = 0x01, per_device_hz: Optional[float] = None, use_struct: bool = False):
        """依据预设路径启动监听任务（后台自动运行）。"""
        if self._watch_task:
            return
        if not self.watch_file_path:
            print("❌ 未设置监听文件路径")
            return
        self._watch_task = asyncio.create_task(
            self.watch_file_and_broadcast(
                path=self.watch_file_path,
                group_size=group_size,
                data_type=data_type,
                per_device_hz=per_device_hz,
                poll_interval=0.5,
                delimiter=None,
                use_struct=use_struct
            )
        )
        print(f"✅ 已启动自动监听任务: {self.watch_file_path}")
    
    async def wait_for_responses(self, timeout=10, target_device_id=None):
        """等待设备响应（支持指定目标设备ID）"""
        print(f"⏳ 等待响应，超时时间: {timeout}秒")
        start_time = time.time()
        
        # 如果指定了目标设备ID，只等待该设备的响应
        if target_device_id is not None:
            print(f"🎯 等待设备ID {target_device_id} 的响应")
        
        while time.time() - start_time < timeout:
            # 检查响应情况
            if target_device_id is not None:
                # 单电机控制：只检查目标设备的响应
                target_device_responded = False
                for device_address, responses in self.device_responses.items():
                    # 检查是否有在发送命令后的响应，并且响应包含目标设备ID
                    recent_responses = [r for r in responses if r['timestamp'] > start_time - 1]
                    for response in recent_responses:
                        if f"{target_device_id}:" in response['message']:
                            target_device_responded = True
                            print(f"✅ 设备ID {target_device_id} 已响应: {response['message']}")
                            break
                    if target_device_responded:
                        break
                
                if target_device_responded:
                    break
                else:
                    # 检查是否有其他设备的错误响应
                    other_device_responded = False
                    for device_address, responses in self.device_responses.items():
                        recent_responses = [r for r in responses if r['timestamp'] > start_time - 1]
                        for response in recent_responses:
                            # 如果有其他设备的响应，但不是目标设备的
                            if ":" in response['message'] and "HEARTBEAT" not in response['message']:
                                try:
                                    resp_device_id = int(response['message'].split(':')[0])
                                    if resp_device_id != target_device_id:
                                        other_device_responded = True
                                        print(f"⚠️ 收到设备ID {resp_device_id} 的响应，但不是目标设备 {target_device_id}")
                                except ValueError:
                                    pass
                    
            else:
                # 多电机控制：检查所有设备的响应
                responded_count = 0
                for device_address, responses in self.device_responses.items():
                    recent_responses = [r for r in responses if r['timestamp'] > start_time - 1]
                    if recent_responses:
                        responded_count += 1
                
                if responded_count > 0:
                    print(f"✅ {responded_count} 个设备已响应")
                    break
            
            await asyncio.sleep(0.1)
        
        # 打印响应统计
        self.print_response_summary(target_device_id=target_device_id)
    
    def print_response_summary(self, expected_devices=None, target_device_id=None):
        """打印响应统计（支持指定目标设备ID）"""
        print("\n" + "="*50)
        print("响应统计:")
        
        if expected_devices is None:
            expected_devices = list(self.device_status.keys())
        
        target_device_found = False
        
        for device_address in expected_devices:
            status = self.device_status.get(device_address, {})
            responses = self.device_responses.get(device_address, [])
            
            if status.get('is_online', False):
                if responses:
                    latest_response = responses[-1]
                    response_time = time.time() - latest_response['timestamp']
                    if response_time < 10:  # 10秒内的响应认为是有效的
                        # 检查是否为目标设备的响应
                        if target_device_id is not None:
                            try:
                                resp_device_id = int(latest_response['message'].split(':')[0])
                                if resp_device_id == target_device_id:
                                    print(f"🟢 [{device_address}] 在线 - 目标设备响应: {latest_response['message']}")
                                    target_device_found = True
                                else:
                                    print(f"🟡 [{device_address}] 在线 - 非目标设备响应: {latest_response['message']}")
                            except ValueError:
                                print(f"🟡 [{device_address}] 在线 - 最新响应: {latest_response['message']}")
                        else:
                            print(f"🟢 [{device_address}] 在线 - 最新响应: {latest_response['message']}")
                    else:
                        print(f"🟡 [{device_address}] 在线 - 响应较旧 ({response_time:.1f}秒前)")
                else:
                    print(f"🟡 [{device_address}] 在线 - 无响应")
            else:
                print(f"🔴 [{device_address}] 离线")
        
        # 单电机控制时，检查目标设备是否响应
        if target_device_id is not None and not target_device_found:
            print(f"❌ 设备ID {target_device_id} 未响应（可能设备不存在或未连接）")
        
        print("="*50)
    
    async def disconnect_all(self):
        """断开所有连接（静音退出）"""
        self.shutting_down = True
        self.is_monitoring = False
        if self.heartbeat_task:
            self.heartbeat_task.cancel()

        tasks = []
        for device_address, client in self.clients.items():
            try:
                await client.stop_notify(CHARACTERISTIC_UUID_TX)
            except Exception:
                pass
            task = asyncio.create_task(client.disconnect())
            tasks.append(task)

        await asyncio.gather(*tasks, return_exceptions=True)
        self.clients.clear()
        self.device_responses.clear()
        self.device_status.clear()
        print("🔌 已断开所有设备连接")
    
    async def send_to_single_device(self, device_address, packet_data):
        """向指定设备发送数据（单设备发送）"""
        if device_address not in self.clients:
            print(f"❌ 设备 {device_address} 未连接")
            return False
        
        try:
            client = self.clients[device_address]
            await client.write_gatt_char(CHARACTERISTIC_UUID_RX, packet_data)
            print(f"📤 向设备 {device_address} 发送单电机控制数据")
            return True
        except Exception as e:
            print(f"❌ 发送数据到设备 {device_address} 失败: {e}")
            return False

async def multi_device_demo():
    communicator = MultiBLECommunicator(max_devices=50)
    try:
        # 扫描设备
        devices = await communicator.scan_esp32_devices()
        if not devices:
            print("❌ 未找到ESP32设备")
            return

        print(f"🎯 发现 {len(devices)} 个ESP32设备")
        # 连接设备
        connected_devices = await communicator.connect_to_devices(devices)
        if not connected_devices:
            print("❌ 没有成功连接的设备")
            return

        print("\n" + "="*50)
        print("多设备BLE通信测试开始（带断电检测）")
        print("输入 'status' 查看设备状态")
        print("输入 'quit' 退出程序")
        print("="*50)

        while True:
            
            print("\n选择操作:")
            print("0. 退出程序")
            print("1. 单电机控制（指定设备ID）")
            print("2. 多电机批量控制（切片MULTI）")
            print("3. 检查设备状态")
            print("4. 自动分组一次性发送（切片MULTI）")
            print("5. 自动执行（从文件读取并一键发送）")
            print("6. 发送结构体数据（MULTI_STRUCT）")
            choice = input("请输入选择 (0-6): ").strip()

            if choice == '1':
                # 单电机控制（按设备ID路由；若未知ID映射则广播）
                try:
                    device_id = int(input("请输入目标设备ID (1-20): "))
                    value = float(input("请输入目标角度值: "))
                    packet = communicator.create_single_packet(device_id, 0x01, value)

                    target_addr = communicator.id_to_address.get(device_id)
                    if target_addr:
                        await communicator.send_to_single_device(target_addr, packet)
                    else:
                        await communicator.send_broadcast_data(packet)

                    await communicator.wait_for_responses(target_device_id=device_id)
                except ValueError:
                    print("❌ 输入格式错误")

            elif choice == '2':
                # 多电机批量控制（新版切片式）
                try:
                    start_id = int(input("请输入起始设备ID (1-20): ").strip())
                    values_input = input("请输入该组的角度值（用空格分隔，建议5个）: ").strip()
                    values = [float(x) for x in values_input.split()]
                    packet = communicator.create_multi_slice_packet(start_id, values, 0x01)
                    await communicator.send_broadcast_data(packet)
                    await communicator.wait_for_responses()
                except ValueError:
                    print("❌ 输入格式错误")

            elif choice == '3':
                await communicator.check_connection_status()

            elif choice == '4':
                try:
                    values_input = input("请输入所有设备的角度值（用空格分隔）: ").strip()
                    values = [float(x) for x in values_input.split()]
                    group_size_in = input("请输入每组数量（默认5）: ").strip()
                    group_size = int(group_size_in) if group_size_in else 5
                    freq_in = input("可选刷新频率Hz（留空表示按队列耗尽）: ").strip()
                    per_device_hz = float(freq_in) if freq_in else None
                    rounds_in = input("可选最大轮次（留空直到队列耗尽）: ").strip()
                    max_rounds = int(rounds_in) if rounds_in else None

                    # 用顺序值映射为 {id: value} 并加载
                    id_values = {i + 1: v for i, v in enumerate(values)}
                    communicator.load_id_values_dict(id_values)

                    await communicator.broadcast_buffers_multi_rounds(
                        group_size=group_size,
                        data_type=0x01,
                        per_device_hz=per_device_hz,
                        max_rounds=max_rounds
                    )
                    await communicator.wait_for_responses()
                except ValueError:
                    print("❌ 输入格式错误")

            elif choice == '5':
                try:
                    path = communicator.watch_file_path or DEFAULT_WATCH_FILE
                    if not path or not os.path.isfile(path):
                        print(f"❌ 文件不存在: {path}")
                        continue
                    # 读取ID→值，并解析底部配置（group_size、per_device_hz、max_rounds、data_type）
                    id_values: Dict[int, float] = {}
                    group_size = 5
                    per_device_hz: Optional[float] = None
                    max_rounds: Optional[int] = None
                    data_type_str: Optional[str] = None
                    ext = os.path.splitext(path)[1].lower()
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
                                        data_type_str = val.strip().lower()
                    else:
                        with open(path, encoding='utf-8') as f:
                            for line in f:
                                line = line.strip()
                                if not line:
                                    continue
                                parts = [p.strip() for p in (line.split(',') if ',' in line else line.split())]
                                if len(parts) < 2:
                                    continue
                                key = parts[0]
                                val = parts[1]
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
                                        data_type_str = val.strip().lower()

                    communicator.load_id_values_dict(id_values)
                    if data_type_str == "velocity":
                        data_type = 0x02
                    elif data_type_str == "current":
                        data_type = 0x03
                    else:
                        data_type = 0x01

                    print(f"🚀 自动执行：group_size={group_size}, per_device_hz={per_device_hz}, max_rounds={max_rounds}, data_type={data_type_str or 'angle'}")
                    await communicator.broadcast_buffers_multi_rounds(
                        group_size=group_size,
                        data_type=data_type,
                        per_device_hz=per_device_hz,
                        max_rounds=max_rounds
                    )
                    await communicator.wait_for_responses()
                except Exception as e:
                    print(f"❌ 自动执行失败: {e}")

            elif choice == '6':
                # 自动读取targets文件并按结构体化 MULTI 发送
                try:
                    path = communicator.watch_file_path or DEFAULT_WATCH_FILE
                    if not path or not os.path.isfile(path):
                        print(f"❌ 文件不存在: {path}")
                        continue

                    id_values: Dict[int, float] = {}
                    group_size = 5
                    per_device_hz: Optional[float] = 1.0  # 你的示例为 1
                    max_rounds: Optional[int] = 1         # 你的示例为 1
                    data_type_str: Optional[str] = None
                    packet_mode_str: Optional[str] = None

                    ext = os.path.splitext(path)[1].lower()
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
                                        data_type_str = val.strip().lower()
                                    elif k in ("packet_mode", "packet_type"):
                                        packet_mode_str = val.strip().lower()
                    else:
                        with open(path, encoding='utf-8') as f:
                            for line in f:
                                line = line.strip()
                                if not line:
                                    continue
                                parts = [p.strip() for p in (line.split(',') if ',' in line else line.split())]
                                if len(parts) < 2:
                                    continue
                                key = parts[0]
                                val = parts[1]
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
                                        data_type_str = val.strip().lower()
                                    elif k in ("packet_mode", "packet_type"):
                                        packet_mode_str = val.strip().lower()

                    communicator.load_id_values_dict(id_values)

                    # 数据类型映射
                    if data_type_str == "velocity":
                        data_type = 0x02
                    elif data_type_str == "current":
                        data_type = 0x03
                    else:
                        data_type = 0x01  # angle

                    # 是否结构体化：默认 True；显式 slice 时改为 False
                    use_struct = True
                    if packet_mode_str is not None:
                        use_struct = packet_mode_str in ("struct", "multi_struct", "03", "0x03")

                    print(f"🚀 结构体化MULTI：group_size={group_size}, per_device_hz={per_device_hz}, max_rounds={max_rounds}, data_type={data_type_str or 'angle'}, packet_mode={packet_mode_str or 'struct'}")
                    await communicator.broadcast_buffers_multi_rounds(
                        group_size=group_size,
                        data_type=data_type,
                        per_device_hz=per_device_hz,
                        max_rounds=max_rounds,
                        use_struct=use_struct
                    )
                    await communicator.wait_for_responses()
                except Exception as e:
                    print(f"❌ 结构体化自动执行失败: {e}")

            elif choice == '0' or choice.lower() == 'quit':
                break
            else:
                print("❌ 无效选择")

    except Exception as e:
        print(f"❌ 程序错误: {e}")
    finally:
        # 断开所有连接
        await communicator.disconnect_all()

if __name__ == "__main__":
    # 检查Python版本
    if sys.version_info < (3, 7):
        print("❌ 需要Python 3.7或更高版本")
        sys.exit(1)
    
    print("ESP32多设备BLE通信测试程序（带断电检测）")
    print("支持同时连接最多10个ESP32设备，实时检测断电情况")
    
    # 运行多设备演示
    asyncio.run(multi_device_demo())
