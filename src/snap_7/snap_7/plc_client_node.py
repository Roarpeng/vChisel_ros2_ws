#!/usr/bin/env python3
"""
A simple ROS2 node that talks to Siemens PLC DB using snap7.
Behavior:
- Periodically read a DB word (DB number and offset configurable).
- If trigger value changes to 110, call the `norm_calc` service provided by `norm_calc` package.
- Write back a small status and number-of-points to PLC DB.

Notes:
- Requires `python-snap7` (snap7) and ROS2 (rclpy).
- This is a minimal example: production code should handle byte-order, types,
  retries, connection loss, and DB layout precisely.
"""

import sys
import time
import struct
import threading

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from norm_calc.srv import NormCalcData
import subprocess
import shlex

try:
    import snap7
    from snap7.util import get_int, set_int
except Exception as e:
    snap7 = None


class PLCClientNode(Node):
    def __init__(self):
        super().__init__('plc_client_node')

        # Parameters (could be remapped via launch)
        self.declare_parameter('plc_address', '192.168.50.225')
        self.declare_parameter('plc_rack', 0)
        self.declare_parameter('plc_slot', 1)
        self.declare_parameter('db_number', 2120)  # Updated to match the working test script
        self.declare_parameter('db_start', 0)  # byte offset
        self.declare_parameter('poll_rate', 20.0)  # Hz
        self.declare_parameter('simulate_plc', False)  # Add simulate mode parameter

        self.plc_address = self.get_parameter('plc_address').get_parameter_value().string_value
        self.plc_rack = self.get_parameter('plc_rack').get_parameter_value().integer_value
        self.plc_slot = self.get_parameter('plc_slot').get_parameter_value().integer_value
        self.db_number = self.get_parameter('db_number').get_parameter_value().integer_value
        self.db_start = self.get_parameter('db_start').get_parameter_value().integer_value
        self.poll_rate = float(self.get_parameter('poll_rate').get_parameter_value().double_value)
        self.simulate_plc = self.get_parameter('simulate_plc').get_parameter_value().bool_value

        if snap7 is None:
            self.get_logger().error('python-snap7 is not installed. Install it with pip install snap7')
            rclpy.shutdown()
            return

        self.client = snap7.client.Client()
        self.connected = False
        self.lock = threading.Lock()
        self.prev_trigger = None

        # Initialize connection with retries
        self.connect_with_retry()

        # state variables similar to modbus client
        self.camStatus = False
        self.cam_proc = None
        self.norm_launch = None
        self.mark_1 = 0
        self.mark_2 = 0
        self.pakg_new = 0
        self.callSeq = 0
        self.Old_trigger = 0
        self.Trigger_Portal = 0
        self.time_stamp = time.localtime()
        
        # 用于相机状态监控的额外变量
        self.camera_disconnect_count = 0  # 记录连续检测到断开的次数
        self.max_disconnect_threshold = 5  # 连续检测到断开的最大阈值，提高容错性
        self.camera_connect_time = None  # 记录相机连接时间
        self.grace_period_seconds = 10  # 宽限期，在此期间对断开更宽容

        # buffers for pages
        self.Output_Status = []
        self.Outputlist = [0] * 288
        

        # For async service call
        self.pending_norm_future = None
        self.pending_seq = None

        # Create a client for calling norm_calc service
        self.cli = self.create_client(NormCalcData, 'norm_calc')
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warning('norm_calc service not available yet')

        # Start polling timer
        timer_period = 1.0 / max(self.poll_rate, 0.1)
        self.timer = self.create_timer(timer_period, self.poll_plc)

        # Try to connect with retries
        self.connect_with_retry()

    def connect(self):
        try:
            if not self.client.get_connected():
                self.client.connect(self.plc_address, self.plc_rack, self.plc_slot)
            # Even if connection appears successful, verify by trying a simple operation
            # For now, just set the connection status and let operations handle errors as needed
            self.connected = True
            self.get_logger().info(f'Connected to PLC at {self.plc_address} (rack={self.plc_rack}, slot={self.plc_slot})')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to PLC: {e}')
            self.connected = False  # Keep connection as False to indicate issue

    def connect_with_retry(self, max_retries=3):
        retry_count = 0
        retry_interval = 2
        max_retry_interval = 10
        while retry_count < max_retries:
            try:
                self.connect()
                if self.connected:
                    return True
            except Exception as e:
                self.get_logger().error(f'Connection attempt failed: {e}')
            retry_count += 1
            if retry_count < max_retries:
                self.get_logger().info(f'Connect retry {retry_count}/{max_retries} in {retry_interval}s')
                time.sleep(retry_interval)
                retry_interval = min(retry_interval * 2, max_retry_interval)  # Exponential backoff
        self.get_logger().warning('Failed to connect to PLC after retries, continuing with limited functionality')
        # Even if connection fails, we can still attempt operations later
        return True  # Return True to allow node to continue running

    def read_db_area(self, DB_num, start, size=2):
        with self.lock:
            try:
                if self.simulate_plc:
                    # Return simulated data (two zero bytes)
                    return bytearray([0, 0])
                    
                # Attempt to connect if not connected
                if not self.client.get_connected():
                    self.connect()
                
                # Even if self.connected is False, try the read operation as it might work
                data = self.client.db_read(DB_num, start, size)
                if not self.connected:
                    self.connected = True  # If read succeeds, connection is working
                return data
            except Exception as e:
                self.get_logger().debug(f'Debug: Error reading DB: {e}')  # Use debug level to reduce spam
                # Don't set self.connected = False here as read might work even when connection "fails"
                return None

    def write_real_to_db(self, offset, value):
        """向DB写入REAL类型数据（32位浮点数），每个REAL占用4个字节"""
        with self.lock:
            try:
                if self.simulate_plc:
                    self.get_logger().info(f'[LOG] Simulated write REAL to DB{self.db_number} offset {offset}: {value}')
                    return True
                    
                # Attempt to connect if not connected
                if not self.client.get_connected():
                    self.connect()
                    if not self.connected:
                        self.get_logger().warning(f'Cannot write to PLC at offset {offset}: not connected')
                        return False
                
                # 将浮点数转换为字节数据 (IEEE 754标准，32位单精度浮点数)
                data = struct.pack('>f', float(value))  # '>f' 表示大端序的32位浮点数
                
                # 记录发送的数据日志，便于对比发送的数据和接收的数据是否一致
                self.get_logger().info(f'[LOG] Sending REAL to DB{self.db_number}[{offset}]: {value} (bytes: {data.hex()})')
                
                # 写入DB
                self.client.db_write(self.db_number, offset, data)
                self.get_logger().debug(f'Successfully wrote REAL to DB{self.db_number}[{offset}] = {value}')
                return True
            except Exception as e:
                error_msg = str(e)
                self.get_logger().error(f'Error writing REAL to DB: {e}')
                
                # Check if this is the specific CPU error we're seeing
                if "CPU : Item not available" in error_msg or "function refused by CPU" in error_msg:
                    self.get_logger().warning(f'PLC CPU refused write operation at DB{self.db_number} offset {offset}. This may be due to DB configuration or PLC security settings.')
                    # Add more detailed logging about the attempted write
                    self.get_logger().info(f'Attempted to write REAL value {value} to DB{self.db_number} at offset {offset}')
                    self.get_logger().info('PLC Configuration Required: Ensure "Full Access" level and "PUT/GET Communication" are enabled in PLC security settings. Also verify "Optimized Block Access" is turned OFF in the DB properties in TIA Portal.')
                else:
                    # For other errors, assume connection problem
                    self.connected = False
                
                return False

    def write_integer_to_db(self, offset, value):
        """向DB写入整数，参考Tsnap7test.py的实现方式"""
        with self.lock:
            try:
                if self.simulate_plc:
                    self.get_logger().info(f'[LOG] Simulated write to DB{self.db_number} offset {offset}: {value}')
                    return True
                    
                # Attempt to connect if not connected
                if not self.client.get_connected():
                    self.connect()
                    if not self.connected:
                        self.get_logger().warning(f'Cannot write to PLC at offset {offset}: not connected')
                        return False
                
                # 将整数转换为字节数据 (参考Tsnap7test.py)
                data = bytearray(2)
                data[0] = (value >> 8) & 0xFF  # 高字节
                data[1] = value & 0xFF         # 低字节
                
                # 记录发送的数据日志，便于对比发送的数据和接收的数据是否一致
                self.get_logger().info(f'[LOG] Sending integer to DB{self.db_number}[{offset}]: {value} (bytes: {data[0]:02X}{data[1]:02X})')
                
                # 写入DB
                self.client.db_write(self.db_number, offset, data)
                self.get_logger().debug(f'Successfully wrote to DB{self.db_number}[{offset}] = {value}')
                return True
            except Exception as e:
                error_msg = str(e)
                self.get_logger().error(f'Error writing integer to DB: {e}')
                
                # Check if this is the specific CPU error we're seeing
                if "CPU : Item not available" in error_msg or "function refused by CPU" in error_msg:
                    self.get_logger().warning(f'PLC CPU refused write operation at DB{self.db_number} offset {offset}. This may be due to DB configuration or PLC security settings.')
                    # Add more detailed logging about the attempted write
                    self.get_logger().info(f'Attempted to write value {value} to DB{self.db_number} at offset {offset}')
                    self.get_logger().info('PLC Configuration Required: Ensure "Full Access" level and "PUT/GET Communication" are enabled in PLC security settings. Also verify "Optimized Block Access" is turned OFF in the DB properties in TIA Portal.')
                else:
                    # For other errors, assume connection problem
                    self.connected = False
                
                return False

    def write_db_area(self, data_bytes, start=64):
        with self.lock:
            try:
                if self.simulate_plc:
                    self.get_logger().info(f'[LOG] Simulated write to DB{self.db_number} offset {start}: {data_bytes}')
                    return True
                    
                # Attempt to connect if not connected
                if not self.client.get_connected():
                    self.connect()
                    if not self.connected:
                        self.get_logger().warning(f'Cannot write to PLC at offset {start}: not connected')
                        return False
                
                # Check if data_bytes is valid
                if not isinstance(data_bytes, (bytes, bytearray)):
                    self.get_logger().error(f'Invalid data type for writing: {type(data_bytes)}')
                    return False
                
                # Verify that the data length is reasonable
                if len(data_bytes) <= 0:
                    self.get_logger().error(f'Invalid data length: {len(data_bytes)}')
                    return False
                
                # 记录发送的数据日志，便于对比发送的数据和接收的数据是否一致
                self.get_logger().info(f'[LOG] Sending {len(data_bytes)} bytes to DB{self.db_number}[{start}]: {data_bytes.hex() if hasattr(data_bytes, "hex") else bytes(data_bytes).hex()}')
                
                self.client.db_write(self.db_number, start, data_bytes)
                return True
            except Exception as e:
                error_msg = str(e)
                self.get_logger().error(f'Error writing DB: {e}')
                
                # Check if this is the specific CPU error we're seeing
                if "CPU : Item not available" in error_msg or "function refused by CPU" in error_msg:
                    self.get_logger().warning(f'PLC CPU refused write operation at DB{self.db_number} offset {start}. This may be due to DB configuration or PLC security settings.')
                    # Don't set connected to False for this specific error as read operations may still work
                    # The error is likely due to DB write protection or configuration rather than connection issue
                    self.get_logger().info(f'Attempted to write {len(data_bytes)} bytes to DB{self.db_number} at offset {start}')
                    self.get_logger().info('PLC Configuration Required: Ensure "Full Access" level and "PUT/GET Communication" are enabled in PLC security settings. Also verify "Optimized Block Access" is turned OFF in the DB properties in TIA Portal.')
                else:
                    # For other errors, assume connection problem
                    self.connected = False
                
                return False

    def write_real_array_to_db(self, real_list, start=20):
        """向DB写入REAL类型数组，从指定偏移开始，每个REAL占4个字节"""
        if not real_list:
            self.get_logger().warning(f'Empty REAL list provided for write at offset {start}')
            return False
            
        # Verify that the client is connected before attempting to write
        if not self.client.get_connected():
            self.get_logger().error(f'Cannot write to PLC: not connected. Attempting to connect...')
            self.connect()
            if not self.connected:
                self.get_logger().error(f'Failed to connect to PLC, aborting write at offset {start}')
                return False
        
        # 记录发送的数据日志，便于对比发送的数据和接收的数据是否一致
        self.get_logger().info(f'[LOG] Sending {len(real_list)} REAL values to DB{self.db_number} starting at offset {start}: {real_list}')
        
        success_count = 0
        for i, value in enumerate(real_list):
            offset = start + (i * 4)  # 每个REAL占4个字节
            write_result = self.write_real_to_db(offset, value)
            if write_result:
                success_count += 1
            else:
                self.get_logger().debug(f'Failed to write REAL value {value} at offset {offset} (index {i})')
                # Check if this was a CPU access error and stop further writes if so
                # This prevents spamming the PLC with requests it will refuse
                # We can continue with other operations though
        
        self.get_logger().debug(f'Successfully wrote {success_count}/{len(real_list)} REAL values starting at offset {start}')
        # Return True even if some writes failed, as long as we attempted them all
        # This allows the program to continue functioning
        return success_count > 0 or len(real_list) == 0

    def write_registers_uint16(self, reg_list, start=64):
        # Create bytearray with appropriate size using snap7 recommended approach
        if not reg_list:
            self.get_logger().warning(f'Empty register list provided for write at offset {start}')
            return False
            
        # Verify that the client is connected before attempting to write
        if not self.client.get_connected():
            self.get_logger().error(f'Cannot write to PLC: not connected. Attempting to connect...')
            self.connect()
            if not self.connected:
                self.get_logger().error(f'Failed to connect to PLC, aborting write at offset {start}')
                return False
        
        # 记录发送的数据日志，便于对比发送的数据和接收的数据是否一致
        self.get_logger().info(f'[LOG] Sending {len(reg_list)} registers to DB{self.db_number} starting at offset {start}: {reg_list}')
        
        # 使用更简单的方式写入整数列表，参考Tsnap7test.py的方法
        # 对于每个整数，我们直接调用write_integer_to_db方法
        success_count = 0
        for i, value in enumerate(reg_list):
            offset = start + (i * 2)  # 每个整数占2个字节
            converted_value = int(float(value))
            write_result = self.write_integer_to_db(offset, converted_value)
            if write_result:
                success_count += 1
            else:
                self.get_logger().debug(f'Failed to write value {converted_value} at offset {offset} (index {i})')
                # Check if this was a CPU access error and stop further writes if so
                # This prevents spamming the PLC with requests it will refuse
                # We can continue with other operations though
        
        self.get_logger().debug(f'Successfully wrote {success_count}/{len(reg_list)} registers starting at offset {start}')
        # Return True even if some writes failed, as long as we attempted them all
        # This allows the program to continue functioning
        return success_count > 0 or len(reg_list) == 0

    def Clean_Buffer(self):
        # Clear data for offsets 20-592 (从偏移20开始到592)
        # 144个REAL值需要576字节 (144 * 4) 
        zlist = [0.0] * 144  # 使用浮点数0.0来清空
        self.write_real_array_to_db(zlist, start=20)
        self.get_logger().info('Clear All Data In Buffer!')

    def Msg_Status_write(self, Msg_S):
        # 写入单个整数值到寄存器，从start=16开始
        Output_Status = [int(Msg_S) & 0xFFFF]
        self.write_registers_uint16(Output_Status, start=16)

    def Msg_write(self):
        self.Clean_Buffer()
        if hasattr(self, 'Outputlist'):
            out = self.Outputlist[:144]  # 144 values for offsets 20-306
            if len(out) < 144:
                out += [0] * (144 - len(out))
            self.write_registers_uint16(out, start=20)

    def cam_bringup(self):
        # 首先确保之前的相机进程被终止
        if self.cam_proc and self.cam_proc.poll() is None:
            self.get_logger().info('Terminating existing camera process before starting new one...')
            try:
                self.cam_proc.terminate()
                try:
                    self.cam_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.cam_proc.kill()
                    self.get_logger().info('Killed existing camera process that did not terminate gracefully')
            except Exception as e:
                self.get_logger().warning(f'Error terminating existing camera process: {e}')
        
        # 重置相机状态和断开计数器
        self.camStatus = False
        self.camera_disconnect_count = 0
        import time
        self.camera_connect_time = time.time()  # 记录连接时间
        time.sleep(1)  # 等待一段时间确保旧进程完全终止
        
        # 重试机制：最多尝试5次
        max_attempts = 5
        for attempt in range(max_attempts):
            self.get_logger().info(f'Camera startup attempt {attempt + 1}/{max_attempts}')
            
            # 每次尝试时都检查物理连接
            if not self.check_physical_camera_connection_stable():
                self.get_logger().info(f'Physical camera not connected on attempt {attempt + 1}, waiting for connection...')
                # 在等待一段时间后再次尝试
                time.sleep(2)
                continue

            # 再次检查是否已经有运行中的进程（可能在等待期间启动了）
            if self.camStatus:
                self.get_logger().info('Camera already launched')
                return

            try:
                # 修改相机启动参数，提高帧率从10到15fps，并禁用一些可能导致缓存的选项
                cmd = ['ros2', 'run', 'realsense2_camera', 'realsense2_camera_node',
                       '--ros-args', 
                       '-p', 'rgb_camera.profile:=1280,720,15',
                       '-p', 'depth_module.profile:=1280,720,15',
                       '-p', 'color_width:=1280', '-p', 'color_height:=720',
                       '-p', 'color_fps:=15.0',
                       '-p', 'depth_width:=1280', '-p', 'depth_height:=720',
                       '-p', 'depth_fps:=15.0',
                       '-p', 'enable_color:=true', '-p', 'enable_depth:=true',
                       '-p', 'align_depth.enable:=true', '-p', 'align_depth:=true',
                       '-p', 'enable_gyro:=false', '-p', 'enable_accel:=false',
                       '-p', 'enable_infra1:=false', '-p', 'enable_infra2:=false',
                       '-p', 'publish_tf:=false']
                self.cam_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                self.camStatus = True
                time.sleep(3)  # 增加等待时间确保相机完全启动
                
                if self.cam_proc.poll() is not None:
                    self.get_logger().error(f'Camera process exited immediately with code {self.cam_proc.poll()} on attempt {attempt + 1}')
                    self.camStatus = False
                    # 终止进程并准备重试
                    try:
                        if self.cam_proc and self.cam_proc.poll() is None:
                            self.cam_proc.terminate()
                            try:
                                self.cam_proc.wait(timeout=5)
                            except subprocess.TimeoutExpired:
                                self.cam_proc.kill()
                                self.get_logger().info(f'Killed failed camera process on attempt {attempt + 1}')
                    except:
                        pass
                    # 在重试前稍等
                    time.sleep(2)
                    continue
                self.get_logger().info('Camera process started successfully')
                
                # 添加相机状态监控
                if hasattr(self, 'camera_monitor_thread') and self.camera_monitor_thread.is_alive():
                    # 如果已有监控线程在运行，先等待它结束
                    try:
                        self.camera_monitor_thread.join(timeout=1)  # 等待最多1秒
                    except:
                        pass
                
                self.camera_monitor_thread = threading.Thread(target=self._monitor_camera_status, daemon=True)
                self.camera_monitor_thread.start()
                
                # 成功启动后退出循环
                break
                
            except FileNotFoundError:
                self.get_logger().error('ros2 executable not found. Ensure ROS2 is sourced.')
                self.camStatus = False  # 确保将状态设为False，以便后续可以重试
                self.write_registers_uint16([999], start=16)  # 发送错误代码
                return
            except Exception as e:
                self.get_logger().error(f'Failed to start camera process on attempt {attempt + 1}: {e}')
                self.camStatus = False  # 确保将状态设为False，以便后续可以重试
                # 终止进程并准备重试
                try:
                    if self.cam_proc and self.cam_proc.poll() is None:
                        self.cam_proc.terminate()
                        try:
                            self.cam_proc.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            self.cam_proc.kill()
                            self.get_logger().info(f'Killed failed camera process on attempt {attempt + 1}')
                except:
                    pass
                # 在重试前稍等
                time.sleep(2)
        
        # 如果所有尝试都失败了
        if not self.camStatus:
            self.get_logger().error(f'Failed to start camera after {max_attempts} attempts')
            self.write_registers_uint16([999], start=16)  # 发送错误代码

    def cam_shutdown(self):
        if not self.camStatus:
            self.get_logger().info('Camera not running')
            # 即使相机没有运行，也要确保状态被正确重置
            self.camStatus = False
            self.mark_1 = 0
            self.mark_2 = 0
            self.pakg_new = 0
            return

        try:
            if self.cam_proc and self.cam_proc.poll() is None:
                self.cam_proc.terminate()
                try:
                    self.cam_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    self.cam_proc.kill()
        except Exception as e:
            self.get_logger().warning(f'Error terminating camera process: {e}')

        # 确保所有相关状态都被重置
        self.camStatus = False
        self.mark_1 = 0
        self.mark_2 = 0
        self.pakg_new = 0
        self.get_logger().info('Camera process shutdown complete')

    def _monitor_camera_status(self):
        """监控相机进程状态，检测意外断开"""
        # 重置断开计数器
        self.camera_disconnect_count = 0
        import time
        start_time = time.time()
        
        while self.camStatus and rclpy.ok():
            try:
                # 检查物理连接 - 使用更稳定的检测方法
                if not self.check_physical_camera_connection_stable():
                    self.camera_disconnect_count += 1
                    self.get_logger().warning(f'Physical camera disconnected (count: {self.camera_disconnect_count})')
                    
                    # 检查是否在宽限期内
                    time_since_connect = time.time() - start_time if hasattr(self, 'camera_connect_time') else time.time() - start_time
                    in_grace_period = time_since_connect < self.grace_period_seconds
                    
                    # 如果在宽限期内，使用更宽松的阈值
                    threshold = 2 if in_grace_period else self.max_disconnect_threshold
                    
                    # 只有当连续检测到断开超过阈值时，才认为相机真正断开
                    if self.camera_disconnect_count >= threshold:
                        self.get_logger().warning(f'Physical camera disconnected permanently after {self.camera_disconnect_count} consecutive failures, stopping camera')
                        # 设置相机状态为断开
                        self.camStatus = False
                        # 向PLC发送错误状态
                        self.write_registers_uint16([999], start=16)  # 发送错误代码
                        break
                else:
                    # 如果检测到连接，重置断开计数器
                    self.camera_disconnect_count = 0
                
                if self.cam_proc and self.cam_proc.poll() is not None:
                    # 相机进程意外退出
                    self.get_logger().warning(f'Camera process unexpectedly terminated with code {self.cam_proc.poll()}')
                    self.camStatus = False
                    # 向PLC发送错误状态
                    self.write_registers_uint16([999], start=16)  # 发送错误代码
                    break
                time.sleep(2)  # 增加检查间隔到2秒，减少频繁检查的干扰
            except Exception as e:
                self.get_logger().error(f'Error in camera monitoring thread: {e}')
                break

    def check_camera_connection(self):
        """检查相机连接状态"""
        if not self.camStatus:
            return False
        if self.cam_proc and self.cam_proc.poll() is None:
            return True
        else:
            self.camStatus = False
            return False

    def wait_for_camera_ready(self, timeout=15):
        """等待相机准备好，不仅检查进程和物理连接，还等待一段时间让相机初始化"""
        start_time = time.time()
        self.get_logger().info('Waiting for camera to be fully ready...')
        
        # 先检查物理连接和进程状态
        while time.time() - start_time < timeout:
            if self.check_camera_connection() and self.check_physical_camera_connection():
                self.get_logger().info('Camera process and physical connection OK, waiting for initialization...')
                # 如果进程和物理连接都正常，等待一段时间让相机完全初始化
                time.sleep(3)  # 等待3秒让相机完全初始化
                
                # 再次检查是否仍然连接正常
                if self.check_camera_connection() and self.check_physical_camera_connection():
                    self.get_logger().info('Camera is fully ready')
                    return True
                else:
                    self.get_logger().info('Camera connection lost during initialization wait')
            time.sleep(0.5)
        return False

    def check_physical_camera_connection(self):
        """检查物理相机连接状态，使用多次检测减少误报"""
        try:
            # 尝试使用pyrealsense2来检测物理设备
            import pyrealsense2 as rs
            try:
                ctx = rs.context()
                devices = ctx.query_devices()
                if len(devices) > 0:
                    # 检查设备是否可访问
                    for device in devices:
                        try:
                            device_name = device.get_info(rs.camera_info.name) if device else "Unknown"
                            serial_number = device.get_info(rs.camera_info.serial_number) if device else "Unknown"
                            self.get_logger().debug(f'Physical RealSense Camera Found: {device_name}, Serial: {serial_number}')
                            return True
                        except Exception:
                            continue
            except Exception as e:
                self.get_logger().debug(f'Error with pyrealsense2 detection: {e}')
                
            return False
        except ImportError:
            # 如果pyrealsense2不可用，使用lsusb命令
            try:
                import subprocess
                result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=5)
                # 查找Intel RealSense设备
                for line in result.stdout.split('\n'):
                    if 'Intel' in line and ('RealSense' in line or '8086:' in line):
                        return True
                return False
            except Exception as e:
                self.get_logger().warning(f'Error checking physical camera connection: {e}')
                return False
        except Exception as e:
            self.get_logger().warning(f'Error checking physical camera connection: {e}')
            return False

    def check_physical_camera_connection_stable(self, num_checks=3, interval=0.1):
        """通过多次检测来确定物理连接状态，减少误报"""
        true_count = 0
        for i in range(num_checks):
            if self.check_physical_camera_connection():
                true_count += 1
            if i < num_checks - 1:  # 不在最后一次检查后等待
                time.sleep(interval)
        
        # 如果超过一半的检测结果为真，则认为连接正常
        return true_count > num_checks / 2

    def safe_camera_operation(self):
        """安全的相机操作，包含连接检查"""
        if not self.check_camera_connection() or not self.check_physical_camera_connection_stable():
            self.get_logger().warning('Camera not connected or physically disconnected, attempting to restart...')
            self.cam_bringup()
            time.sleep(3)  # 等待相机启动
            if not self.check_camera_connection() or not self.check_physical_camera_connection_stable():
                self.get_logger().error('Failed to establish camera connection')
                return False
        return True

    def norm_bringup(self, seq):
        """Initiate async norm_calc service call."""
        if not self.camStatus:
            self.get_logger().warning('Camera not launched yet, cannot perform norm calculation')
            # 向PLC发送错误状态
            self.write_registers_uint16([998], start=16)  # 相机未启动错误代码
            return
            
        # 额外检查物理连接
        if not self.check_physical_camera_connection():
            self.get_logger().warning('Physical camera not connected, cannot perform norm calculation')
            # 向PLC发送错误状态
            self.write_registers_uint16([998], start=16)  # 相机未启动错误代码
            return
            
        if self.pending_norm_future is not None and not self.pending_norm_future.done():
            self.get_logger().warning('Another norm_calc request is pending, skipping')
            return

        try:
            req = NormCalcData.Request()
            req.seq = int(seq)
            self.get_logger().info('Calling norm_calc service asynchronously...')
            future = self.cli.call_async(req)
            self.pending_norm_future = future
            self.pending_seq = seq
        except Exception as e:
            self.get_logger().error(f'Failed to initiate norm_calc call: {e}')
            # 向PLC发送错误状态
            self.write_registers_uint16([997], start=16)  # 服务调用错误代码

    def _handle_norm_response(self, resp, seq):
        """Process the norm_calc service response."""
        self.get_logger().info(f'we got {len(resp.pose_list.poses)} valid positions')

        # Prepare data array for writing to PLC as REAL values (floats)
        # Write the calculated pose data to offsets 20-592 (偏移20开始到592)
        output_data = []
        
        if len(resp.pose_list.poses) > 0:
            for i, pose in enumerate(resp.pose_list.poses):
                if i < 24:  # Only process first 24 poses
                    # Directly add the float values for x, y, z, rx, ry, rz
                    vals = [
                        pose.position.x, pose.position.y, pose.position.z,
                        pose.orientation.x, pose.orientation.y, pose.orientation.z
                    ]
                    output_data.extend(vals)

        # Ensure output_data has the right size for 24 poses (24 * 6 = 144 values)
        # Each pose has 6 values (x, y, z, rx, ry, rz), so 24 poses = 144 float values
        expected_size = 24 * 6  # 144 values for 24 poses
        if len(output_data) < expected_size:
            output_data += [0.0] * (expected_size - len(output_data))
        elif len(output_data) > expected_size:
            output_data = output_data[:expected_size]
        
        # Write the pose data as REAL values to offset 20-592 (24 poses * 6 values * 4 bytes = 576 bytes = 144 floats)
        self.get_logger().debug(f'Attempting to write {len(output_data)} REAL values to offset 20')
        pose_write_success = self.write_real_array_to_db(output_data, start=20)
        
        # Write pointNum to offset 18 (still as integer)
        pointNum = min(len(resp.pose_list.poses), 24)  # Limit to 24 points maximum
        self.get_logger().debug(f'Attempting to write point count {pointNum} to offset 18')
        pointnum_write_success = self.write_registers_uint16([int(pointNum) & 0xFFFF], start=18)
        
        # Write rosStatus = 210 to offset 16 after all data is written (still as integer)
        self.get_logger().debug(f'Attempting to write status 210 to offset 16')
        rosstatus_write_success = self.write_registers_uint16([210], start=16)
        
        self.pakg_new = 1
        
        # Report status of operations
        success_count = sum([pose_write_success, pointnum_write_success, rosstatus_write_success])
        total_ops = 3
        self.get_logger().info(f'Norm calculation done. {min(len(resp.pose_list.poses), 24)} poses processed, {success_count}/{total_ops} write operations successful')
        
        if not (pose_write_success and pointnum_write_success and rosstatus_write_success):
            self.get_logger().warning('Some write operations failed. This may be due to PLC DB configuration or write protection.')
            if not pose_write_success:
                self.get_logger().warning(f'Pose data write to offset 20 failed')
            if not pointnum_write_success:
                self.get_logger().warning(f'Point count write to offset 18 failed')
            if not rosstatus_write_success:
                self.get_logger().warning(f'Status write to offset 16 failed')
        
        # 内存清理：清理临时数据结构
        del output_data  # 显式删除临时数据数组
        self.Outputlist = [0] * 288  # 重置输出列表

    def poll_plc(self):
        # Check if norm_calc response is ready
        if self.pending_norm_future is not None and self.pending_norm_future.done():
            future = self.pending_norm_future
            seq = self.pending_seq
            self.pending_norm_future = None
            self.pending_seq = None
            try:
                resp = future.result()
                if resp is None:
                    self.get_logger().error('norm_calc service returned None')
                    # 发送错误状态到PLC
                    self.write_registers_uint16([997], start=16)  # norm_calc服务错误
                elif resp.pose_list.header.frame_id == "error":
                    self.get_logger().error('norm_calc service returned error status')
                    # 发送错误状态到PLC
                    self.write_registers_uint16([996], start=16)  # 计算错误
                else:
                    self._handle_norm_response(resp, seq)
            except Exception as e:
                self.get_logger().error(f'Exception processing norm_calc result: {e}')
                # 发送错误状态到PLC
                self.write_registers_uint16([997], start=16)  # 异常错误

        # Read trigger from PLC
        data = self.read_db_area(DB_num=self.db_number, start=0, size=2)
        if data is None:
            # 尝试重新连接PLC
            self.connect_with_retry()
            return

        try:
            plcStatus = int.from_bytes(data[0:2], byteorder='big', signed=False)
        except Exception:
            plcStatus = 0

        self.get_logger().debug(f'plcStatus: {plcStatus}')  # Use debug to reduce spam

        if self.prev_trigger is None:
            self.prev_trigger = plcStatus

        if plcStatus != self.prev_trigger:
            self.get_logger().info(f'plcStatus changed: {self.prev_trigger} -> {plcStatus}')
            self.Trigger_Portal = int(plcStatus)

            if self.Trigger_Portal == 100 and self.Old_trigger != self.Trigger_Portal:
                self.get_logger().info('Open the Cam!')
                
                # 检查物理连接
                if not self.check_physical_camera_connection_stable():
                    self.get_logger().warning('Physical camera not connected, attempting to start camera anyway...')
                    # 即使物理连接不可用，也要尝试启动相机，因为用户可能刚刚插入了相机
                    self.cam_bringup()
                else:
                    self.get_logger().info('Physical camera detected, proceeding to start camera')
                    if not self.camStatus:
                        self.cam_bringup()
                
                # 等待相机完全准备好
                self.get_logger().info('Waiting for camera to be ready...')
                is_ready = self.wait_for_camera_ready(timeout=15)  # 增加超时时间到15秒
                self.get_logger().info(f'Camera ready status: {is_ready}')
                self.get_logger().info(f'Camera process status: {self.check_camera_connection()}')
                self.get_logger().info(f'Physical camera status: {self.check_physical_camera_connection_stable()}')
                
                if is_ready:
                    self.write_registers_uint16([200], start=16)  # 成功启动
                    self.get_logger().info('Camera successfully launched and connected')
                else:
                    self.write_registers_uint16([999], start=16)  # 启动失败
                    self.get_logger().error('Failed to launch camera or camera not connected')
                self.Old_trigger = self.Trigger_Portal

            elif self.Trigger_Portal == 110 and self.Old_trigger != self.Trigger_Portal:
                self.get_logger().info('Take a Picture and calculate pose data!')
                # 在执行计算前先检查相机连接
                if self.safe_camera_operation():
                    self.callSeq += 1
                    self.norm_bringup(self.callSeq)
                else:
                    self.get_logger().error('Cannot perform norm calculation: camera not connected')
                    # 向PLC发送错误状态
                    self.write_registers_uint16([998], start=16)  # 发送相机错误代码
                # Note: rosStatus will be written after processing is complete in _handle_norm_response
                self.Old_trigger = self.Trigger_Portal
                
            elif self.Trigger_Portal == 130 or 120 and self.Old_trigger != self.Trigger_Portal:
                self.get_logger().info('Close the Camera!')
                self.get_logger().info('[LOG] Clearing buffer and logging data!')
                self.cam_shutdown()
                self.write_registers_uint16([220], start=16)
                # 重置相机状态标志，以便下次100指令时可以重新检测物理连接
                self.get_logger().info('Camera closed and status reset')
                self.Old_trigger = self.Trigger_Portal
                # self.Clean_Buffer()
                self.Old_trigger = self.Trigger_Portal

            elif self.Trigger_Portal == 99 and self.Old_trigger != self.Trigger_Portal:
                self.Old_trigger = self.Trigger_Portal
                self.write_registers_uint16([0], start=16)
                self.get_logger().info('Received exit trigger 99, shutting down node')
                rclpy.shutdown(),

            self.prev_trigger = plcStatus


def main(args=None):
    rclpy.init(args=args)
    node = PLCClientNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if hasattr(node, 'client') and node.client:
                node.client.disconnect()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()