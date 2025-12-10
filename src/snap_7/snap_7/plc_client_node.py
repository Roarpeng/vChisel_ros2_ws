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
        self.declare_parameter('plc_address', '192.168.1.36')
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
        if self.camStatus:
            self.get_logger().info('Camera already launched')
            return

        try:
            cmd = ['ros2', 'run', 'realsense2_camera', 'realsense2_camera_node']
            self.cam_proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.camStatus = True
            time.sleep(0.5)
            if self.cam_proc.poll() is not None:
                self.get_logger().error(f'Camera process exited immediately with code {self.cam_proc.poll()}')
                self.camStatus = False
                return
            self.get_logger().info('Camera process started')
        except FileNotFoundError:
            self.get_logger().error('ros2 executable not found. Ensure ROS2 is sourced.')
        except Exception as e:
            self.get_logger().error(f'Failed to start camera process: {e}')

    def cam_shutdown(self):
        if not self.camStatus:
            self.get_logger().info('Camera not running')
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

        self.camStatus = False
        self.mark_1 = 0
        self.mark_2 = 0
        self.pakg_new = 0
        self.get_logger().info('Camera process shutdown complete')

    def norm_bringup(self, seq):
        """Initiate async norm_calc service call."""
        if not self.camStatus:
            self.get_logger().info('Camera not launched yet')
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
                else:
                    self._handle_norm_response(resp, seq)
            except Exception as e:
                self.get_logger().error(f'Exception processing norm_calc result: {e}')

        # Read trigger from PLC
        data = self.read_db_area(DB_num=self.db_number, start=0, size=2)
        if data is None:
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
                if not self.camStatus:
                    self.cam_bringup()
                # Write rosStatus = 200
                self.write_registers_uint16([200], start=16)
                self.Old_trigger = self.Trigger_Portal

            elif self.Trigger_Portal == 110 and self.Old_trigger != self.Trigger_Portal:
                self.get_logger().info('Take a Picture and calculate pose data!')
                self.callSeq += 1
                self.norm_bringup(self.callSeq)
                # Note: rosStatus will be written after processing is complete in _handle_norm_response
                self.Old_trigger = self.Trigger_Portal
                
            elif self.Trigger_Portal == 130 and self.Old_trigger != self.Trigger_Portal:
                self.get_logger().info('Close the Camera!')
                self.get_logger().info('[LOG] Clearing buffer and logging data!')
                self.cam_shutdown()
                self.write_registers_uint16([0], start=16)
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