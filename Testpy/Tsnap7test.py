import snap7
import socket
import subprocess
from snap7.util import *
from snap7 import type as snap7_type

class PLCDataWriter:
    def __init__(self, plc_ip="192.168.1.36"):
        self.plc_ip = plc_ip
        self.client = snap7.client.Client()
        self.db_number = 2120
        
    def connect(self, rack=0, slot=1):
        """连接到PLC"""
        try:
            print(f"正在连接到PLC {self.plc_ip}...")
            self.client.connect(self.plc_ip, rack, slot)
            if self.client.get_connected():
                print("✓ 成功连接到PLC")
                return True
            else:
                print("✗ 连接失败")
                return False
        except Exception as e:
            print(f"✗ 连接过程中出现错误: {e}")
            return False
    
    def disconnect(self):
        """断开与PLC的连接"""
        try:
            self.client.disconnect()
            print("已断开与PLC的连接")
        except Exception as e:
            print(f"断开连接时出现错误: {e}")
    
    def write_integer_to_db(self, offset, value):
        """向DB2120写入整数"""
        try:
            # 将整数转换为字节数据
            data = bytearray(2)
            data[0] = (value >> 8) & 0xFF  # 高字节
            data[1] = value & 0xFF         # 低字节
            
            # 写入DB
            self.client.db_write(self.db_number, offset, data)
            print(f"✓ 成功写入: DB{self.db_number}[{offset}] = {value}")
            return True
        except Exception as e:
            print(f"✗ 写入数据时出现错误: {e}")
            return False
    
    def read_integer_from_db(self, offset):
        """从DB2120读取整数以验证写入"""
        try:
            # 从DB读取2个字节
            data = self.client.db_read(self.db_number, offset, 2)
            # 转换为整数
            value = (data[0] << 8) | data[1]
            print(f"✓ 读取数据: DB{self.db_number}[{offset}] = {value}")
            return value
        except Exception as e:
            print(f"✗ 读取数据时出现错误: {e}")
            return None
    
    def interactive_write(self):
        """交互式写入数据"""
        if not self.connect():
            return
        
        print(f"\n开始向DB{self.db_number}写入数据")
        print("输入 'quit' 退出程序\n")
        
        while True:
            try:
                # 获取偏移地址
                offset_input = input("请输入偏移地址 (例如: 0, 2, 4): ")
                if offset_input.lower() == 'quit':
                    break
                
                offset = int(offset_input)
                
                # 获取要写入的整数值
                value_input = input("请输入要写入的整数值: ")
                if value_input.lower() == 'quit':
                    break
                    
                value = int(value_input)
                
                # 写入数据
                if self.write_integer_to_db(offset, value):
                    # 验证写入的数据
                    self.read_integer_from_db(offset)
                
                print("-" * 30)
                
            except ValueError:
                print("✗ 输入无效，请输入数字")
            except KeyboardInterrupt:
                print("\n程序被用户中断")
                break
            except Exception as e:
                print(f"✗ 发生错误: {e}")
        
        self.disconnect()

def main():
    """主函数"""
    print("=" * 50)
    print("PLC DB 数据写入工具")
    print("=" * 50)
    
    # PLC IP 地址
    PLC_IP = "192.168.1.36"
    
    writer = PLCDataWriter(PLC_IP)
    writer.interactive_write()

if __name__ == "__main__":
    main()
