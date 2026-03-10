import fcntl
import os
import sys

# Constants for USBDEVFS_RESET
USBDEVFS_RESET = 21780

def reset_usb_device(bus, device):
    filename = f"/dev/bus/usb/{bus}/{device}"
    print(f"正在重置设备: {filename}...")
    try:
        with open(filename, 'wb') as f:
            fcntl.ioctl(f, USBDEVFS_RESET, 0)
        print("重置命令已发送成功。")
    except Exception as e:
        print(f"重置失败: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 3:
        print("用法: python3 reset_usb.py <bus> <device>")
        sys.exit(1)
    
    # 转换为 3 位字符串格式
    bus = sys.argv[1].zfill(3)
    device = sys.argv[2].zfill(3)
    reset_usb_device(bus, device)
