#!/usr/bin/env python3
"""
测试向9090端口发送TCP数据
"""
import socket
import time
import sys

def send_status_query():
    """发送状态查询报文"""
    # 创建TCP客户端连接到本地9090端口
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.settimeout(5)  # 5秒超时

    try:
        # 连接到上位机TCP Server
        print("正在连接 127.0.0.1:9090...")
        sock.connect(('127.0.0.1', 9090))
        print("连接成功！")

        # 发送状态查询报文
        message = '{\nMessageType=status\n}'
        print(f"\n发送报文: {repr(message)}")
        print(f"十六进制: {message.encode('utf-8').hex()}")

        sock.send(message.encode('utf-8'))
        print("报文已发送，等待回复...")

        # 等待回复
        sock.settimeout(3)
        try:
            data = sock.recv(4096)
            if data:
                print(f"\n收到回复: {data.decode('utf-8')}")
                print(f"回复十六进制: {data.hex()}")
            else:
                print("连接已关闭，未收到数据")
        except socket.timeout:
            print("\n超时: 3秒内未收到回复")

    except ConnectionRefusedError:
        print("连接被拒绝: 请确保9090端口的服务已启动")
        sys.exit(1)
    except Exception as e:
        print(f"错误: {e}")
        sys.exit(1)
    finally:
        sock.close()
        print("\n连接已关闭")

if __name__ == '__main__':
    send_status_query()
