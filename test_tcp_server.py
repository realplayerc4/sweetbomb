#!/usr/bin/env python3
"""
TCP服务端测试 - 等待下位机连接后发送消息
"""
import socket
import threading
import time
import sys

def handle_client(conn, addr):
    """处理客户端连接"""
    print(f"\n[+] 下位机已连接: {addr}")

    try:
        # 发送测试消息
        message = '{\nMessageType=status\n}'
        print(f"\n[>] 发送报文: {repr(message)}")
        print(f"    十六进制: {message.encode('utf-8').hex()}")

        conn.send(message.encode('utf-8'))
        print("[+] 报文已发送")

        # 等待接收回复
        print("\n[<] 等待回复 (5秒超时)...")
        conn.settimeout(5)

        try:
            data = conn.recv(4096)
            if data:
                print(f"[+] 收到 {len(data)} 字节:")
                try:
                    text = data.decode('utf-8')
                    print(f"    内容:\n{text}")
                except:
                    print(f"    十六进制: {data.hex()}")
            else:
                print("[-] 连接已关闭，未收到数据")
        except socket.timeout:
            print("[-] 超时: 未收到回复")

    except Exception as e:
        print(f"[!] 错误: {e}")
    finally:
        conn.close()
        print(f"\n[+] 连接已关闭: {addr}")

def main():
    host = "0.0.0.0"
    port = 9090

    # 创建socket
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((host, port))
        server.listen(5)
        print(f"[*] TCP服务端已启动: {host}:{port}")
        print("[*] 等待下位机连接...")
        print("\n" + "="*50)

        while True:
            conn, addr = server.accept()
            # 创建新线程处理连接
            client_thread = threading.Thread(
                target=handle_client,
                args=(conn, addr)
            )
            client_thread.daemon = True
            client_thread.start()

    except KeyboardInterrupt:
        print("\n[!] 服务端已停止")
    except Exception as e:
        print(f"[!] 错误: {e}")
    finally:
        server.close()

if __name__ == '__main__':
    main()
