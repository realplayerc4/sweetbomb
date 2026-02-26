import paramiko
import sys

def check_remote(host, user, password):
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    try:
        print(f"Connecting to {host}...")
        client.connect(host, username=user, password=password)
        
        commands = [
            "python3 --version",
            "node --version",
            "git --version",
            "lsb_release -a"
        ]
        
        for cmd in commands:
            stdin, stdout, stderr = client.exec_command(cmd)
            out = stdout.read().decode().strip()
            err = stderr.read().decode().strip()
            print(f"Command: {cmd}")
            if out: print(f"Output: {out}")
            if err: print(f"Error: {err}")
            print("-" * 20)
            
    except Exception as e:
        print(f"Failed to connect: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    check_remote("192.168.0.73", "yq", "1")
