import paramiko

def setup_systemd():
    host = "192.168.0.73"
    user = "jetson"
    password = "jetson"
    
    service_content = """[Unit]
Description=RealSense Web Monitor (FastAPI & SPA)
After=network.target

[Service]
User=jetson
Group=jetson
WorkingDirectory=/home/jetson/sweetbomb
Environment="PATH=/home/jetson/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
Environment="PYTHONPATH=/home/jetson/sweetbomb"
ExecStart=/home/jetson/sweetbomb/venv/bin/python main.py
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
"""

    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(host, username=user, password=password)
        
        # Kill the nohup process if exists
        print("Stopping previous manual instances...")
        stdin, stdout, stderr = client.exec_command("pkill -f 'python main.py'")
        stdout.channel.recv_exit_status()
        
        # Write service file to tmp first
        print("Creating service file...")
        sftp = client.open_sftp()
        with sftp.file('/tmp/realsense-api.service', 'w') as f:
            f.write(service_content)
        sftp.close()
        
        # Move to systemd and enable
        print("Configuring systemd...")
        setup_cmds = [
            "sudo -S mv /tmp/realsense-api.service /etc/systemd/system/",
            "sudo -S systemctl daemon-reload",
            "sudo -S systemctl enable realsense-api",
            "sudo -S systemctl restart realsense-api"
        ]
        
        for cmd in setup_cmds:
            stdin, stdout, stderr = client.exec_command(cmd)
            stdin.write(password + '\n')
            stdin.flush()
            status = stdout.channel.recv_exit_status()
            print(f"[{status}] {cmd}")
            if status != 0:
                print(stderr.read().decode())
        
        print("\nChecking service status...")
        stdin, stdout, stderr = client.exec_command("systemctl status realsense-api --no-pager | head -n 15")
        print(stdout.read().decode().strip())
        
        print("Systemd setup complete.")
        
    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    setup_systemd()
