import paramiko
import time

def run_sudo_cmd(client, cmd, password):
    print(f"Executing: {cmd}")
    # Using 'sudo -S bash -c' to handle pipes and redirections safely
    stdin, stdout, stderr = client.exec_command(f"sudo -S bash -c \"{cmd}\"")
    stdin.write(password + '\n')
    stdin.flush()
    
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    status = stdout.channel.recv_exit_status()
    
    print(f"Status: {status}")
    if out: print(f"OUT: {out}")
    if err: print(f"ERR: {err}")
    return status == 0

def final_fix():
    host = "192.168.0.73"
    user = "yq"
    password = "1"
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(host, username=user, password=password)
        
        print("--- Step 0: Aggressive Cleanup ---")
        # Remove all ROS related lists to unblock apt-get update
        run_sudo_cmd(client, "rm -v /etc/apt/sources.list.d/ros*.list", password)
        # Clear apt cache
        run_sudo_cmd(client, "apt-get clean", password)
        # Update
        run_sudo_cmd(client, "apt-get update", password)

        print("--- Step 1: Install Python 3.10 ---")
        run_sudo_cmd(client, "add-apt-repository -y ppa:deadsnakes/ppa", password)
        run_sudo_cmd(client, "apt-get update", password)
        run_sudo_cmd(client, "apt-get install -y python3.10 python3.10-venv python3.10-dev git", password)

        print("--- Step 2: Install Node.js 18 ---")
        run_sudo_cmd(client, "curl -fsSL https://deb.nodesource.com/setup_18.x | bash -", password)
        run_sudo_cmd(client, "apt-get install -y nodejs", password)

        print("--- Step 3: Verification ---")
        for cmd in ["python3.10 --version", "node --version", "npm --version"]:
            stdin, stdout, stderr = client.exec_command(cmd)
            print(f"{cmd}: {stdout.read().decode().strip() or stderr.read().decode().strip()}")

        print("--- Step 4: Full Redeploy ---")
        run_sudo_cmd(client, "rm -rf sweetbomb", password)
        # Clone
        stdin, stdout, stderr = client.exec_command("git clone https://github.com/realplayerc4/sweetbomb.git")
        stdout.channel.recv_exit_status()
        
        # Backend
        print("Backend Setup...")
        client.exec_command("cd sweetbomb/rest-api && python3.10 -m venv venv")
        time.sleep(2)
        # Upgrade pip and install requirements
        stdin, stdout, stderr = client.exec_command("cd sweetbomb/rest-api && ./venv/bin/python -m pip install --upgrade pip")
        stdout.channel.recv_exit_status()
        stdin, stdout, stderr = client.exec_command("cd sweetbomb/rest-api && ./venv/bin/python -m pip install -r requirements.txt")
        print(f"Backend requirements status: {stdout.channel.recv_exit_status()}")

        # Frontend
        print("Frontend Setup...")
        stdin, stdout, stderr = client.exec_command("cd sweetbomb/rest-api/ui/frontend && npm install --registry=https://registry.npmmirror.com && npm run build")
        print(f"Frontend setup status: {stdout.channel.recv_exit_status()}")

        print("\nFinal deployment complete.")

    except Exception as e:
        print(f"CRITICAL ERROR: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    final_fix()
