import paramiko
import time

def run_sudo_cmd(client, cmd, password):
    print(f"Executing: {cmd}")
    stdin, stdout, stderr = client.exec_command(f"sudo -S bash -c \"{cmd}\"")
    stdin.write(password + '\n')
    stdin.flush()
    status = stdout.channel.recv_exit_status()
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    print(f"Status: {status}")
    return status == 0, out, err

def clean_and_install():
    host = "192.168.0.73"
    user = "jetson"
    password = "jetson"
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(host, username=user, password=password)
        
        print("--- Step 0: Backup and PURGE sources.list.d ---")
        run_sudo_cmd(client, "mkdir -p /home/jetson/apt_backup", password)
        run_sudo_cmd(client, "mv -v /etc/apt/sources.list.d/* /home/jetson/apt_backup/", password)
        
        print("--- Step 1: Clean Update ---")
        run_sudo_cmd(client, "apt-get clean", password)
        run_sudo_cmd(client, "apt-get update", password)

        print("--- Step 2: Install Python 3.10 ---")
        run_sudo_cmd(client, "apt-get install -y software-properties-common", password)
        run_sudo_cmd(client, "add-apt-repository -y ppa:deadsnakes/ppa", password)
        run_sudo_cmd(client, "apt-get update", password)
        success, out, err = run_sudo_cmd(client, "apt-get install -y python3.10 python3.10-venv python3.10-dev git", password)
        if not success:
            print("Python 3.10 installation still failing. Checking cache...")
            run_sudo_cmd(client, "apt-cache policy python3.10", password)
            return

        print("--- Step 3: Install Node.js 18 ---")
        # Removing any traces of old nodejs
        run_sudo_cmd(client, "apt-get purge -y nodejs npm", password)
        run_sudo_cmd(client, "curl -fsSL https://deb.nodesource.com/setup_18.x | bash -", password)
        run_sudo_cmd(client, "apt-get install -y nodejs", password)

        print("--- Step 4: Verification ---")
        client.exec_command("python3.10 --version")
        stdin, stdout, stderr = client.exec_command("node --version")
        print(f"Node: {stdout.read().decode().strip()}")
        stdin, stdout, stderr = client.exec_command("npm --version")
        print(f"NPM: {stdout.read().decode().strip()}")

        print("--- Step 5: Setup Project ---")
        run_sudo_cmd(client, "rm -rf sweetbomb", password)
        client.exec_command("git clone https://github.com/realplayerc4/sweetbomb.git")
        time.sleep(5)

        print("Setting up Backend venv...")
        run_sudo_cmd(client, "cd /home/jetson/sweetbomb && python3.10 -m venv venv", password)
        run_sudo_cmd(client, "cd /home/jetson/sweetbomb && ./venv/bin/python -m pip install --upgrade pip", password)
        print("Installing Backend Requirements...")
        run_sudo_cmd(client, "cd /home/jetson/sweetbomb && ./venv/bin/python -m pip install -r requirements.txt", password)

        print("Setting up Frontend...")
        run_sudo_cmd(client, "cd /home/jetson/sweetbomb/ui/frontend && npm install --registry=https://registry.npmmirror.com", password)
        run_sudo_cmd(client, "cd /home/jetson/sweetbomb/ui/frontend && npm run build", password)

        print("\nClean Deployment finished.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    clean_and_install()
