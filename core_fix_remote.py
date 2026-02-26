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

def core_fix_and_install():
    host = "192.168.0.73"
    user = "yq"
    password = "1"
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(host, username=user, password=password)
        
        print("--- Step 0: Fix main sources.list ---")
        # Commenting out any line that contains download.docker.com which is failing
        run_sudo_cmd(client, "sed -i '/download.docker.com/s/^/#/' /etc/apt/sources.list", password)
        # Also aliyun docker just in case
        run_sudo_cmd(client, "sed -i '/mirrors.aliyun.com\/docker-ce/s/^/#/' /etc/apt/sources.list", password)
        
        print("--- Step 1: Clean Update ---")
        run_sudo_cmd(client, "apt-get update", password)

        print("--- Step 2: Install Python 3.10 ---")
        run_sudo_cmd(client, "add-apt-repository -y ppa:deadsnakes/ppa", password)
        run_sudo_cmd(client, "apt-get update", password)
        success = run_sudo_cmd(client, "apt-get install -y python3.10 python3.10-venv python3.10-dev git", password)[0]
        if not success:
            print("Python 3.10 installation FAILED again.")
            return

        print("--- Step 3: Install Node.js 18 ---")
        run_sudo_cmd(client, "apt-get purge -y nodejs npm", password)
        run_sudo_cmd(client, "curl -fsSL https://deb.nodesource.com/setup_18.x | bash -", password)
        run_sudo_cmd(client, "apt-get install -y nodejs", password)

        print("--- Step 4: Setup Project ---")
        run_sudo_cmd(client, "rm -rf sweetbomb", password)
        client.exec_command("git clone https://github.com/realplayerc4/sweetbomb.git")
        # Wait for clone
        time.sleep(5)
        
        print("Backend Setup...")
        run_sudo_cmd(client, "cd sweetbomb/rest-api && python3.10 -m venv venv", password)
        run_sudo_cmd(client, "cd sweetbomb/rest-api && ./venv/bin/python -m pip install --upgrade pip", password)
        run_sudo_cmd(client, "cd sweetbomb/rest-api && ./venv/bin/python -m pip install -r requirements.txt", password)

        print("Frontend Setup...")
        # Use mirror for speed
        run_sudo_cmd(client, "cd sweetbomb/rest-api/ui/frontend && npm install --registry=https://registry.npmmirror.com", password)
        run_sudo_cmd(client, "cd sweetbomb/rest-api/ui/frontend && npm run build", password)

        print("\nSUCCESS: Migration complete.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    core_fix_and_install()
