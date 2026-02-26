import paramiko
import time

def run_sudo_cmd(client, cmd, password):
    print(f"Executing: {cmd}")
    stdin, stdout, stderr = client.exec_command(f"sudo -S {cmd}")
    stdin.write(password + '\n')
    stdin.flush()
    # Read output to ensure completion
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    status = stdout.channel.recv_exit_status()
    print(f"Status: {status}")
    return status == 0, out, err

def fix_and_redeploy():
    host = "192.168.0.73"
    user = "yq"
    password = "1"
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(host, username=user, password=password)
        
        # Step 0: Fix GPG keys and redundant lists
        print("--- Fixing GPG keys and apt lists ---")
        # Removing redundant ros lists to avoid warnings
        run_sudo_cmd(client, "rm -f /etc/apt/sources.list.d/ros-fish.list", password)
        # Update ROS key
        run_sudo_cmd(client, "curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -", password)
        # Final attempt to update (ignoring failures if any specific repo is still broken but deadsnakes is added)
        run_sudo_cmd(client, "apt-get update", password)

        # Step 1: Re-install Python 3.10
        print("--- Re-installing Python 3.10 ---")
        run_sudo_cmd(client, "add-apt-repository -y ppa:deadsnakes/ppa", password)
        run_sudo_cmd(client, "apt-get update", password)
        success, out, err = run_sudo_cmd(client, "apt-get install -y python3.10 python3.10-venv python3.10-dev git", password)
        if not success:
            print("Failed to install Python 3.10. Trying without venv/dev...")
            run_sudo_cmd(client, "apt-get install -y python3.10", password)

        # Step 2: Re-install Node.js 18 properly
        print("--- Re-installing Node.js 18 ---")
        run_sudo_cmd(client, "apt-get remove -y nodejs npm", password)
        run_sudo_cmd(client, "curl -fsSL https://deb.nodesource.com/setup_18.x | bash -", password)
        run_sudo_cmd(client, "apt-get install -y nodejs", password)

        # Step 3: Verify versions
        print("--- Verifying Versions ---")
        client.exec_command("python3.10 --version")
        stdin, stdout, stderr = client.exec_command("node --version")
        print(f"Node: {stdout.read().decode().strip()}")
        stdin, stdout, stderr = client.exec_command("npm --version")
        print(f"NPM: {stdout.read().decode().strip()}")

        # Step 4: Clone and Setup again
        print("--- Cloning and Setup ---")
        run_sudo_cmd(client, "rm -rf sweetbomb", password)
        stdin, stdout, stderr = client.exec_command("git clone https://github.com/realplayerc4/sweetbomb.git")
        stdout.channel.recv_exit_status()
        
        print("--- Backend Setup ---")
        client.exec_command("cd sweetbomb/rest-api && python3.10 -m venv venv")
        time.sleep(2)
        client.exec_command("cd sweetbomb/rest-api && ./venv/bin/python -m pip install --upgrade pip")
        print("Installing requirements (can be slow)...")
        stdin, stdout, stderr = client.exec_command("cd sweetbomb/rest-api && ./venv/bin/python -m pip install -r requirements.txt")
        print(f"Pip status: {stdout.channel.recv_exit_status()}")

        print("--- Frontend Setup ---")
        print("Npm install (can be very slow)...")
        stdin, stdout, stderr = client.exec_command("cd sweetbomb/rest-api/ui/frontend && npm install --registry=https://registry.npmmirror.com")
        print(f"Npm status: {stdout.channel.recv_exit_status()}")

        print("\nFix and Deployment finished.")

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    fix_and_redeploy()
