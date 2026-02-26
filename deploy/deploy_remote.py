import paramiko
import time

def run_sudo_cmd(client, cmd, password):
    print(f"Executing: {cmd}")
    stdin, stdout, stderr = client.exec_command(f"sudo -S {cmd}")
    stdin.write(password + '\n')
    stdin.flush()
    
    # Wait for execution and read output
    exit_status = stdout.channel.recv_exit_status()
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    
    if exit_status == 0:
        print(f"SUCCESS: {cmd}")
        return True, out
    else:
        print(f"FAILED: {cmd} (Exit: {exit_status})")
        print(f"ERR: {err}")
        return False, err

def deploy():
    host = "192.168.0.73"
    user = "yq"
    password = "1"
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(host, username=user, password=password)
        
        # Step 1: Bootstrap Python 3.10
        print("--- Bootstrapping Python 3.10 ---")
        run_sudo_cmd(client, "apt-get update", password)
        run_sudo_cmd(client, "apt-get install -y software-properties-common", password)
        run_sudo_cmd(client, "add-apt-repository -y ppa:deadsnakes/ppa", password)
        run_sudo_cmd(client, "apt-get update", password)
        run_sudo_cmd(client, "apt-get install -y python3.10 python3.10-venv python3.10-dev git", password)
        
        # Step 2: Bootstrap Node.js 18
        print("--- Bootstrapping Node.js 18 ---")
        run_sudo_cmd(client, "curl -fsSL https://deb.nodesource.com/setup_18.x | bash -", password)
        run_sudo_cmd(client, "apt-get install -y nodejs", password)
        
        # Step 3: Clone Repository
        print("--- Cloning Repository ---")
        client.exec_command("rm -rf sweetbomb") # Clean start
        stdin, stdout, stderr = client.exec_command("git clone https://github.com/realplayerc4/sweetbomb.git")
        if stdout.channel.recv_exit_status() != 0:
            print("Git clone failed!")
            print(stderr.read().decode())
            return
        
        # Step 4: Setup Backend
        print("--- Setting up Backend ---")
        setup_cmds = [
            "cd sweetbomb && python3.10 -m venv venv",
            "cd sweetbomb && ./venv/bin/python -m pip install --upgrade pip",
            "cd sweetbomb && ./venv/bin/python -m pip install -r requirements.txt"
        ]
        for cmd in setup_cmds:
            stdin, stdout, stderr = client.exec_command(cmd)
            if stdout.channel.recv_exit_status() != 0:
                print(f"Failed cmd: {cmd}")
                print(stderr.read().decode())
                break
        
        # Step 5: Setup Frontend
        print("--- Setting up Frontend ---")
        npm_cmds = [
            "cd sweetbomb/ui/frontend && npm install --registry=https://registry.npmmirror.com"
        ]
        for cmd in npm_cmds:
            print(f"Executing: {cmd} (this may take a while...)")
            stdin, stdout, stderr = client.exec_command(cmd)
            # Npm install can be very slow, we might need to wait or monitor
            if stdout.channel.recv_exit_status() != 0:
                print(f"Failed cmd: {cmd}")
                print(stderr.read().decode())
                break
        
        print("\nDeployment sequence finished.")
        
    except Exception as e:
        print(f"Error during deployment: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    deploy()
