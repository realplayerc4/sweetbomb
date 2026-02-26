import paramiko
import time
import os

def run_cmd(client, cmd, description=None, get_output=False):
    if description:
        print(f"--- {description} ---")
    print(f"Executing: {cmd}")
    stdin, stdout, stderr = client.exec_command(cmd)
    
    exit_status = stdout.channel.recv_exit_status()
    out = stdout.read().decode().strip()
    err = stderr.read().decode().strip()
    
    if exit_status == 0:
        print(f"SUCCESS: {cmd.split()[0]}")
        return (True, out) if get_output else True
    else:
        print(f"FAILED: {cmd.split()[0]} (Exit: {exit_status})")
        if out: print(f"STDOUT: {out}")
        if err: print(f"STDERR: {err}")
        return (False, err) if get_output else False

def portable_deploy():
    host = "192.168.0.73"
    user = "yq"
    password = "1"
    
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(host, username=user, password=password)
        
        conda_bin = "$HOME/miniconda3/bin/conda"
        node_path = "$HOME/node-v18/bin"

        # 1. Configure Conda
        print("--- Configuring Conda ---")
        run_cmd(client, f"{conda_bin} config --add channels conda-forge")
        run_cmd(client, f"{conda_bin} config --set channel_priority strict")
        run_cmd(client, f"{conda_bin} tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main")

        # 2. Create Conda Env with heavy dependencies
        print("--- Creating Python 3.10 Env with Binary Dependencies ---")
        # We install av and opencv via conda to avoid compilation issues with FFmpeg
        run_cmd(client, f"{conda_bin} create -y -n rs_env python=3.10 av opencv pyrealsense2 numpy -c conda-forge")
        
        env_python = "$HOME/miniconda3/envs/rs_env/bin/python"
        env_pip = "$HOME/miniconda3/envs/rs_env/bin/pip"

        # 3. Install remaining Backend Deps via pip
        print("--- Installing Remaining Backend Deps ---")
        run_cmd(client, f"{env_pip} install --upgrade pip")
        # requirements.txt contains many things, pip will skip those already installed by conda
        pip_install_cmd = f"cd sweetbomb/rest-api && {env_pip} install -r requirements.txt -i https://pypi.tuna.tsinghua.edu.cn/simple"
        run_cmd(client, pip_install_cmd)

        # 4. Setup Frontend
        print("--- Setting up Frontend ---")
        frontend_setup = (
            f"export PATH={node_path}:$PATH && "
            "cd sweetbomb/rest-api/ui/frontend && "
            "npm install --registry=https://registry.npmmirror.com && "
            "npm run build"
        )
        run_cmd(client, frontend_setup, "npm install & build")

        print("\nPortable Deployment SUCCESSFUL.")
        print(f"Backend Python: {env_python}")

    except Exception as e:
        print(f"CRITICAL ERROR: {e}")
    finally:
        client.close()

if __name__ == "__main__":
    portable_deploy()
