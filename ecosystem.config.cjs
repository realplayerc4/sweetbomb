module.exports = {
  apps: [
    {
      name: 'sweetbomb-8000',
      script: 'python3',
      args: '-m uvicorn main:combined_app --host 0.0.0.0 --port 8000 --reload',
      cwd: '/home/jetson/sweetbomb',
      env: {
        PYTHONPATH: '/home/jetson/sweetbomb'
      },
      log_file: '/home/jetson/.pm2/logs/sweetbomb-8000.log',
      error_file: '/home/jetson/.pm2/logs/sweetbomb-8000-error.log',
      combine_logs: true,
      watch: false
    },
    {
      name: 'sweetbomb-5173',
      cwd: '/home/jetson/sweetbomb/ui/frontend',
      script: 'npm',
      args: 'run dev',
      env: {
        NODE_ENV: 'development',
        VITE_API_BASE_URL: 'http://localhost:8000'
      },
      log_file: '/home/jetson/.pm2/logs/sweetbomb-5173.log',
      error_file: '/home/jetson/.pm2/logs/sweetbomb-5173-error.log',
      combine_logs: true
    }
  ]
};
