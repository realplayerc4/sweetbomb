module.exports = {
  apps: [
    {
      name: 'sweetbomb-8000',
      script: './main.py',
      interpreter: '/usr/bin/python3',
      args: 'api',
      cwd: '/home/jetson/sweetbomb',
      env: {
        NODE_ENV: 'production',
        PYTHONPATH: '/home/jetson/sweetbomb'
      },
      autorestart: true,
      watch: false,
      max_memory_restart: '1G',
      log_type: 'json',
      log_max_size: '10M',
      log_rotate_interval: '1d',
      log_retention: '7d',
      out_file: '/home/jetson/.pm2/logs/sweetbomb-8000-out.log',
      error_file: '/home/jetson/.pm2/logs/sweetbomb-8000-error.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss Z'
    },
    {
      name: 'sweetbomb-5173',
      script: 'npm',
      args: 'run dev -- --port 5173 --host',
      cwd: '/home/jetson/sweetbomb/ui/frontend',
      env: {
        NODE_ENV: 'development',
        VITE_API_BASE_URL: 'http://localhost:8000'
      },
      autorestart: true,
      watch: false,
      log_type: 'json',
      log_max_size: '10M',
      log_rotate_interval: '1d',
      log_retention: '7d',
      out_file: '/home/jetson/.pm2/logs/sweetbomb-5173-out.log',
      error_file: '/home/jetson/.pm2/logs/sweetbomb-5173-error.log',
      log_date_format: 'YYYY-MM-DD HH:mm:ss Z'
    }
  ]
};
