module.exports = {
  apps: [
    // Frontend: Vite
    {
      name: 'sweetbomb-5173',
      cwd: './ui/frontend',
      script: 'node_modules/vite/bin/vite.js',
      args: '--port 5173 --host 0.0.0.0',
      interpreter: 'node',
      env: { NODE_ENV: 'development' }
    },
    // Backend: FastAPI/uvicorn (using python3 directly)
    {
      name: 'sweetbomb-8000',
      cwd: '.',
      script: 'python3',
      args: '-m uvicorn main:combined_app --host 0.0.0.0 --port 8000 --log-level debug',
      interpreter: 'none',
      env: { PYTHONUNBUFFERED: '1' }
    }
  ]
}
