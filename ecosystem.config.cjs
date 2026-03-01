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
    // Backend: FastAPI/uvicorn
    {
      name: 'sweetbomb-8000',
      cwd: '.',
      script: 'start.cjs',
      interpreter: 'node',
      env: { PYTHONUNBUFFERED: '1' }
    }
  ]
}
