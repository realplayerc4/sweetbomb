#!/bin/bash

# Start the FastAPI server
./venv/bin/python -u -m uvicorn main:combined_app --host 0.0.0.0 --port 8000 > server.log 2>&1