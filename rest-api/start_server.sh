#!/bin/bash

# Start the FastAPI server
./venv/bin/python -m uvicorn main:combined_app --host 0.0.0.0 --port 8000