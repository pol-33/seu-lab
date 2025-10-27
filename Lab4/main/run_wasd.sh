#!/bin/bash
# Wrapper script to run send.py with proper environment

cd "$(dirname "$0")"

# Activate virtual environment
source .venv/bin/activate

# Run with sudo, preserving the virtual environment's Python
sudo -E .venv/bin/python3 send.py
