#!/bin/bash
# Script to run send.py with the virtual environment activated and sudo

cd "$(dirname "$0")"

# Use the virtual environment's python with sudo
sudo .venv/bin/python3 send.py
