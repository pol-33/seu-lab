#!/bin/bash
# Script to run send.py with the virtual environment activated

cd "$(dirname "$0")"
source .venv/bin/activate
python3 send.py
