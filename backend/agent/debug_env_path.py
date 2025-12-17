
import os
import sys

current_dir = os.path.dirname(os.path.abspath(__file__))
env_path = os.path.join(current_dir, "../.env")
abs_env = os.path.abspath(env_path)

print(f"Current Dir: {current_dir}")
print(f"Target Env Path: {env_path}")
print(f"Absolute Env Path: {abs_env}")
print(f"Exists? {os.path.exists(abs_env)}")

try:
    with open(abs_env, 'r') as f:
        print("--- ENV CONTENT START ---")
        for i, line in enumerate(f):
            print(f"{i+1}: {repr(line)}")
        print("--- ENV CONTENT END ---")

except Exception as e:
    print(f"Read Error: {e}")
