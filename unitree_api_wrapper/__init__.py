import sys
import os

# make library available internally
path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "lib")
sys.path.append(path)


def get_policy_path(name):
    return os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "policies", name)
