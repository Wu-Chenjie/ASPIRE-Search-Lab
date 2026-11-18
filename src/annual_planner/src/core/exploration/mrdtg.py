"""Exact required helper subset from original Annual-Planner; no alternate planner."""
import hashlib,json


def observation_grid(runtime):
    description = [list(map(int, runtime.shape)), runtime.origin.tolist(), float(runtime.resolution)]
    return hashlib.sha256(json.dumps(description, separators=(',', ':')).encode()).hexdigest()
