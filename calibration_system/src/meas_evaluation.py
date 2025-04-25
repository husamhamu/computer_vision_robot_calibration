#!/usr/bin/env python3
import yaml
import numpy as np

def load_yaml(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def compute_statistics():
    errors = []
    
    for i in range(1, 16):
        meas_file = f"test/meas_{i}.yaml"
        my_meas_file = f"test/my_meas_{i}.yaml"
        
        meas_data = load_yaml(meas_file)
        my_meas_data = load_yaml(my_meas_file)
        
        meas_pos = np.array([meas_data['translation']['x'], meas_data['translation']['y'], meas_data['translation']['z']])
        my_meas_pos = np.array([my_meas_data['translation']['x'], my_meas_data['translation']['y'], my_meas_data['translation']['z']])
        
        error = np.abs(meas_pos - my_meas_pos)
        errors.append(error)
    
    errors = np.array(errors)
    mean_error = np.mean(errors, axis=0)
    std_error = np.std(errors, axis=0)
    min_error = np.min(errors, axis=0)
    max_error = np.max(errors, axis=0)
    
    print(f"Mean Position Error:\n x: {mean_error[0]:.6f}\n y: {mean_error[1]:.6f}\n z: {mean_error[2]:.6f}")
    print(f"Standard Deviation Error:\n x: {std_error[0]:.6f}\n y: {std_error[1]:.6f}\n z: {std_error[2]:.6f}")
    print(f"Min Position Error:\n x: {min_error[0]:.6f}\n y: {min_error[1]:.6f}\n z: {min_error[2]:.6f}")
    print(f"Max Position Error:\n x: {max_error[0]:.6f}\n y: {max_error[1]:.6f}\n z: {max_error[2]:.6f}")

if __name__ == "__main__":
    compute_statistics()
