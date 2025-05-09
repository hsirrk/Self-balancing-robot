import multiprocessing
import subprocess

def run_nn():
    """Runs the hand gesture detection script (nn.py)"""
    subprocess.run(["python", "app.py"])

def run_ble():
    """Runs the Bluetooth communication script (ble.py)"""
    subprocess.run(["python", "ble.py"])

if __name__ == "__main__":
    multiprocessing.set_start_method("spawn")  # Ensure compatibility on Windows
    
    # Create processes
    nn_process = multiprocessing.Process(target=run_nn)
    ble_process = multiprocessing.Process(target=run_ble)

    # Start processes
    nn_process.start()
    ble_process.start()

    # Wait for processes to finish
    nn_process.join()
    ble_process.join()
