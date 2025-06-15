import subprocess
import time
import sys

def start_processes():
    python_cmd = "python" if sys.platform.startswith("win") else "python3"
    voice_process = subprocess.Popen([python_cmd, "voice.py"])
    flask_process = subprocess.Popen([python_cmd, "server.py"])
    time.sleep(3)
    streamlit_process = subprocess.Popen(["streamlit", "run", "streamlit_ui.py"])
    eye_process = subprocess.Popen([python_cmd, "iris_detect_tflite_ball.py"])
    return voice_process, flask_process, streamlit_process, eye_process

def stop_processes(voice_process, flask_process, streamlit_process, eye_process):
    flask_process.terminate()
    voice_process.terminate()
    streamlit_process.terminate()
    eye_process.terminate()

if __name__ == "__main__":
    voice_process, flask_process, streamlit_process, eye_process = start_processes()
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping all processes...")
        stop_processes(voice_process, flask_process, streamlit_process, eye_process)