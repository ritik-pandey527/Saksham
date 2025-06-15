from flask import Flask, render_template, request
import serial

app = Flask(__name__)

# Change this if needed: Check ESP32's port with ls /dev/tty*
SERIAL_PORT = "/dev/ttyUSB0"  # Or "/dev/ttyACM0" depending on ESP32
BAUD_RATE = 115200

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Error opening serial port: {e}")
    ser = None

@app.route("/")
def index():
    return render_template("index.html")

@app.route("/set_mode", methods=["POST"])
def set_mode():
    mode = request.form["mode"]
    if ser:
        ser.write((mode + "\n").encode())  # Send mode via USB
    return f"Mode changed to: {mode}"

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000, debug=True)
