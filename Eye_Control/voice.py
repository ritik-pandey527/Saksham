import speech_recognition as sr
from gtts import gTTS
import os
import datetime
import random
import subprocess
import signal
from twilio.rest import Client

# Twilio Credentials (Replace with actual values)
TWILIO_ACCOUNT_SID = "ACe288dadd8211d574d2e1ca484fa4b180"
TWILIO_AUTH_TOKEN = "5c890273bcc843d5d6640bcffd2245bf"
TWILIO_PHONE_NUMBER = "+19786259680"
RECIPIENT_PHONE_NUMBER = "+918169872235"  # Number to receive the SMS

# Global variables
active_process = None
is_active = False  # Tracks assistant activation

# Function to print highlighted text
def print_highlighted(text):
    print(f"***************** {text} *****************")

# Function to convert text to speech
def speak(text):
    print_highlighted(text)
    tts = gTTS(text=text, lang='en')
    tts.save("output.mp3")
    os.system("mpg321 output.mp3")  # Ensure mpg321 is installed

# Function to get the current time
def get_time():
    return datetime.datetime.now().strftime("%H:%M")

# Function to stop any running process
def stop_process():
    global active_process
    if active_process:
        os.kill(active_process.pid, signal.SIGTERM)
        active_process = None
        return "Process stopped."
    return "No process is currently running."

# Function to send an emergency SMS using Twilio
def send_help_sms():
    try:
        client = Client(TWILIO_ACCOUNT_SID, TWILIO_AUTH_TOKEN)
        message = client.messages.create(
            body="Emergency! Assistance is needed.",
            from_=TWILIO_PHONE_NUMBER,
            to=RECIPIENT_PHONE_NUMBER
        )
        return "Help message sent successfully."
    except Exception as e:
        return f"Failed to send help message: {str(e)}"

# Function to open eye.py
def open_eye_script():
    global active_process
    try:
        active_process = subprocess.Popen(["python", "iris_detect_tflite_ball.py"])  # Adjust to "python" if using Windows
        return "Opening eye control system."
    except Exception as e:
        return f"Failed to open eye control system: {str(e)}"

# Function to process user commands
def respond(command):
    global is_active

    command = command.lower()

    if "google" in command or is_active:
        if not is_active:
            speak("Hello there!")
            is_active = True  # Activate assistant

        if "time" in command:
            speak(f"The current time is {get_time()}.")

        elif "hello" in command:
            speak(random.choice(["Hello there!", "Hi, how can I help you?", "Greetings!"]))

        elif "your name" in command:
            speak("I am your voice assistant.")

        elif "how are you" in command:
            speak(random.choice(["I'm just a program, but thanks for asking!", "Doing well, how about you?"]))

        elif "help" in command:
            speak("Sending emergency help message.")
            speak(send_help_sms())

        elif "stop" in command:
            speak(stop_process())

        elif "eye" in command and ("open" in command or "start" in command):  # Voice command to open eye.py
            speak(open_eye_script())

        else:
            speak("I'm sorry, I didn't understand that.")

# Main function for speech recognition
def main():
    recognizer = sr.Recognizer()
    global is_active

    with sr.Microphone() as source:
        print_highlighted("Listening for your command...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

        try:
            command = recognizer.recognize_google(audio)
            print_highlighted(f"You said: {command}")
            respond(command)
        except sr.UnknownValueError:
            print_highlighted("Sorry, I could not understand the audio.")
        except sr.RequestError as e:
            print_highlighted(f"Could not request results from Google Speech Recognition service; {e}")

# Run the assistant continuously
if __name__ == "__main__":
    while True:
        main()
