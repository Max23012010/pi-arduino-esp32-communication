import speech_recognition as sr
import pywhatkit
import datetime
import wikipedia
import pyjokes
from google import genai
from elevenlabs.client import ElevenLabs
from playsound import playsound
import tempfile
import os
import re

# ================== CONFIGURE GEMINI ==================
CLIENT = genai.Client(api_key="AIzaSyCNqJnz_vKvbIhW54sZju56ls18XR7TBV0")  # Replace with your Gemini API key

# ================== CONFIGURE ELEVENLABS ==================
ELEVENLABS_API_KEY = "sk_8729e7008ae87f322a44bb549529acad86c6953557c9a472"  # Replace with your ElevenLabs API key
VOICE_ID = "8PfKHL4nZToWC3pbz9U9"               # Female/anime-style voice
eleven = ElevenLabs(api_key=ELEVENLABS_API_KEY)

listener = sr.Recognizer()

# ================== TALK FUNCTION ==================
def talk(text):
    try:
        print("Samaira:", text)
        # Convert text to audio
        audio_gen = eleven.text_to_speech.convert(
            text=text,
            voice_id=VOICE_ID,
            model_id="eleven_multilingual_v2",
            output_format="mp3_44100_128"
        )
        audio_bytes = b"".join(audio_gen)

        # Use a temporary file for playback
        with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmp:
            tmp.write(audio_bytes)
            tmp_path = tmp.name

        playsound(tmp_path)
        os.remove(tmp_path)

    except Exception as e:
        print("Error in talk():", e)

# ================== LISTEN FUNCTION ==================
def take_command(timeout=None):
    try:
        with sr.Microphone() as source:
            listener.adjust_for_ambient_noise(source)
            print("🎧 Listening...")
            voice = listener.listen(source, timeout=timeout)
            command = listener.recognize_google(voice)
            command = command.lower().strip()
            print("You said:", command)
            return command
    except sr.WaitTimeoutError:
        return ""
    except sr.UnknownValueError:
        return ""
    except sr.RequestError:
        talk("Speech service is unavailable at the moment.")
        return ""
    except OSError:
        talk("Mic not found. Please check your microphone.")
        return ""

# ================== SIMPLE ANSWERS FUNCTION ==================
def simple_answer(command):
    # --- Basic Math ---
    math_match = re.match(r'what is (\d+)\s*([+\-*/])\s*(\d+)', command)
    if math_match:
        a, op, b = math_match.groups()
        a, b = int(a), int(b)
        if op == '+': result = a + b
        elif op == '-': result = a - b
        elif op == '*': result = a * b
        elif op == '/': result = a / b if b != 0 else "undefined (division by zero)"
        talk(f"{a} {op} {b} is {result}")
        return

    # --- Joke ---
    if "joke" in command:
        talk(pyjokes.get_joke())
        return

    # --- Wikipedia ---
    if "wikipedia" in command:
        query = command.replace("wikipedia", "").strip()
        if query:
            try:
                summary = wikipedia.summary(query, sentences=2)
                talk(summary)
            except Exception:
                talk("Sorry, I couldn't find information on that topic.")
        return

    # --- Play YouTube ---
    if "play" in command:
        song = command.replace("play", "").strip()
        talk(f"Playing {song} on YouTube")
        pywhatkit.playonyt(song)
        return

    # --- Time ---
    if "time" in command:
        now = datetime.datetime.now().strftime("%I:%M %p")
        talk(f"The time is {now}")
        return

    # --- Gemini AI Response ---
    try:
        response = CLIENT.models.generate_content(
            model="gemini-2.0-flash",
            contents=command
        )
        if response.text:
            talk(response.text)
        else:
            talk("Sorry, I couldn't generate a response.")
    except Exception as e:
        print("Gemini AI error:", e)
        talk("Sorry, I couldn't fetch a response from Gemini AI.")

# ================== MAIN LOOP ==================
if __name__ == "__main__":
    talk("Hello, I’m Samaira. How can I help you today?")
    while True:
        cmd = take_command()
        if cmd:
            simple_answer(cmd)
            # Pause until user says ok/okay/hey
            print("💡 Say 'ok', 'okay', or 'hey' when you're ready for the next question.")
            while True:
                cont = take_command(timeout=7)
                if any(word in cont for word in ["ok", "okay", "hey"]):
                    break
