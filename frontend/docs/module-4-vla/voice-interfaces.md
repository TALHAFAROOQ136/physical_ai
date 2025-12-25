---
sidebar_position: 2
title: Voice Interfaces
description: Speech recognition and synthesis for robots
---

# Voice Interfaces

Voice interfaces enable natural human-robot interaction through speech.

## Learning Objectives

- Implement speech-to-text with Whisper
- Create text-to-speech output
- Build a voice command system
- Handle streaming audio

## Speech-to-Text with Whisper

```python
import whisper

# Load model
model = whisper.load_model("base")

# Transcribe audio
def transcribe_audio(audio_path: str) -> str:
    result = model.transcribe(audio_path)
    return result["text"]
```

## Real-time Transcription

```python
import sounddevice as sd
import numpy as np

def listen_for_command(duration: float = 5.0) -> str:
    """Record audio and transcribe."""
    sample_rate = 16000

    # Record audio
    audio = sd.rec(
        int(duration * sample_rate),
        samplerate=sample_rate,
        channels=1,
        dtype=np.float32
    )
    sd.wait()

    # Transcribe
    result = model.transcribe(audio.flatten())
    return result["text"]
```

## Text-to-Speech

```python
from gtts import gTTS
import pygame

def speak(text: str):
    """Convert text to speech and play."""
    tts = gTTS(text=text, lang='en')
    tts.save("response.mp3")

    pygame.mixer.init()
    pygame.mixer.music.load("response.mp3")
    pygame.mixer.music.play()
```

## ROS 2 Voice Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VoiceCommandNode(Node):
    def __init__(self):
        super().__init__('voice_command')
        self.publisher = self.create_publisher(
            String, '/voice_command', 10
        )
        self.timer = self.create_timer(5.0, self.listen_callback)

    def listen_callback(self):
        command = listen_for_command()
        msg = String()
        msg.data = command
        self.publisher.publish(msg)
        self.get_logger().info(f'Command: {command}')
```

## Exercise

Create a voice interface that:
1. Listens for "robot" wake word
2. Transcribes the following command
3. Publishes to a ROS 2 topic

---

Continue to [LLM Planning](/docs/module-4-vla/llm-planning).
