# Unitree G1 Remote Camera Client

This repository contains a graphical client (g1_client.py) for connecting to and monitoring the camera streams of a Unitree G1 humanoid robot over a local network. 

![g1 client](../assets/client.png)

## System Overview
The client is designed to interface with a custom server running on the robot. The system architecture operates as follows:
1. A server script (just been included in this repository as g1_server.py) runs inside a Docker container directly on the Unitree G1 robot. 
2. The server captures video frames from both the internal RealSense camera SDK and a secondary USB camera attached to the robot.
3. The server compresses these frames into JPEG format and broadcasts them over the local network using the ZeroMQ (ZMQ) Publish-Subscribe protocol.
4. This Python client connects to the robot's IP address, subscribes to the ZMQ streams, and renders the video in a low-latency Tkinter GUI.
5. The server also captures real-time audio from the USB webcam microphone (Logitech C270) and streams it via Port 6003.
6. The client features a TTS (Text-To-Speech) panel, allowing users to send text strings that the robot will play through its internal speakers.
7. If using mujoco, use its own client. There's no "mujoco server" since it's not needed. See how to run the simulation in the mujoco subdirectory.

## Prerequisites
To run the client, you need Python 3 and the following dependencies:

```bash
# Python libraries
pip install pyzmq opencv-python Pillow numpy sounddevice

# System library (Required for audio processing)
sudo apt-get install libportaudio2
```

## How to Use the Client
1. Ensure the Unitree G1 is powered on and connected to the same local network as your computer.
2. Ask me to initialize the Docker container on the robot and start the server script. Or if you want to do it yourself for testing, do:
	- Double click on the robot battery (if it's not ON already)
	- Once it talks (before it talks, it won't let you do anything), run
	```bash
	ssh -X unitree@192.168.1.126
	```
	- It will ask for a password, type '123'
	- Once inside, we will open docker with
	```bash
	docker_humble
	```
	- Once we're in docker, run
	```bash
	python3 g1_server.py
	```
	- Now the robot is ready
3. Run the client on your machine:
```bash
python g1_client.py
```
4. Use the dropdown menu at the top of the interface to switch seamlessly between the main RealSense camera and the secondary USB camera (probably you will only use the USB one, but it is good to have as many options as possible!)
5. If you're using the mujoco client, you'll only need the teleoperation WASD part (for now). For using movement go to the manipulation subdirectory.

## Robot Locomotion and Control
The graphical interface includes panels for arm actuation and lower-body locomotion (Zero Torque, Damping, Squatting, and Standing). Because the Unitree G1 requires specific state machine transitions to operate safely without falling, these controls must be used strictly following the designated operational flow. For example, if the robot is set using the Rack, it won't be able to perform prefabricated moves. It would be required to start from Squat, get up, and then perform. However, if we don't want to use the prefabricated moves (for example we want to use any move I could code) it is required to enter into Rack mode. 

If you need to move the robot or manipulate its arms, please ask me first before clicking any command buttons.

## Developing Custom Camera Clients
If you wish to process the video streams for computer vision tasks without using the provided GUI, you can easily write your own Python scripts. 

The server broadcasts the video on two ports:
* **Port 6001:** Main RealSense Camera stream.
* **Port 6002:** Secondary USB Camera stream (`/dev/video6`).

## Audio Streaming and Voice (TTS)
The system supports bidirectional audio interaction:

* **Live Monitoring (Robot -> Client):** The client receives a continuous 16kHz Mono audio stream from the robot's environment. This provides "ears" to the pilot during remote operation.
* **Voice Synthesis (Client -> Robot):** Using the "Audio TTS" panel, you can type any phrase in English for the G1 to speak. 
    * **Jumpscare Mode:** Typing the keyword `jumpscare` in the TTS box triggers a high-volume custom sound effect (use with caution!).

### Technical Audio Specs
* **Port 6003:** Audio Stream (ZMQ SUB).
* **Format:** Raw PCM (float32), 16000 Hz, Mono.
* **Source:** Hardware device `hw:0,0` (C270 USB Webcam).

### IMPORTANT NOTE
For the USB Camera to work, it is required to be unplugged before even booting up the robot. Once it is booted up, we can safely plug the USB Camera port to the robot
**Audio Note:** The audio capture system relies on this USB camera's microphone. If the camera is not recognized as `hw:0,0`, the audio server will fail to start. You can check detected audio devices on the robot using `arecord -l`.
The streams are sent as raw byte arrays of JPEG-encoded images. Below is a minimal working example of how to subscribe to a stream and display it using OpenCV:

### Developing Custom Camera Clients

```python
import zmq
import cv2
import numpy as np

# Configuration
ROBOT_IP = "192.168.123.164"
PORT = "6002" # Change to 6001 for the prebuilt camera

# Initialize ZeroMQ Subscriber
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt(zmq.CONFLATE, 1) # Only keep the most recent frame to reduce latency
socket.setsockopt_string(zmq.SUBSCRIBE, "")
socket.connect(f"tcp://{ROBOT_IP}:{PORT}")

print(f"Subscribed to video stream at tcp://{ROBOT_IP}:{PORT}")

while True:
    try:
        # Receive the byte array (non-blocking)
        frame_bytes = socket.recv(zmq.NOBLOCK)
        
        # Decode the JPEG buffer into an OpenCV image matrix
        np_img = np.frombuffer(frame_bytes, dtype=np.uint8)
        frame = cv2.imdecode(np_img, cv2.IMREAD_COLOR)

        if frame is not None:
            cv2.imshow("Unitree G1 Stream", frame)

        # Press 'q' to exit
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break

    except zmq.Again:
        # No new frame received yet, continue loop
        pass
    except Exception as e:
        print(f"Stream error: {e}")
        break

# Cleanup
socket.close()
cv2.destroyAllWindows()
```

### Developing Custom Audio Clients
You can subscribe to the audio feed independently. Below is a minimal subscriber:

```python
import zmq
import numpy as np
import sounddevice as sd

# Configuration
ROBOT_IP = "192.168.123.164"
PORT = "6003"

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect(f"tcp://{ROBOT_IP}:{PORT}")
socket.setsockopt_string(zmq.SUBSCRIBE, "")

# Open a real-time output stream on your local speakers
with sd.OutputStream(samplerate=16000, channels=1, dtype='float32') as stream:
    print("Listening to G1...")
    while True:
        raw_audio = socket.recv()
        data = np.frombuffer(raw_audio, dtype='float32')
        stream.write(data)
```

Multiple clients can run this exact script simultaneously on the same network without causing latency or connection drops on the robot's server.

---

If there's anything you need, just ask!
**Audio Note:** The audio capture system relies on this USB camera's microphone. If the camera is not recognized as `hw:0,0`, the audio server will fail to start. You can check detected audio devices on the robot using `arecord -l`.
