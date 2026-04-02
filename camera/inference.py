import torch
import cv2
from collections import deque
from PIL import Image
from torchvision import transforms

from model import AudioOnlyModel, VisualOnlyModel, AVFusionModel


class EmotionInference:
    def __init__(self, ckpt_path, device="cuda"):
        self.device = torch.device(device if torch.cuda.is_available() else "cpu")

        audio_model = AudioOnlyModel().to(self.device)
        visual_model = VisualOnlyModel().to(self.device)
        self.model = AVFusionModel(audio_model, visual_model).to(self.device)

        ckpt = torch.load(ckpt_path, map_location=self.device)
        self.model.load_state_dict(ckpt)
        self.model.eval()

        self.frame_buffer = deque(maxlen=16)

        self.transform = transforms.Compose([
            transforms.Resize((224,224)),
            transforms.ToTensor(),
            transforms.Normalize([0.485,0.456,0.406],
                                 [0.229,0.224,0.225])
        ])

    def update_frame(self, frame):
        self.frame_buffer.append(frame)

    def preprocess_frames(self):
        frames = list(self.frame_buffer)

        processed = []
        for f in frames:
            img = Image.fromarray(cv2.cvtColor(f, cv2.COLOR_BGR2RGB))
            img = self.transform(img)
            processed.append(img)

        frames_tensor = torch.stack(processed)
        frames_tensor = frames_tensor.unsqueeze(0).to(self.device)

        return frames_tensor

    def predict(self, wav_tensor):
        # if len(self.frame_buffer) < 16:
        #     return None

        frames_tensor = self.preprocess_frames()

        with torch.no_grad():
            logits = self.model(wav_tensor, frames_tensor)
            pred = torch.argmax(logits, dim=1).item()

        return pred
