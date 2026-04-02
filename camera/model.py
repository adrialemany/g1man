# 💡 Version: 56.60%
import torch
import torch.nn as nn
import timm
from transformers import Wav2Vec2Model


class AudioOnlyModel(nn.Module):
    def __init__(self, num_classes=6):
        super().__init__()

        self.wav2vec = Wav2Vec2Model.from_pretrained(
            "facebook/wav2vec2-base"
        )

        # 🔥 freeze 먼저 (과적합 방지)
        for param in self.wav2vec.parameters():
            param.requires_grad = True
            
        # for name, param in self.wav2vec.named_parameters():
        #     if "encoder.layers.8" in name or "encoder.layers.9" in name or "encoder.layers.10" in name or "encoder.layers.10" in name or "encoder.layers.11" in name:
        #         param.requires_grad = True
        #     else:
        #         param.requires_grad = False

        hidden_dim = self.wav2vec.config.hidden_size  # 768

        # 🔥 attention pooling
        self.attention = nn.Sequential(
            nn.Linear(hidden_dim, 128),
            nn.Tanh(),
            nn.Linear(128, 1)
        )

        self.classifier = nn.Sequential(
            nn.Linear(hidden_dim, 512),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(512, num_classes)
        )

    def forward(self, wav):
        """
        wav: (B, T)
        """

        outputs = self.wav2vec(wav)
        features = outputs.last_hidden_state  # (B, L, 768)

        # 🔥 Attention pooling
        attn_weights = self.attention(features)  # (B, L, 1)
        attn_weights = torch.softmax(attn_weights, dim=1)
        
        mean_feat = features.mean(dim=1)
        attn_feat = torch.sum(features * attn_weights, dim=1)
        
        pooled = 0.5 * mean_feat + 0.5 * attn_feat

        # pooled = torch.sum(features * attn_weights, dim=1)  # (B, 768)

        logits = self.classifier(pooled)
        return logits
    

# 💡 Version: 36.93%
class VisualOnlyModel(nn.Module):
    def __init__(self, num_classes=6):
        super().__init__()

        # 🔥 Stronger backbone
        self.backbone = timm.create_model(
            "swin_base_patch4_window7_224",
            pretrained=True
        )
        self.backbone.reset_classifier(0)

        embed_dim = self.backbone.num_features  # 1024 (swin-base)
        
        for name, param in self.backbone.named_parameters():
            param.requires_grad = True

        # # 🔥 Freeze 대부분, 마지막 stage만 열기
        # for name, param in self.backbone.named_parameters():
        #     if "layers.3" in name:  # 마지막 stage
        #         param.requires_grad = True
        #     else:
        #         param.requires_grad = False

        # 🔥 Temporal modeling (BiGRU)
        self.temporal = nn.GRU(
            input_size=embed_dim,
            hidden_size=512,
            num_layers=2,
            batch_first=True,
            bidirectional=True,
            dropout=0.3
        )

        self.dropout = nn.Dropout(0.6)

        self.classifier = nn.Sequential(
            nn.Linear(1024, 512),
            nn.ReLU(),
            nn.Dropout(0.6),
            nn.Linear(512, num_classes)
        )

    def forward(self, frames):

        B, T, C, H, W = frames.shape
        frames = frames.view(B*T, C, H, W)

        feat = self.backbone.forward_features(frames)
        feat = self.backbone.forward_head(feat, pre_logits=True)

        feat = feat.view(B, T, -1)

        out, _ = self.temporal(feat)

        pooled = out.mean(dim=1)

        pooled = self.dropout(pooled)

        return self.classifier(pooled)


class AVFusionModel(nn.Module):
    def __init__(self, audio_model, visual_model, num_classes=6):
        super().__init__()

        # Encoders
        self.audio_encoder = audio_model.wav2vec
        self.visual_backbone = visual_model.backbone
        self.visual_temporal = visual_model.temporal

        self.audio_dim = 768
        self.visual_dim = 1024
        self.fusion_dim = 512

        # Projection
        self.audio_proj = nn.Linear(self.audio_dim, self.fusion_dim)
        self.visual_proj = nn.Linear(self.visual_dim, self.fusion_dim)

        # 🔥 Bidirectional Cross Attention
        self.audio_to_visual = nn.MultiheadAttention(
            embed_dim=self.fusion_dim,
            num_heads=8,
            batch_first=True
        )

        self.visual_to_audio = nn.MultiheadAttention(
            embed_dim=self.fusion_dim,
            num_heads=8,
            batch_first=True
        )

        self.norm_a = nn.LayerNorm(self.fusion_dim)
        self.norm_v = nn.LayerNorm(self.fusion_dim)

        # 🔥 Reliability Gate
        self.gate = nn.Sequential(
            nn.Linear(self.fusion_dim * 2, self.fusion_dim),
            nn.ReLU(),
            nn.Linear(self.fusion_dim, 1),
            nn.Sigmoid()
        )

        # # Classifier
        # self.classifier = nn.Sequential(
        #     nn.Linear(self.fusion_dim, 256),
        #     nn.ReLU(),
        #     nn.Dropout(0.5),
        #     nn.Linear(256, num_classes)
        # )
        
        self.classifier = nn.Sequential(
            nn.Linear(self.fusion_dim, 256),  # 🔥 512 → 1024
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(256, num_classes)
        )

    def forward(self, wav, frames):

        # ======================
        # 🔊 AUDIO
        # ======================
        audio_out = self.audio_encoder(wav)
        audio_seq = self.audio_proj(audio_out.last_hidden_state)  # (B, La, 512)

        # ======================
        # 👁 VISUAL
        # ======================
        B, T, C, H, W = frames.shape
        frames = frames.view(B*T, C, H, W)

        v_feat = self.visual_backbone.forward_features(frames)
        v_feat = self.visual_backbone.forward_head(v_feat, pre_logits=True)
        v_feat = v_feat.view(B, T, -1)

        v_feat, _ = self.visual_temporal(v_feat)
        visual_seq = self.visual_proj(v_feat)  # (B, Lv, 512)

        # ======================
        # 🔥 CROSS ATTENTION
        # ======================

        # Audio queries Visual
        a2v, _ = self.audio_to_visual(
            query=audio_seq,
            key=visual_seq,
            value=visual_seq
        )
        a2v = self.norm_a(a2v + audio_seq)

        # Visual queries Audio
        v2a, _ = self.visual_to_audio(
            query=visual_seq,
            key=audio_seq,
            value=audio_seq
        )
        v2a = self.norm_v(v2a + visual_seq)

        # ======================
        # 🔥 Pooling
        # ======================
        audio_pool = a2v.mean(dim=1)
        visual_pool = v2a.mean(dim=1)
        
        
        # Discrepancy feature #추가
        delta = torch.abs(audio_pool - visual_pool)

        # class-wise gate
        gate_input = torch.cat([audio_pool, visual_pool], dim=1)
        g = self.gate(gate_input)  # (B, fusion_dim)

        fused = g * audio_pool + (1 - g) * visual_pool

        # final concat
        final_feat = fused #torch.cat([fused, delta], dim=1)

        logits = self.classifier(final_feat)


        # # ======================
        # # 🔥 Reliability Gate
        # # ======================
        # gate_input = torch.cat([audio_pool, visual_pool], dim=1)
        # g = self.gate(gate_input)

        # fused = g * audio_pool + (1 - g) * visual_pool

        # # ======================
        # # 🔥 Classification
        # # ======================
        # logits = self.classifier(fused)

        return logits
