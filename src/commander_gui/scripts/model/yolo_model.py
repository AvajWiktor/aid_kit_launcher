import numpy as np
import cv2
import torch
from PIL import ImageTk, Image

class YoloModel:
    def __init__(self):
        # self.yolo_cfg_path = yolo_cfg_path
        # self.yolo_weights = yolo_weights
        # self.model = torch.nn.Module()
        # self.model.cpu()
        # device = torch.device("cpu")
        # self.model.to(device)
        # self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')#, map_location=torch.device("cpu"))
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', device='cpu')#, map_location=torch.device("cpu"))

    def detect_human(self, img):
        results = self.model(img)
        results.show()
        pass




