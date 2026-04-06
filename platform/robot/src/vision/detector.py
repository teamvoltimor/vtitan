"""Vision and YOLO detection module."""

from abc import ABC, abstractmethod

import cv2
import numpy as np


class SignDetection:
    """Represents a detected traffic sign or parking block."""

    def __init__(self, color: str, bbox: tuple[float, float, float, float], confidence: float):
        """Initialize a detection.

        Args:
            color: 'red', 'green', or 'magenta'
            bbox: (x1, y1, x2, y2)
            confidence: 0.0 to 1.0
        """
        self.color = color
        self.bbox = bbox
        self.confidence = confidence
        self.position_estimate = None

    def to_dict(self) -> dict:
        """Convert detection to a dictionary for JSON serialization."""
        return {
            "color": self.color,
            "bbox": self.bbox,
            "confidence": self.confidence,
        }


class DetectorBase(ABC):
    """Base class for vision detectors."""

    @abstractmethod
    def detect(self, image: np.ndarray) -> list[SignDetection]:
        """Detect objects in an RGB image."""


class LocalYoloDetector(DetectorBase):
    """Local YOLO detector using ultralytics (for simulation/dev)."""

    def __init__(self, model_path: str = "yolov8n.pt"):
        from ultralytics import YOLO  # noqa: PLC0415

        self.model = YOLO(model_path)

        # Mapping from class id to string color.
        # This will depend on the trained model. Assuming 0=red, 1=green, 2=magenta.
        self.class_names = {0: "red", 1: "green", 2: "magenta"}

    def detect(self, image: np.ndarray) -> list[SignDetection]:
        """Detect objects using Ultralytics YOLO."""
        # Perform inference
        results = self.model.predict(source=image, verbose=False)

        detections = []
        if not results:
            return detections

        for box in results[0].boxes:
            class_id = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            color = self.class_names.get(class_id, "unknown")
            if color != "unknown":
                detections.append(SignDetection(color, (x1, y1, x2, y2), conf))

        return detections


class HailoDetector(DetectorBase):
    """Hailo 8 NPU detector using HailoRT Python API."""

    def __init__(self, hef_path: str):
        try:
            from hailo_platform import (  # noqa: PLC0415
                HEF,
                ConfigureParams,
                FormatType,
                HailoStreamInterface,
                InputVStreamParams,
                OutputVStreamParams,
                VDevice,
            )
        except ImportError as e:
            msg = "hailo_platform module not found. Are you running on the Raspberry Pi 5 with HailoRT installed?"
            raise ImportError(msg) from e

        self.hef = HEF(hef_path)
        self.target = VDevice()

        # Configure device
        configure_params = ConfigureParams.create_from_hef(self.hef, interface=HailoStreamInterface.PCIe)
        self.network_group = self.target.configure(self.hef, configure_params)[0]

        # Setup streams
        self.input_vstreams_params = InputVStreamParams.make_from_network_group(
            self.network_group,
            quantized=False,
            format_type=FormatType.UINT8,
        )
        self.output_vstreams_params = OutputVStreamParams.make_from_network_group(
            self.network_group,
            quantized=False,
            format_type=FormatType.FLOAT32,
        )

        self.input_stream_info = self.hef.get_input_vstream_infos()[0]
        # (height, width, channels) expected by the network
        self.input_shape = self.input_stream_info.shape

        # Mapping from class id to string color (assuming 0=red, 1=green, 2=magenta)
        self.class_names = {0: "red", 1: "green", 2: "magenta"}

    def detect(self, image: np.ndarray) -> list[SignDetection]:
        """Detect objects using Hailo 8 NPU."""
        from hailo_platform import InferVStreams  # noqa: PLC0415

        # Resize image to network expected shape
        img_resized = cv2.resize(image, (self.input_shape[1], self.input_shape[2]))
        # Hailo models expect [batch, H, W, C]
        input_data = np.expand_dims(img_resized, axis=0)

        detections = []

        with InferVStreams(
            self.network_group,
            self.input_vstreams_params,
            self.output_vstreams_params,
        ) as infer_pipeline:
            # Perform inference
            infer_results = infer_pipeline.infer({self.input_stream_info.name: input_data})

            # Hailo YOLO NMS output format depends on compilation, but typically returns
            # a dictionary with a single output stream containing shape (batch, max_boxes, 6)
            output_stream_data = next(iter(infer_results.values()))
            output_data = output_stream_data[0]  # Take first batch item

            # The embedded NMS output is usually [y_min, x_min, y_max, x_max, confidence, class_id]
            for box in output_data:
                conf = float(box[4])
                if conf < 0.25:  # Confidence threshold
                    continue

                class_id = int(box[5])
                color = self.class_names.get(class_id, "unknown")
                if color == "unknown":
                    continue

                # Convert normalized coords back to original image size
                if box[2] <= 1.05 and box[3] <= 1.05:
                    y1 = box[0] * image.shape[0]
                    x1 = box[1] * image.shape[1]
                    y2 = box[2] * image.shape[0]
                    x2 = box[3] * image.shape[1]
                else:
                    scale_y = image.shape[0] / self.input_shape[1]
                    scale_x = image.shape[1] / self.input_shape[2]
                    y1, x1, y2, x2 = box[0] * scale_y, box[1] * scale_x, box[2] * scale_y, box[3] * scale_x

                detections.append(SignDetection(color, (x1, y1, x2, y2), conf))

        return detections
