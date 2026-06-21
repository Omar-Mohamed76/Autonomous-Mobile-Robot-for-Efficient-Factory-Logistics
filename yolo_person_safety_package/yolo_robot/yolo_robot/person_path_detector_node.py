#!/usr/bin/env python3

import time

import cv2
import numpy as np
import rclpy

from rclpy.node import Node
from std_msgs.msg import Bool, String
from ultralytics import YOLO


# ============================================================
# Configuration
# ============================================================

CAMERA_INDEX = 0

CAMERA_WIDTH = 640
CAMERA_HEIGHT = 480
CAMERA_FPS = 30

MODEL_PATH = "/home/ubuntu/yolov8n.pt"

PERSON_CLASS_ID = 0

CONFIDENCE_THRESHOLD = 0.45
IMAGE_SIZE = 320

CAMERA_OPEN_RETRIES = 3
CAMERA_RETRY_DELAY = 2.0
CAMERA_WARMUP_FRAMES = 10

PROCESS_EVERY_N_FRAMES = 3

REQUIRED_BLOCKING_FRAMES = 2
REQUIRED_CLEAR_FRAMES = 5

MIN_PERSON_HEIGHT_RATIO = 0.10
PATH_OVERLAP_THRESHOLD = 0.15

DETECTION_LOG_INTERVAL = 1.0
NO_PERSON_LOG_INTERVAL = 2.0
CAMERA_ERROR_LOG_INTERVAL = 2.0

PERSON_TOPIC = "/person_in_path"
VISION_TOPIC = "/vision_command"


class PersonPathDetectorNode(Node):

    def __init__(self):
        super().__init__("person_path_detector_node")

        self.person_publisher = self.create_publisher(
            Bool,
            PERSON_TOPIC,
            10
        )

        self.command_publisher = self.create_publisher(
            String,
            VISION_TOPIC,
            10
        )

        self.get_logger().info(
            f"Opening camera index: {CAMERA_INDEX}"
        )

        self.cap = self.open_camera()

        self.get_logger().info(
            f"Camera index {CAMERA_INDEX} opened successfully"
        )

        self.get_logger().info(
            "Loading YOLO model..."
        )

        self.model = YOLO(MODEL_PATH)

        self.get_logger().info(
            "YOLO model loaded successfully"
        )

        self.frame_counter = 0
        self.blocking_counter = 0
        self.clear_counter = 0
        self.person_blocking = False

        self.last_detection_log_time = 0.0
        self.last_no_person_log_time = 0.0
        self.last_camera_error_log_time = 0.0

        self.camera_timer = self.create_timer(
            0.03,
            self.camera_callback
        )

        self.status_timer = self.create_timer(
            0.5,
            self.publish_current_person_state
        )

        self.get_logger().info(
            "Person path detector started"
        )

        self.get_logger().info(
            f"Continuous safety state: {PERSON_TOPIC}"
        )

        self.get_logger().info(
            f"State-change commands: {VISION_TOPIC}"
        )

    def open_camera(self):
        for attempt in range(
            1,
            CAMERA_OPEN_RETRIES + 1
        ):
            self.get_logger().info(
                f"Camera open attempt "
                f"{attempt}/{CAMERA_OPEN_RETRIES}"
            )

            cap = cv2.VideoCapture(
                CAMERA_INDEX,
                cv2.CAP_V4L2
            )

            if not cap.isOpened():
                cap.release()

                self.get_logger().warn(
                    f"Could not open camera on attempt {attempt}"
                )

                if attempt < CAMERA_OPEN_RETRIES:
                    time.sleep(CAMERA_RETRY_DELAY)

                continue

            cap.set(
                cv2.CAP_PROP_FOURCC,
                cv2.VideoWriter_fourcc(*"MJPG")
            )

            cap.set(
                cv2.CAP_PROP_FRAME_WIDTH,
                CAMERA_WIDTH
            )

            cap.set(
                cv2.CAP_PROP_FRAME_HEIGHT,
                CAMERA_HEIGHT
            )

            cap.set(
                cv2.CAP_PROP_FPS,
                CAMERA_FPS
            )

            cap.set(
                cv2.CAP_PROP_BUFFERSIZE,
                1
            )

            frame_received = False

            for _ in range(CAMERA_WARMUP_FRAMES):
                ret, frame = cap.read()

                if ret and frame is not None:
                    frame_received = True
                    break

                time.sleep(0.1)

            if frame_received:
                actual_width = int(
                    cap.get(cv2.CAP_PROP_FRAME_WIDTH)
                )

                actual_height = int(
                    cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
                )

                actual_fps = cap.get(
                    cv2.CAP_PROP_FPS
                )

                self.get_logger().info(
                    "Camera configuration: "
                    f"{actual_width}x{actual_height}, "
                    f"FPS={actual_fps:.1f}"
                )

                return cap

            cap.release()

            self.get_logger().warn(
                "Camera opened but no frame was received"
            )

            if attempt < CAMERA_OPEN_RETRIES:
                time.sleep(CAMERA_RETRY_DELAY)

        raise RuntimeError(
            f"Webcam could not be opened at index "
            f"{CAMERA_INDEX}. Check that no other process "
            f"is using /dev/video{CAMERA_INDEX}."
        )

    def publish_current_person_state(self):
        message = Bool()
        message.data = self.person_blocking
        self.person_publisher.publish(message)

    def get_path_polygon(
        self,
        frame_width,
        frame_height
    ):
        polygon = np.array(
            [
                [
                    int(frame_width * 0.15),
                    int(frame_height * 0.08)
                ],
                [
                    int(frame_width * 0.85),
                    int(frame_height * 0.08)
                ],
                [
                    int(frame_width * 0.99),
                    int(frame_height * 0.99)
                ],
                [
                    int(frame_width * 0.01),
                    int(frame_height * 0.99)
                ],
            ],
            dtype=np.int32
        )

        return polygon

    def person_is_in_path(
        self,
        bounding_box,
        frame_width,
        frame_height
    ):
        x1, y1, x2, y2 = bounding_box

        x1 = max(
            0,
            min(frame_width - 1, int(x1))
        )

        y1 = max(
            0,
            min(frame_height - 1, int(y1))
        )

        x2 = max(
            0,
            min(frame_width, int(x2))
        )

        y2 = max(
            0,
            min(frame_height, int(y2))
        )

        person_width = x2 - x1
        person_height = y2 - y1

        if person_width <= 0 or person_height <= 0:
            return False, 0.0

        minimum_height = (
            frame_height
            * MIN_PERSON_HEIGHT_RATIO
        )

        if person_height < minimum_height:
            return False, 0.0

        path_polygon = self.get_path_polygon(
            frame_width,
            frame_height
        )

        path_mask = np.zeros(
            (
                frame_height,
                frame_width
            ),
            dtype=np.uint8
        )

        cv2.fillPoly(
            path_mask,
            [path_polygon],
            255
        )

        person_region = path_mask[
            y1:y2,
            x1:x2
        ]

        overlap_pixels = int(
            np.count_nonzero(person_region)
        )

        person_box_area = (
            person_width
            * person_height
        )

        overlap_ratio = (
            overlap_pixels
            / float(person_box_area)
        )

        center_x = int(
            (x1 + x2) / 2
        )

        center_y = int(
            (y1 + y2) / 2
        )

        bottom_center_x = center_x
        bottom_center_y = y2 - 1

        center_inside = (
            cv2.pointPolygonTest(
                path_polygon,
                (
                    float(center_x),
                    float(center_y)
                ),
                False
            )
            >= 0
        )

        bottom_center_inside = (
            cv2.pointPolygonTest(
                path_polygon,
                (
                    float(bottom_center_x),
                    float(bottom_center_y)
                ),
                False
            )
            >= 0
        )

        in_path = (
            overlap_ratio >= PATH_OVERLAP_THRESHOLD
            or center_inside
            or bottom_center_inside
        )

        return in_path, overlap_ratio

    def detect_blocking_person(self, frame):
        frame_height, frame_width = frame.shape[:2]

        results = self.model.predict(
            source=frame,
            classes=[PERSON_CLASS_ID],
            conf=CONFIDENCE_THRESHOLD,
            imgsz=IMAGE_SIZE,
            device="cpu",
            max_det=5,
            verbose=False
        )

        current_time = time.time()

        if not results:
            return False

        boxes = results[0].boxes

        if boxes is None or len(boxes) == 0:
            if (
                current_time
                - self.last_no_person_log_time
                >= NO_PERSON_LOG_INTERVAL
            ):
                self.get_logger().info(
                    "No person detected"
                )

                self.last_no_person_log_time = (
                    current_time
                )

            return False

        blocking_person_found = False

        for detection in boxes:
            confidence = float(
                detection.conf[0].item()
            )

            x1, y1, x2, y2 = (
                detection.xyxy[0].tolist()
            )

            in_path, overlap_ratio = (
                self.person_is_in_path(
                    (
                        x1,
                        y1,
                        x2,
                        y2
                    ),
                    frame_width,
                    frame_height
                )
            )

            if (
                current_time
                - self.last_detection_log_time
                >= DETECTION_LOG_INTERVAL
            ):
                self.get_logger().info(
                    "Person detected: "
                    f"confidence={confidence:.2f}, "
                    f"path_overlap={overlap_ratio:.2f}, "
                    f"in_path={in_path}"
                )

                self.last_detection_log_time = (
                    current_time
                )

            if in_path:
                blocking_person_found = True
                break

        return blocking_person_found

    def publish_person_state(self, blocked):
        bool_message = Bool()
        bool_message.data = blocked

        self.person_publisher.publish(
            bool_message
        )

        command_message = String()

        if blocked:
            command_message.data = (
                "PERSON_BLOCKING"
            )
        else:
            command_message.data = (
                "PATH_CLEAR"
            )

        self.command_publisher.publish(
            command_message
        )

        self.get_logger().warn(
            f"Published: {command_message.data}"
        )

    def update_state(
        self,
        blocking_detected
    ):
        if blocking_detected:
            self.blocking_counter += 1
            self.clear_counter = 0

            if (
                self.blocking_counter
                >= REQUIRED_BLOCKING_FRAMES
                and not self.person_blocking
            ):
                self.person_blocking = True

                self.publish_person_state(
                    True
                )

        else:
            self.clear_counter += 1
            self.blocking_counter = 0

            if (
                self.clear_counter
                >= REQUIRED_CLEAR_FRAMES
                and self.person_blocking
            ):
                self.person_blocking = False

                self.publish_person_state(
                    False
                )

    def camera_callback(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            current_time = time.time()

            if (
                current_time
                - self.last_camera_error_log_time
                >= CAMERA_ERROR_LOG_INTERVAL
            ):
                self.get_logger().error(
                    "Failed to read camera frame"
                )

                self.last_camera_error_log_time = (
                    current_time
                )

            return

        self.frame_counter += 1

        if (
            self.frame_counter
            % PROCESS_EVERY_N_FRAMES
            != 0
        ):
            return

        blocking_detected = (
            self.detect_blocking_person(frame)
        )

        self.update_state(
            blocking_detected
        )

    def destroy_node(self):
        if (
            hasattr(self, "cap")
            and self.cap is not None
        ):
            self.cap.release()

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = PersonPathDetectorNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        print(
            f"Person detector failed: {error}"
        )

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
