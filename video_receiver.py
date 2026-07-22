import socket
import cv2
import numpy as np
import time

class VideoReceiver:
    """
    Live UDP Video Receiver for Raspberry Pi 5.
    Receives compressed JPEG frames over UDP (Port 5006) from PiSim (Unreal Engine 5)
    and displays live video stream using OpenCV.
    """
    def __init__(self, listen_ip: str = "0.0.0.0", listen_port: int = 5006):
        self.listen_ip = listen_ip
        self.listen_port = listen_port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # Increase socket receive buffer size to handle high frame rates without dropping packets
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 2 * 1024 * 1024)
        self.sock.bind((self.listen_ip, self.listen_port))
        print(f"[VideoReceiver] Listening for live video stream on {self.listen_ip}:{self.listen_port}...")

    def start_display_loop(self, frame_callback=None):
        """
        Main loop receiving JPEG frames, decoding with OpenCV, displaying live window,
        and optionally passing the frame to a user-defined image processing function.
        """
        window_name = "PiSim Pi5 Live Camera Stream (Port 5006)"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(window_name, 640, 480)

        frame_count = 0
        start_time = time.time()

        try:
            while True:
                # Max UDP datagram size
                data, addr = self.sock.recvfrom(65507)
                if not data:
                    continue

                # Decode JPEG byte buffer to OpenCV image
                np_arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                if frame is not None:
                    frame_count += 1

                    # Execute custom image processing callback if provided
                    if frame_callback:
                        frame = frame_callback(frame)

                    # Display frame in OpenCV window
                    cv2.imshow(window_name, frame)

                    # Print FPS every 30 frames
                    if frame_count % 30 == 0:
                        elapsed = time.time() - start_time
                        fps = frame_count / elapsed if elapsed > 0 else 0
                        print(f"[VideoReceiver] Streaming active... FPS: {fps:.1f} ({frame.shape[1]}x{frame.shape[0]})")

                # Press 'q' or ESC in video window to exit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q') or key == 27:
                    print("[VideoReceiver] Exit requested by user.")
                    break

        except KeyboardInterrupt:
            print("[VideoReceiver] Stopped by user (KeyboardInterrupt).")
        finally:
            self.sock.close()
            cv2.destroyAllWindows()
            print("[VideoReceiver] Closed socket and video window.")

if __name__ == "__main__":
    def my_image_processing_example(frame):
        """
        Example user image processing function.
        Put your OpenCV / YOLO / AI object detection code here!
        """
        # Example: Add timestamp or overlay text
        cv2.putText(frame, "Pi5 OpenCV Processing Hook", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        return frame

    receiver = VideoReceiver(listen_ip="0.0.0.0", listen_port=5006)
    receiver.start_display_loop(frame_callback=my_image_processing_example)
