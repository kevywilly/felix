import rclpy
import signal
import threading
import flask
from flask_cors import CORS
from flask import Flask, Response, jsonify, request
import cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
#from geometry_msgs.msg import Twist
#from oribot_interfaces.srv import Toggle
from felix.scripts.image_collector import ImageCollector
#import oribot.scripts.cmd_utils as cmd_utils
import felix.scripts.image_utils as image_utils

class Api(Node):
    def __init__(self):
        super().__init__('api_node')
        self.image: Image = None
        self.jpeg_bytes: bytes = None
        self.collector = ImageCollector()
        self.create_subscription(Image, "/video_source/raw", self.image_callback, 10)
       
    def log(self, txt: str):
        self.get_logger().info(f"{txt}")

    def image_callback(self, msg: Image):
        self.image = msg
        # self.get_logger().info(msg)
        self.jpeg_image_bytes = image_utils.sensor_image_to_jpeg_bytes(msg)

    def get_image(self):
        return self.image
    
    def get_jpeg(self):
        return self.jpeg_image_bytes
            

def ros2_thread(node: Api):
    print('entering ros2 thread')
    rclpy.spin(node)
    node.destroy_node()
    print('leaving ros2 thread')


def sigint_handler(signal, frame):
    """
    SIGINT handler

    We have to know when to tell rclpy to shut down, because
    it's in a child thread which would stall the main thread
    shutdown sequence. So we use this handler to call
    rclpy.shutdown() and then call the previously-installed
    SIGINT handler for Flask
    """
    
    rclpy.shutdown()
    if prev_sigint_handler is not None:
        prev_sigint_handler(signal)


rclpy.init(args=None)
app_node = Api()

app = Flask(__name__)
CORS(app)
cors = CORS(app, resource={
    r"/*": {
        "origins": "*"
    }
})

prev_sigint_handler = signal.signal(signal.SIGINT, sigint_handler)


def _get_stream():
    while True:
        # ret, buffer = cv2.imencode('.jpg', frame)
        try:
            yield (
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n' + app_node.get_jpeg() + b'\r\n'
            )  # concat frame one by one and show result
        except Exception as ex:
            pass
    
@app.route('/')
def hello():
    return "hello"

@app.route('/api/stream')
def stream():
    return Response(_get_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

def main(args=None):
    threading.Thread(target=ros2_thread, args=[app_node]).start()
    app.run(host="0.0.0.0", debug=True, use_reloader = False)

if __name__=="__main__":
    main()