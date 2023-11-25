# public imports
import rclpy
import signal
import threading
import flask
from flask_cors import CORS
from flask import Flask, Response, jsonify, request
from rclpy.node import Node
from typing import Optional

# ros imports
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool
import felix.common.image_utils as image_utils
from felix.common.settings import settings
from felix.common.image_collector import ImageCollector



class Api(Node):
    def __init__(self, *args):

        super().__init__(node_name='app', parameter_overrides=[])

        # properties
        self.autodrive_on = False
        self.image: Optional[Image] = None
        self.jpeg_bytes: Optional[bytes] = None
        self.collector = ImageCollector()

        # publishers
        self.motion_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # subscibers
        self.create_subscription(Image, "video_source/raw", self.image_callback, 5)

        # clients
        #self.autodrive_client = self.create_client(SetBool, "set_autodrive_state")

        # wait for client to be ready
        #while not self.autodrive_client.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('autodrive service not available, waiting again...')
        
    # callbacks

    def image_callback(self, msg: Image):
        self.image = msg
        self.jpeg_image_bytes = image_utils.sensor_image_to_jpeg_bytes(msg)

    # publishers

    def twist(self, twist: Twist):
        self.motion_publisher.publish(twist)
        return twist
    
    def get_jpeg(self):
        return self.jpeg_image_bytes

    def collect_image(self, category):
        try:
            self.get_logger().info(f"received collect image request {category}")
            img = self.get_jpeg()
            return self.collector.collect(category, img)
        except Exception as ex:
            self.get_logger().error(str(ex))
    
    def toggle_autodrive(self):
        """
            req = SetBool.Request()
            status = not self.autodrive_on
            req.data = status
            response: SetBool.Response = self.autodrive_client.call(req)
            
            if not response:
                self.get_logger().error(f"Autodrive request timed out.")
            else:
                self.autodrive_on = status
                self.get_logger().info(f"{response}")

            
            return self.autodrive_on
        """
        return self.autodrive_on
            

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
        prev_sigint_handler(signal) # type: ignore


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

# API Methods
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


@app.route('/api/autodrive')
def toggle_autodrive():
    try:
        return {"status" : app_node.toggle_autodrive()}
    except:
        return {"status" : False}

@app.route('/api/stream')
def stream():
    return Response(_get_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/api/categories/<category>/collect')
def collect(category: str):
    try:
        resp = app_node.collect_image(category)
        return {'count': resp}
    except Exception as ex:
        return {'count': -1, 'error': str(ex)}

@app.get('/api/categories/counts')
def category_counts():
    results = app_node.collector.get_categories()
    return jsonify(results)

@app.get('/api/categories/<category>/images')
def category_images(category: str):
    return {"images": app_node.collector.get_images(category)}


@app.get('/api/categories/<category>/images/<name>')
def get_image(category, name):
    bytes_str = app_node.collector.load_image(category, name)
    response = flask.make_response(bytes_str)
    response.headers.set('Content-Type', 'image/jpeg')
    return response

@app.get('/api/categories/<category>/images/<name>/<cam_index>')
def get_image2(category, name, cam_index):
    bytes_str = app_node.collector.load_image(category, name)
    response = flask.make_response(bytes_str)
    response.headers.set('Content-Type', 'image/jpeg')
    return response

@app.delete('/api/categories/<category>/images/<name>')
def delete_image(category, name):
    resp = app_node.collector.delete_image(category, name)
    return {"status": resp}

def _parse_twist(json) -> Twist:
    twist = Twist()
    twist.linear.x = float(json['linear']['x'])
    twist.linear.y = float(json['linear']['y'])
    twist.linear.z = float(json['linear']['z'])
    twist.angular.x = float(json['angular']['x'])
    twist.angular.y = float(json['angular']['y'])
    twist.angular.z = float(json['angular']['z'])
    return twist

def _twist_to_json(twist):
    return {
            "linear": {"x": twist.linear.x, "y": twist.linear.y, "z": twist.linear.z},
            "angular": {"x": twist.angular.x, "y": twist.angular.y, "z": twist.angular.z},
        }
    

@app.post('/api/twist')
def apply_twist():
    return _twist_to_json(app_node.twist(_parse_twist(request.get_json())))


def main(args=None):
    threading.Thread(target=ros2_thread, args=[app_node]).start()
    app.run(host="0.0.0.0", debug=True, use_reloader = False)

if __name__=="__main__":
    main()