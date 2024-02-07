# public imports
import rclpy
import signal
import threading
import flask
from flask_cors import CORS
from flask import Flask, Response, jsonify, request
from rclpy.node import Node
from typing import Dict, Optional
from cv_bridge import CvBridge
# ros imports
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from felix.vision.camera import ArgusCamera
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry
import felix.vision.image_utils as image_utils
from felix.config.settings import settings, TrainingType
from felix.vision.image_collector import ImageCollector
from felix.motion.kinematics import Kinematics
from felix.motion.joystick import JoystickUpdateEvent
from felix.motion.utils import twist_to_json
from std_msgs.msg import Header


JOYSTICK_LINEAR_X_SCALE = 1
JOYSTICK_LINEAR_Y_SCALE = 1
JOYSTICK_ANGULAR_Z_SCALE = 1


class Api(Node):
    def __init__(self, *args):

        super().__init__(node_name='app', parameter_overrides=[])
        # properties
        self.autodrive = False
        self.image: Optional[Image] = None
        self.jpeg_bytes: Optional[bytes] = None
        self.collector = ImageCollector()
        self._sensor_mode = settings.DEFAULT_SENSOR_MODE
        self._sensor_id = 0
        self._camera = ArgusCamera(sensor_id=self._sensor_id, sensor_mode=self._sensor_mode)
        self.bridge = CvBridge()

        # publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, settings.Topics.cmd_vel, 1)
        self.autodrive_publisher = self.create_publisher(Bool, settings.Topics.autodrive, 1)
        self.nav_publisher = self.create_publisher(Odometry, settings.Topics.cmd_nav,10)

        # subscribers
        #self.create_subscription(Image, settings.Topics.raw_video, self.image_callback, 5)
        self.create_timer(1.0/self._sensor_mode.framerate,self.video_timer)
        self._image_publisher = self.create_publisher(Image,settings.Topics.raw_video,5)

    def video_timer(self, *args):
        if self._camera.read():
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "camera"
            msg = self.bridge.cv2_to_imgmsg(self._camera.value, encoding="rgb8", header=header)
            self._image_publisher.publish(msg)
            self.jpeg_image_bytes =self._camera.jpeg
            
        else:
            self.get_logger().warn(f"Can't receive frame for cap{self._sensor_id}")
            

    # callbacks
    def image_callback(self, msg: Image):
        
        self.image = msg
        self.jpeg_image_bytes = image_utils.sensor_image_to_jpeg_bytes(msg)

    # publishers

    def joystick(self, data: Optional[Dict]) -> Twist:
        if not data:
            return Twist()
        
        event: JoystickUpdateEvent = JoystickUpdateEvent(**data)
        t: Twist = event.get_twist()
        self.cmd_vel_publisher.publish(t)
        return t
        
    def twist(self, data: Optional[Dict]) -> Twist:
        if not data:
            return Twist()
        t = Twist()
        t.linear.x = float(data["linear"]["x"])
        t.linear.y = float(data["linear"]["y"])
        t.angular.z = float(data["angular"]["z"])
        self.cmd_vel_publisher.publish(t)
        return t

    def navigate(self, data: Optional[Dict]):

        if not data:
            return Odometry()
        
        x = int(data["cmd"]["x"])
        y = int(data["cmd"]["y"])
        w = int(data["cmd"]["w"])
        h = int(data["cmd"]["h"])

        driveMode = data["driveMode"]
        captureMode = data["captureMode"]

        odom = Kinematics.xywh_to_nav_target(x,y,w,h)
        captured_image = None
        
        if driveMode:
            self.nav_publisher.publish(odom)

        if captureMode:
            captured_image = self.save_navigation_image(x,y,w,h)
            

        return captured_image
    
    def get_jpeg(self):
        return self.jpeg_image_bytes
    
    def toggle_autodrive(self, value: Optional[bool] = None):
        self.autodrive = not self.autodrive if value is None else value
        self.cmd_vel_publisher.publish(Twist())
        msg = Bool()
        msg.data = self.autodrive
        self.autodrive_publisher.publish(msg)
        
        return self.autodrive


    def save_tag(self, tag):
        img = self.get_jpeg()
        return self.collector.save_tag(img, tag)


    def get_tags(self):
        return self.collector.get_tags()
    

    def collect_image(self, category):
        try:
            self.get_logger().info(f"received collect image request {category}")
            img = self.get_jpeg()
            return self.collector.collect(category, img)
        except Exception as ex:
            self.get_logger().error(str(ex))
    


    def save_navigation_image(self, x: int, y: int, width:int, height: int):
        try:
            self.get_logger().info(f"received collect image request xy:{x},{y},{width},{height}")
            img = self.get_jpeg()
            return self.collector.save_navigation_image(x, y, width, height, img)
        except Exception as ex:
            self.get_logger().error(str(ex))

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
def index():
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


@app.post('/api/twist')
def apply_twist():
    data = request.get_json()
    app_node.twist(request.get_json())
    return data

@app.post('/api/navigate')
def navigate():
    data = request.get_json()
    captured = app_node.navigate(data)
    return {'captured': captured}


@app.post('/api/tags')
def add_tag():
    data = request.get_json()
    if data['tag']:
        return app_node.save_tag(data['tag'])
    else:
        return app_node.get_tags()


@app.get('/api/tags')
def get_tags():
    return app_node.get_tags()


@app.post('/api/joystick')
def api_joystick():
    resp = app_node.joystick(request.get_json())
    return twist_to_json(resp)

@app.route('/api/categories/<category>/collect')
def collect(category: str):
    try:
        resp = app_node.collect_image(category)
        return {'count': resp}
    except Exception as ex:
        return {'count': -1, 'error': str(ex)}

@app.get('/api/training/type')
def get_training_type():
    return {"type": settings.Training.type}

@app.get('/api/categories/counts')
def category_counts():
    results = app_node.collector.get_categories() if settings.Training.type == TrainingType.OBSTACLE else {}
    return jsonify(results)

@app.get('/api/categories/<category>/images')
def category_images(category: str):
    if settings.Training.type == TrainingType.OBSTACLE:
        return {"images": app_node.collector.get_images(category)}
    else:
        return {"images": []}


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


def main(args=None):
    threading.Thread(target=ros2_thread, args=[app_node]).start()
    app.run(host="0.0.0.0", debug=True, use_reloader = False)

if __name__=="__main__":
    main()