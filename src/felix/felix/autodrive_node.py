import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from felix.common.settings import settings
from felix.common.image_utils import sensor_image_to_cv2
import cv2
import time
import numpy as np
import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda
from enum import Enum

TRT_LOGGER = trt.Logger()

host_inputs  = []
cuda_inputs  = []
host_outputs = []
cuda_outputs = []
bindings = []

class DriveState(Enum):
    STOPPED = 0
    DRIVING = 1
    AVOIDING = 2

class AutoDriveNode(Node):

    def __init__(self):
        super().__init__("autodrive_node", parameter_overrides=[])

        self.get_logger().info("Started autodrive node.")
        self.drive_state = DriveState.STOPPED
        self.engine = self.prepare_engine()
        self.running = False
        self.create_subscription(Bool, "autodrive", self.set_run_state, 10)
        # publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.image_subscription = self.create_subscription(Image, settings.Topics.raw_video, self.autodrive, 5)
        
        self.get_logger().info("Initialized autodrive node.")
        
    def set_run_state(self, msg: Bool):
        self.running = msg.data
        if msg.data:
            self.start()
        else:
            self.stop()
        
        self.get_logger().info(f"Set autodrive state to: {'on' if msg.data else 'off'}")

    def stop_drivetrain(self):
        msg = Twist()
        self.cmd_vel_publisher.publish(msg)
        self.drive_state = DriveState.STOPPED

    def start(self):
        self.running = True
        self.drive_state = DriveState.STOPPED

    def stop(self):
        self.running = False

    def prepare_engine(self):
        runtime = trt.Runtime(TRT_LOGGER)
        with open(settings.Training.trt_file, 'rb') as f:
            buf = f.read()
            engine = runtime.deserialize_cuda_engine(buf)

        # create buffer
        for binding in engine:
            size = trt.volume(engine.get_binding_shape(binding)) * engine.max_batch_size
            host_mem = cuda.pagelocked_empty(shape=[size],dtype=np.float32)
            cuda_mem = cuda.mem_alloc(host_mem.nbytes)

            bindings.append(int(cuda_mem))
            if engine.binding_is_input(binding):
                host_inputs.append(host_mem)
                cuda_inputs.append(cuda_mem)
            else:
                host_outputs.append(host_mem)
                cuda_outputs.append(cuda_mem)

        return engine

    def inference(self, sensor_image):
        image = sensor_image_to_cv2(sensor_image)
        image = cv2.resize(image, (224, 224), interpolation = cv2.INTER_AREA)
        image = (2.0 / 255.0) * image.transpose((2, 0, 1)) - 1.0
        
        np.copyto(host_inputs[0], image.ravel())
        stream = cuda.Stream()
        context = self.engine.create_execution_context()

        start_time = time.time()
        cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
        context.execute_async(bindings=bindings, stream_handle=stream.handle)
        cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
        stream.synchronize()
        print("execute times "+str(time.time()-start_time))

        output = host_outputs[0].reshape(np.concatenate(([1],self.engine.get_binding_shape(1))))
        result = np.argmax(output)
        self.get_logger().info(f"result: {result}")
        return result

    def autodrive(self, sensor_image: Image):
        
        
        if not self.running:
            self.drive_state = DriveState.STOPPED
            return

        y = self.inference(sensor_image)
        self.get_logger().info(f"inference: {y}")
        
        new_state = DriveState.DRIVING if y == 1 else DriveState.AVOIDING
        
        if new_state != self.drive_state:
            self.drive_state = new_state
            if not settings.debug:
                t = Twist()
                vel_vector = list(settings.Training.velocity_map.values())[y]
                if not vel_vector:
                    vel_vector = [0,0,0]

                t.linear.x, t.linear.y, t.angular.z = [v * settings.Motion.autodrive_speed for v in vel_vector]
                self.get_logger().info(f"Autodrive: {t}")
                #self.cmd_vel_publisher.publish(t)

def main(args=None):
    rclpy.init(args=args)
    node = AutoDriveNode()
    rclpy.spin(node=node)
    node.stop()
    torch.cuda.empty_cache()
    rclpy.shutdown()
    





    