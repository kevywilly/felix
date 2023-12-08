
import rclpy
from rclpy.node import Node
from felix_trainer.scripts.trt_utils import TRTUtils
from felix.common.settings import settings

class ConverterNode(Node):

    def __init__(self):
        super().__init__("convert", parameter_overrides=[])
        
    def convert(self):
        utils = TRTUtils(settings.Training)

        if not utils.onnx_model_exists():
            utils.torch2onnx()

        engine = utils.onnx2trt()
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ConverterNode()
    node.convert()
    rclpy.shutdown()

