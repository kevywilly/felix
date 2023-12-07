
from felix.common.trt_utils import TRTUtils
from felix.common.settings import settings

utils = TRTUtils(settings.Training)

if not utils.onnx_model_exists():
    utils.torch2onnx()

engine = utils.onnx2trt()
