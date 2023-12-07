from felix.common.settings import settings
import cv2
import time
import numpy as np
import tensorrt as trt
import pycuda.autoinit
import pycuda.driver as cuda

EXPLICIT_BATCH = 1 << (int)(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH)
TRT_LOGGER = trt.Logger(trt.Logger.INFO)
runtime = trt.Runtime(TRT_LOGGER)

host_inputs  = []
cuda_inputs  = []
host_outputs = []
cuda_outputs = []
bindings = []


def Inference(engine):
    #image = cv2.imread("/data/obstacle/blocked/1701317636.5884311.jpg")
    image = cv2.imread("/data/obstacle/free/1701317658.831459.jpg")
    image = cv2.resize(image, (224, 224), interpolation = cv2.INTER_AREA)
    image = (2.0 / 255.0) * image.transpose((2, 0, 1)) - 1.0

    np.copyto(host_inputs[0], image.ravel())
    stream = cuda.Stream()
    context = engine.create_execution_context()

    start_time = time.time()
    cuda.memcpy_htod_async(cuda_inputs[0], host_inputs[0], stream)
    context.execute_async(bindings=bindings, stream_handle=stream.handle)
    cuda.memcpy_dtoh_async(host_outputs[0], cuda_outputs[0], stream)
    stream.synchronize()
    print("execute times "+str(time.time()-start_time))

    output = host_outputs[0].reshape(np.concatenate(([1],engine.get_binding_shape(1))))
    print("output:")
    print(np.argmax(output))


def PrepareEngine():
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

def main():
    engine = PrepareEngine()
    Inference(engine)

if __name__ == "__main__":
    main()
    