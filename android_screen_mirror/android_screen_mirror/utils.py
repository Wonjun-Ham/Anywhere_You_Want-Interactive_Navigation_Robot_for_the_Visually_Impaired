import time
import numpy as np
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import cv2

# Rate 클래스 프로젝트에서 쓰이고 있는 거 모두 etri 추가
class Rate:
    def __init__(self, hz):
        self.period = 1.0 / hz
        self.last_time = time.time()

    def sleep(self):
        now = time.time()
        sleep_time = self.period - (now - self.last_time)
        if sleep_time > 0:
            time.sleep(sleep_time)
        self.last_time = time.time()

# process_data_utils custom image processing functions 부분과 동기화
def msg_to_pil(msg: Image) -> PILImage.Image:
    # bgr, rgb 얘기 있으면
    if "bgr" in msg.encoding or "rgb" in msg.encoding:
        
        if "8" in msg.encoding:  # For 8-bit images
            dtype = np.uint8
        else: # For 16-bit images
            dtype = np.uint16  
        # convert to numpy array
        img = np.frombuffer(msg.data, dtype=dtype).reshape(
            msg.height, msg.width, -1)
            
        # For 16-bit images, scale down to 8-bit for PIL
        if dtype == np.uint16:
            img = (img / 256).astype(np.uint8)

        # (rgba, bgra) to (rgb, bgr)  
        if img.shape[-1] == 4: 
            img = img[:, :, :3]  # Keep only rgb or bgr channels

        # bgr to rgb
        if "bgr" in msg.encoding:
            img=cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # convert to PIL Image
        img = PILImage.fromarray(img)

    else:
        raise ValueError(f"encoding format : {msg.encoding}. modify this function to handle this case")    

    return img