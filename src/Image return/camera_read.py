import cv2
import requests
import numpy as np


def get_frame(url):
    response = requests.get(url, stream=True)
    if response.status_code != 200:
        raise ValueError('Failed to get frame: ' + response.status_code)

    bytes_stream = bytes()
    for chunk in response.iter_content(chunk_size=1024):
        bytes_stream += chunk
        # 开始和结束
        a = bytes_stream.find(b'\xff\xd8')
        b = bytes_stream.find(b'\xff\xd9')
        # 读取框架
        if a != -1 and b != -1:
            jpg = bytes_stream[a:b + 2]
            bytes_stream = bytes_stream[b + 2:]
            frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
            return frame


url = 'http://192.168.149.1:5000/video_feed'

while True:
    frame = get_frame(url)
    if frame is not None:
        cv2.imshow('Frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
