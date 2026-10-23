# import sys
# sys.path.append('/usr/lib/python3.8/dist-packages/')
import cv2
import numpy as np
import time
import math
from ultralytics import YOLO

def target_extract(img, model, conf=0.5):

    # Storage for the cropped objects
    objects=[]

    current_time=time.time()
    # result of inference
    results = model(img, conf = conf, half = True, max_det = 4, iou = 0.3)
    if not results:
        return objects, img.copy()

    result = results[0]
    # Get the bounding box points
    target_frame = result.obb.xyxyxyxy.cpu().numpy()
    target_center = result.obb.xywhr

    Annotated_frame = result.plot(labels = False, conf = False)

    for i,pts1 in enumerate(target_frame):

        x, y, w, h, r = target_center[i]

        # print("r=",r.item()/math.pi*180)

        pts1 = np.float32(pts1)

        # Define the target dimensions for the cropped object
        width, height = 200, 373  # These can be adjusted or computed based on the object size

        # Define the target coordinates for the perspective transformation
        # 右上，左上，左下，右下
        pts2 = np.float32([[width, 0], [0, 0], [0, height], [width, height]])

        # Get the perspective transformation matrix
        matrix = cv2.getPerspectiveTransform(pts1, pts2)

        # Apply perspective transformation
        # every object is the image of a single target
        object = cv2.warpPerspective(img, matrix, (width, height))

        # Store the cropped object
        object_data={'frame':object,'x': x.item(),'y': y.item(),'pattern_id': -1}
        objects.append(object_data)

    return objects, Annotated_frame

def graph_recognize(img,model,conf=0.2):
    objects=[]
    # result of inference
    results = model(img, conf = conf, half = True, max_det = 4, iou = 0.3)
    if not results:
        return objects, img.copy()

    result = results[0]
    # Get the bounding box points
    target_center = result.obb.xywhr
    target_data = result.obb.data

    Annotated_frame = result.plot(labels = True, conf = True)

    for i,data in enumerate(target_data):

        x, y, w, h, r = target_center[i]
        class_index = int(data[-1].item())
        # print("r=",r.item()/math.pi*180)

        # Store the cropped object
        object_data={'x': x.item(),'y': y.item(),'pattern_id': class_index}
        objects.append(object_data)

    return objects, Annotated_frame

# 使用示例
if __name__ == "__main__":
    
    # 加载模型
    model = YOLO('./model/hjc_graph_obb2.engine',task = 'obb')
    # model = YOLO('/home/thuair/CUADC/model/tmp/model.engine',task = 'obb')
    # model = YOLO('./model/dronePB8_wl.onnx',task = 'obb')
    # model = YOLO('./model/tmp/dronePB8.engine',task = 'obb')
    # model = YOLO('./model/812_red.engine',task = 'obb')
    
    # 读取图像
    img = cv2.imread('./picture_test/test_graph.jpg')
    print(img.shape)
    # result = model(img, conf = 0.5,half = True, max_det=4, iou=0.3)

    # 调用函数
    # objects, annotated_img = target_extract(img, model, conf=0.5)
    
    # # 显示结果
    # print(f"检测到 {len(objects)} 个对象")
    for j in range(0,5):
        print(j)
        objects, annotated_img = graph_recognize(img, model, conf=0.01)
        # objects, annotated_img = target_extract(img, model, conf=0.5)
        print(j)
        for i, obj in enumerate(objects):
            print(f"对象 {i}: ID={obj['pattern_id']}, 中心点=({obj['x']:.2f}, {obj['y']:.2f})")
        
        # 显示每个裁剪的对象
        # cv2.imshow(f'Object {i}', obj['frame'])
    
    # 显示带标注的图像
    cv2.imwrite('./picture_test/result.jpg', annotated_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
