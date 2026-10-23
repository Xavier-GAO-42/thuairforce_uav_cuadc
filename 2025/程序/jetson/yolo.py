# 已经基本没问题了。但是模型还有问题——我认为需要更改模型方案。。。见README
# yolo进程负责解算出靶标的gps位置与信息，将其返回到target_queue
import time
import logging
import cv2
import numpy as np
import locate
from ultralytics import YOLO

logger = logging.getLogger(__name__)

def initializer(config):
    global mission, mission_color
    global modelpath_extract_tianjing_blue,modelpath_extract_tianjing_red # 用于数字靶标
    global modelpath_number
    global modelpath_graph # 用于图像靶标的识别。直接由整张图片给出各个目标的位置和pattern_id
    global model_extract,model_detect # 用于数字靶标的识别。先切割出靶标区域再识别数字
    global model_graph

    mission = config.get_int("mission")
    if mission == 1:
        mission_color = config.get_string("mission_color")
        if mission_color == 'blue':
            modelpath_extract_tianjing_blue = config.get_string("yolo.modelpath_extract_tianjing_blue")
            model_extract=YOLO(modelpath_extract_tianjing_blue, task = 'obb')
        elif mission_color == 'red':
            modelpath_extract_tianjing_red = config.get_string("yolo.modelpath_extract_tianjing_red")
            model_extract=YOLO(modelpath_extract_tianjing_red, task = 'obb')
        else:
            logger.error(f"error color align. red or blue; but got {mission_color}")
            return False
        modelpath_number = config.get_string("yolo.modelpath_number")
        model_detect=YOLO(modelpath_number, task = 'detect')
    elif mission == 0:
        modelpath_graph = config.get_string("yolo.modelpath_graph")
        model_graph=YOLO(modelpath_graph, task = 'obb')
        # model_graph.model.to('cuda') # 若模型为.pt可以这么干
    else:
        logger.error(f"error mission align. 0 = graph; 1 = number; but got {mission}")
        return False
    logger.info("yolo config done")
    return True

def target_extract(img, model, conf=0.5):
    '''
    拉伸变换，返回一个list包含所有识别到的目标；annotated frame是绘制检测框的标注图像
    object每个元素包含：
        校正后的目标图像
        目标在原始图像中的中心x坐标
        目标在原始图像中的中心y坐标
    this method is used to extract the target from the image captured by the camera
    the conf of the target should be top three in the image
    arg image: the image to be processed captured by the camera
    arg model: the model used to detect the target
    return: a list of cropped objects and the annotated frame

    另一种更简单的方法：在训练模型时，我们要求箭头的方向与旋转框的0度方向一致
    （即靶标箭头头部指向0度方向，也就是旋转框的第一个顶点）。
    这样，在透视变换后，箭头在图像中就是向上的。
    因此，我们不需要修改代码，只需要保证训练数据中箭头框的标注方向一致即可。

    我们希望这个代码能返回靶标朝向上方的图像
    '''
    objects=[]
    results = model(img, conf = conf, half = True, max_det = 4, iou = 0.3)
    if not results:
        return objects, img.copy()
    result = results[0]
    # Get the bounding box points
    target_frame = result.obb.xyxyxyxy.cpu().numpy()
    target_center = result.obb.xywhr
    # Count the number of target
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

        # Annotated_frame = cv2.circle(Annotated_frame, (int(pts1[0][0]), int(pts1[0][1])), 5, (0, 255, 0), -1)
        # Annotated_frame = cv2.circle(Annotated_frame, (int(pts1[1][0]), int(pts1[1][1])), 5, (0, 200, 0), -1)
        # Annotated_frame = cv2.circle(Annotated_frame, (int(pts1[2][0]), int(pts1[2][1])), 5, (0, 150, 0), -1)
        # Annotated_frame = cv2.circle(Annotated_frame, (int(pts1[3][0]), int(pts1[3][1])), 5, (0, 100, 0), -1)

    return objects, Annotated_frame


def number_recognize(img, model, conf=0.5):
    '''
    不处理倒置的数字
    this function is used to recognize the number in a image after the target is extracted and turned
    arg model: the model used to recognize the number
    arg image: the image to be processed
    return: the number in the image and the annotated frame
    '''
    target_num = -1
    result = model(img, conf = conf, half = True, max_det = 2, iou = 0.3)[0]
    boxes = result.boxes
    # Annotate each target
    annotated_frame = result.plot()
    annotated_frame = cv2.resize(annotated_frame, (0, 0), fx=0.6, fy=0.6)
    if boxes and len(boxes.data) == 2:
        x1 = boxes.data[0][0].item()  # x coordinate of the first detected number
        x2 = boxes.data[1][0].item()  # x coordinate of the second detected number
        if x1 < x2:
            left_number = int(boxes.data[0][5].item())  # Number is stored in the 6th position
            right_number = int(boxes.data[1][5].item())
        else:
            left_number = int(boxes.data[1][5].item())
            right_number = int(boxes.data[0][5].item())
        target_num = left_number * 10 + right_number
        # Draw the number on frame
        textSize = cv2.getTextSize(f'{target_num}', cv2.FONT_HERSHEY_COMPLEX, 1, 1)
        cv2.putText(annotated_frame, f'{target_num}', (0, textSize[0][1]), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)
    return target_num, annotated_frame

def number_recognize2(img, model, conf=0.5):
    """
    处理倒置的数字
    """
    result1 = model(img, conf = conf, half = True, max_det = 2)[0]
    result2 = model(cv2.rotate(img, cv2.ROTATE_180), conf = conf, half = True, max_det = 2)[0]
    target_num = -1
    boxes1 = result1.boxes
    boxes2 = result2.boxes
    result = None
    if boxes1 and len(boxes1.data) == 2 and boxes2 and len(boxes2.data) == 2:
        y1 = (boxes1.data[0][1].item()+boxes1.data[1][1].item())/2
        y2 = (boxes2.data[0][1].item()+boxes2.data[1][1].item())/2
        # 第一个代表数字，第二个代表xy
        if y1 < y2: # box1靠上 box2靠下，说明box2是对的
            result = result2
        else:
            result = result1
    elif boxes1 and len(boxes1.data) == 2:
        result = result1
    elif boxes2 and len(boxes2.data) == 2:
        result = result2
    else:
        return -1, None
    
    boxes = result.boxes
    x1 = boxes.data[0][0].item()  # x coordinate of the first detected number
    x2 = boxes.data[1][0].item()  # x coordinate of the second detected number

    if x1 < x2:
        left_number = int(boxes.data[0][5].item())  # Number is stored in the 6th position
        right_number = int(boxes.data[1][5].item())
    else:
        left_number = int(boxes.data[1][5].item())
        right_number = int(boxes.data[0][5].item())

    target_num = left_number * 10 + right_number

    # Annotate each target
    annotated_frame = result.plot()
    annotated_frame = cv2.resize(annotated_frame, (0, 0), fx=0.6, fy=0.6)
    # Draw the number on frame
    textSize = cv2.getTextSize(f'{target_num}', cv2.FONT_HERSHEY_COMPLEX, 1, 1)
    cv2.putText(annotated_frame, f'{target_num}', (0, textSize[0][1]), cv2.FONT_HERSHEY_COMPLEX, 1, (255, 255, 255), 1, cv2.LINE_AA)

    return target_num, annotated_frame

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
    target_frame = result.obb.xyxyxyxy.cpu().numpy()

    Annotated_frame = result.plot(labels = False, conf = False)

    for i,data in enumerate(target_data):
        x, y, w, h, r = target_center[i]
        class_index = int(data[-1].item())
        pts1 = np.float32(target_frame[i])
        width, height = 200, 200 
        # 右上，左上，左下，右下
        pts2 = np.float32([[width, 0], [0, 0], [0, height], [width, height]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        object = cv2.warpPerspective(img, matrix, (width, height))
        object_data={'frame':object,'x': x.item(),'y': y.item(),'pattern_id': int(data[-1].item())}
        objects.append(object_data)
    return objects, Annotated_frame

def run_yolo_detection(timestamp, pose_data, frame, 
                       target_queue, picture_queue , shutdown_event):
    """
    执行目标检测并计算GPS坐标
    参数:
        image_ts: 图像时间戳
        pose_data: 包含位置和姿态的字典
        img_path: 图像文件路径
        target_queue: 结果输出队列
        picture_queue: 标注过的opencv图像输出序列
        shutdown_event: 结束事件
    """
    logger.info("run yolo detection")
    global mission, mission_color, modelpath_extract_tianjing_blue,modelpath_extract_tianjing_red
    global modelpath_number,modelpath_graph
    global model_detect,model_extract
    global model_graph

    # 1. 使用OpenCV读取图像
    # frame = cv2.imread(img_path)

    # 2. YOLO目标检测
    annotated_frame = None
    display_objects = []
    if mission == 1:
        # 识别数字
        # annotated_frame1 is the frame with boxes 获取感兴趣区域
        objects, annotated_frame = target_extract(frame, model_extract)

        for count, obj in enumerate(objects):
            target_num, target_frame = number_recognize2(obj['frame'],model_detect)
            # target_num, target_frame = number_recognize(obj['frame'],model_detect)
            objects[count]['pattern_id'] = target_num
            objects[count]['frame'] = target_frame
            if target_num != -1:
                # 3. 坐标转换计算目标GPS
                lat,lon = locate.locate(pose_data, obj['x'], obj['y'])
                # 4. 结果格式示例:
                result = {
                    'timestamp': timestamp,
                    'lat': lat,          # 目标纬度
                    'lon': lon,         # 目标经度
                    'alt': pose_data['alt']-pose_data['hei'],  # 假设与家同高度
                    'pattern_id': target_num,          # 靶标数值
            #        'confidence': 0.92,       # 检测置信度
            #        'source_pose': pose_data  # 原始位姿数据（可选）
                }
                if shutdown_event.is_set():
                    return
                target_queue.put(result)
                display_objects.append({'pattern_id':target_num, 'frame':target_frame})
    else: # 识别靶标
        objects, annotated_frame = graph_recognize(frame, model_graph)
        for count, obj in enumerate(objects):
            lat,lon = locate.locate(pose_data, obj['x'], obj['y'])
            result = {
                'timestamp': timestamp,
                'lat': lat,          # 目标纬度
                'lon': lon,         # 目标经度
                'alt': pose_data['alt']-pose_data['hei'],  # 假设与家同高度
                'pattern_id': obj['pattern_id'],          # 靶标图案ID
            }
            if shutdown_event.is_set():
                return
            target_queue.put(result)
            display_objects.append({'pattern_id':obj['pattern_id'], 'frame':obj['frame']})

    # 按照id顺序
    display_objects = sorted(display_objects, key=lambda x: x['pattern_id'], reverse=True)
    # 在所有识别完成后统一标注左上角信息
    display_height = 50
    for i, target in enumerate(display_objects):
        # 缩放靶标图像 (宽度100像素，保持比例)
        img = target['frame']
        scale_percent = 150 / img.shape[1]
        width = int(img.shape[1] * scale_percent)
        height = int(img.shape[0] * scale_percent)
        dim = (width, height)
        resized = cv2.resize(img, dim)
        
        # 计算显示位置 (左上角区域)
        start_y = 10 + display_height
        
        # 确保不超出图像边界
        if start_y + height < annotated_frame.shape[0] and width < annotated_frame.shape[1]:
            # 放置缩略图
            annotated_frame[start_y:start_y+height, 10:10+width] = resized
            # 显示pattern_id
            cv2.putText(annotated_frame, f'ID: {target["pattern_id"]}', 
                        (10, start_y + height + 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, 
                        (0, 255, 0), 2)
            # 更新显示位置
            display_height += height + 40  # 图像高度 + 文字区域
    
    picture_queue.put(annotated_frame)

#### 以下为测试用代码 ####
"""
import multiprocessing
import configuration
def test_graph():
    # 加载模型
    config = configuration.Config("./config/config_test_yolo_graph.json")
    initializer(config)
    locate.initializer(config)
    timestamp = 114514
    pose_data = {
        'timestamp': timestamp,  # Unix时间戳（ms）
        'lat': 0,        # 纬度（度）
        'lon': 0,        # 经度（度）
        'alt': 30,        # 海拔高度（米）
        'heading': 0,    # 航向角（度）
        'roll': 0,       # 滚转角（度）
        'pitch': 0,      # 俯仰角（度）
        'yaw': 0,        # 偏航角（度）
        'hei': 30# 离地高度（米）
    }
    
    picture_queue = multiprocessing.Queue()
    target_queue = multiprocessing.Queue()
    shutdown_event = multiprocessing.Event()
    # 调用函数
    current = time.time()
    mem = time.time()
    count = 0
    while count < 100:
        run_yolo_detection(timestamp, pose_data, './picture_test/test_graph.jpg', 
                       target_queue, picture_queue , shutdown_event)
        count += 1
        print(f"time {(time.time()-mem)}, {count}")
        mem = time.time()
        run_yolo_detection(timestamp, pose_data, './picture_test/test_graph1.jpg', 
                       target_queue, picture_queue , shutdown_event)
        count += 1
        print(f"time {(time.time()-mem)}, {count}")
        mem = time.time()
    print(f"avg time {(time.time() - current) / count}")

    # run_yolo_detection(timestamp, pose_data, './picture_test/test_graph.jpg', 
    #                    target_queue, picture_queue , shutdown_event)
    # run_yolo_detection(timestamp, pose_data, './picture_test/test_graph1.jpg', 
    #                    target_queue, picture_queue , shutdown_event)
    # run_yolo_detection(timestamp, pose_data, './picture_test/test_graph2.jpg', 
    #                    target_queue, picture_queue , shutdown_event)
    # 显示结果
    # i = 0
    # while not picture_queue.empty():
    #     pic = picture_queue.get()
    #     i = i + 1
    #     cv2.imwrite(f"./picture_test/output{i}.jpg", pic)  # 参数：(窗口名称, 图像数据)
    # while not target_queue.empty():
    #     target = target_queue.get()
    #     print(target)

def test_number():
    # 加载模型
    config = configuration.Config("./config/config_test_yolo_number.json")
    initializer(config)
    locate.initializer(config)
    timestamp = 114514
    pose_data = {
        'timestamp': timestamp,  # Unix时间戳（ms）
        'lat': 0,        # 纬度（度）
        'lon': 0,        # 经度（度）
        'alt': 30,        # 海拔高度（米）
        'heading': 0,    # 航向角（度）
        'roll': 0,       # 滚转角（度）
        'pitch': 0,      # 俯仰角（度）
        'yaw': 0,        # 偏航角（度）
        'hei': 30# 离地高度（米）
    }
    
    picture_queue = multiprocessing.Queue()
    target_queue = multiprocessing.Queue()
    shutdown_event = multiprocessing.Event()
    # 调用函数
    # time.sleep(10) # 测试warmup是否有效
    # current = time.time()
    # mem = time.time()
    # count = 0
    # while count < 10:
    #     run_yolo_detection(timestamp, pose_data, './picture_test/test_number_blue.jpg', 
    #                     target_queue, picture_queue , shutdown_event)
    #     count += 1
    #     print(f"time {(time.time()-mem)}, {count}")
    #     mem = time.time()
    #     run_yolo_detection(timestamp, pose_data, './picture_test/test_number_red.jpg', 
    #                     target_queue, picture_queue , shutdown_event)
    #     count += 1
    #     print(f"time {(time.time()-mem)}, {count}")
    #     mem = time.time()
    # print(f"avg time {(time.time() - current) / count}")

    run_yolo_detection(timestamp, pose_data, './picture_test/test_number_red.jpg', 
                target_queue, picture_queue , shutdown_event)
    run_yolo_detection(timestamp, pose_data, './picture_test/test_number_blue.jpg', 
                target_queue, picture_queue , shutdown_event)

    # 显示结果
    i = 0
    while not picture_queue.empty():
        pic = picture_queue.get()
        i = i + 1
        cv2.imwrite(f"./picture_test/output{i}.jpg", pic)  # 参数：(窗口名称, 图像数据)
    while not target_queue.empty():
        target = target_queue.get()
        print(target)

if __name__ == '__main__':
    test_graph()
"""