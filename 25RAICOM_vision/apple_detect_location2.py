#!/home/raicom/v5lite/bin/python3.11

import os

os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "xcb"

import cv2
import numpy as np
import onnxruntime as ort
import time
import random
import serial


def plot_one_box(x, img, color=None, label=None, line_thickness=None):
    """
    description: Plots one bounding box on image img,
                 this function comes from YoLov5 project.
    param:
        x:      a box likes [x1,y1,x2,y2]
        img:    a opencv image object
        color:  color to draw rectangle, such as (0,255,0)
        label:  str
        line_thickness: int
    return:
        no return
    """
    print("x:", x)
    print("label:", label)
    tl = (
            line_thickness or round(0.002 * (img.shape[0] + img.shape[1]) / 2) + 1
    )  # line/font thickness
    color = color or [random.randint(0, 255) for _ in range(3)]
    c1, c2 = (int(x[0]), int(x[1])), (int(x[2]), int(x[3]))
    cv2.rectangle(img, c1, c2, color, thickness=tl, lineType=cv2.LINE_AA)
    if label:
        tf = max(tl - 1, 1)  # font thickness
        t_size = cv2.getTextSize(label, 0, fontScale=tl / 3, thickness=tf)[0]
        c2 = c1[0] + t_size[0], c1[1] - t_size[1] - 3
        cv2.rectangle(img, c1, c2, color, -1, cv2.LINE_AA)  # filled
        cv2.putText(
            img,
            label,
            (c1[0], c1[1] - 2),
            0,
            tl / 3,
            [225, 255, 255],
            thickness=tf,
            lineType=cv2.LINE_AA,
        )


def _make_grid(nx, ny):
    xv, yv = np.meshgrid(np.arange(ny), np.arange(nx))
    return np.stack((xv, yv), 2).reshape((-1, 2)).astype(np.float32)


def cal_outputs(outs, nl, na, model_w, model_h, anchor_grid, stride):
    row_ind = 0
    grid = [np.zeros(1)] * nl
    for i in range(nl):
        h, w = int(model_w / stride[i]), int(model_h / stride[i])
        length = int(na * h * w)
        if grid[i].shape[2:4] != (h, w):
            grid[i] = _make_grid(w, h)

        outs[row_ind:row_ind + length, 0:2] = (outs[row_ind:row_ind + length, 0:2] * 2. - 0.5 + np.tile(
            grid[i], (na, 1))) * int(stride[i])
        outs[row_ind:row_ind + length, 2:4] = (outs[row_ind:row_ind + length, 2:4] * 2) ** 2 * np.repeat(
            anchor_grid[i], h * w, axis=0)
        row_ind += length
    return outs


def post_process_opencv(outputs, model_h, model_w, img_h, img_w, thred_nms, thred_cond):
    conf = outputs[:, 4].tolist()
    c_x = outputs[:, 0] / model_w * img_w
    c_y = outputs[:, 1] / model_h * img_h
    w = outputs[:, 2] / model_w * img_w
    h = outputs[:, 3] / model_h * img_h
    p_cls = outputs[:, 5:]
    if len(p_cls.shape) == 1:
        p_cls = np.expand_dims(p_cls, 1)
    cls_id = np.argmax(p_cls, axis=1)

    p_x1 = np.expand_dims(c_x - w / 2, -1)
    p_y1 = np.expand_dims(c_y - h / 2, -1)
    p_x2 = np.expand_dims(c_x + w / 2, -1)
    p_y2 = np.expand_dims(c_y + h / 2, -1)
    areas = np.concatenate((p_x1, p_y1, p_x2, p_y2), axis=-1)

    areas = areas.tolist()
    ids = cv2.dnn.NMSBoxes(areas, conf, thred_cond, thred_nms)
    if len(ids) > 0:
        return np.array(areas)[ids], np.array(conf)[ids], cls_id[ids]
    else:
        return [], [], []


def infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid, thred_nms=0.4, thred_cond=0.5):
    # 图像预处理
    img = cv2.resize(img0, [model_w, model_h], interpolation=cv2.INTER_AREA)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    img = img.astype(np.float32) / 255.0
    blob = np.expand_dims(np.transpose(img, (2, 0, 1)), axis=0)

    # 模型推理
    outs = net.run(None, {net.get_inputs()[0].name: blob})[0].squeeze(axis=0)

    # 输出坐标矫正
    outs = cal_outputs(outs, nl, na, model_w, model_h, anchor_grid, stride)

    # 检测框计算
    img_h, img_w, _ = np.shape(img0)
    boxes, confs, ids = post_process_opencv(outs, model_h, model_w, img_h, img_w, thred_nms, thred_cond)

    return boxes, confs, ids


if __name__ == "__main__":
    # 配置串口模块
    port = '/dev/ttyAMA0'
    baudrate = 115200
    ser = serial.Serial(port, baudrate, timeout=1)

    # 果实大小
    fruit_width = 55
    fruit_height = 50
    # 中心点坐标
    center_x = 320
    center_y = 240
    # 测算距离参数
    K = 17100  # 需要测算修改，距离×像素值
    # 测算距离
    distance = 0
    # 测算偏移量
    x_offset = 0
    y_offset = 0

    # 模型加载
    model_pb_path = "best2.onnx"
    so = ort.SessionOptions()
    net = ort.InferenceSession(model_pb_path, so)

    # 标签字典
    dic_labels = {0: 'green', 1: 'red'}

    # 模型参数
    model_h = 320
    model_w = 320
    nl = 3
    na = 3
    stride = [8., 16., 32.]
    anchors = [[10, 13, 16, 30, 33, 23], [30, 61, 62, 45, 59, 119], [116, 90, 156, 198, 373, 326]]
    anchor_grid = np.asarray(anchors, dtype=np.float32).reshape(nl, -1, 2)

    video = 0
    cap = cv2.VideoCapture(video)

    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(width, height)

    flag_det = False  # 初始化为不检测状态
    
    task_index = -1

    # 定义保存图像的目录
    output_dir = "output_images"
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
    while True:
        # 检查串口消息
        if ser.in_waiting > 0:
            serial_msg = ser.readline().decode('ascii').strip()
            print("接收到串口消息:", serial_msg)

            # 这里判断接收到的串口消息
            if serial_msg == "one":  # 设定一个触发消息
                for i in range(10):
                    success, img0 = cap.read()
                    if success:
                        t1 = time.time()
                        det_boxes, scores, ids = infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid,
                                                           thred_nms=0.4, thred_cond=0.65)
                        t2 = time.time()

                        # 查找面积最大的检测框
                        if len(det_boxes) > 0:
                            areas = [(box[2] - box[0]) * (box[3] - box[1]) for box in det_boxes]
                            max_area_index = np.argmax(areas)
                            largest_box = det_boxes[max_area_index]
                            largest_score = scores[max_area_index]
                            largest_id = ids[max_area_index]

                            # 处理最大的检测框
                            label = '%s:%.2f' % (dic_labels[largest_id], largest_score)

                            weight = largest_box[2] - largest_box[0]
                            height = largest_box[3] - largest_box[1]

                            # 计算物体离摄像头距离
                            Lm = (weight + height) / 2
                            distance = K / Lm

                            # 计算物体偏移量
                            x_offset = ((center_x - (largest_box[2] + largest_box[0]) / 2) / weight) * fruit_width
                            y_offset = -(((largest_box[3] + largest_box[1]) / 2 - center_y) / height) * fruit_height

                            # 画框
                            print(dic_labels[largest_id])
                            plot_one_box(largest_box.astype(np.int16), img0, color=(255, 0, 0), label=label,
                                         line_thickness=None)

                            # 串口发送信息
                            rounded_distance = round(distance)
                            rounded_x_offset = round(x_offset)
                            rounded_y_offset = round(y_offset)

                            formatted_distance = '{:+05d}'.format(rounded_distance)
                            formatted_x_offset = '{:+05d}'.format(rounded_x_offset)
                            formatted_y_offset = '{:+05d}'.format(rounded_y_offset)

                            uart_message = str(
                                largest_id) + ',' + formatted_distance + ',' + formatted_x_offset + ',' + formatted_y_offset + '\r\n'
                            uart_message_bytes = uart_message.encode('ascii')
                            # ser.write(uart_message_bytes)
                            # print("串口发送:", uart_message)
                        else:
                            rounded_distance = round(0)
                            rounded_x_offset = round(0)
                            rounded_y_offset = round(0)

                            formatted_distance = '{:+05d}'.format(rounded_distance)
                            formatted_x_offset = '{:+05d}'.format(rounded_x_offset)
                            formatted_y_offset = '{:+05d}'.format(rounded_y_offset)

                            uart_message = str(
                                0) + ',' + formatted_distance + ',' + formatted_x_offset + ',' + formatted_y_offset + '\r\n'
                            uart_message_bytes = uart_message.encode('ascii')
                            # ser.write(uart_message_bytes)
                            # print("串口发送(没有检测到任何):", uart_message)
                            continue

                        # 仅在最后一个循环发送串口消息
                        if i == 9:
                            ser.write(uart_message_bytes)
                            print("串口发送:", uart_message)

                        str_FPS = "FPS: %.2f" % (1. / (t2 - t1))
                        cv2.putText(img0, str_FPS, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)

                        # 保存图像，文件名包含时间戳
                        image_path = os.path.join(output_dir, f"image_taskindex:_{task_index}.jpg")
                        task_index = task_index + 1
                        cv2.imwrite(image_path, img0)
                        print(f"图像已保存为: {image_path}")

                    # cv2.imshow("video", img0)

            elif serial_msg == "two":# 设定一个触发消息
                for i in range(10):
                    success, img0 = cap.read()
                    if success:
                        t1 = time.time()
                        det_boxes, scores, ids = infer_img(img0, net, model_h, model_w, nl, na, stride, anchor_grid,
                                                           thred_nms=0.4, thred_cond=0.65)
                        t2 = time.time()

                        # 查找面积最大的检测框
                        if len(det_boxes) > 0:
                            areas = [(box[2] - box[0]) * (box[3] - box[1]) for box in det_boxes]

                            # 查找ID为1的最大面积检测框
                            max_area_id1 = -1
                            largest_box_id1 = None

                            for i, id in enumerate(ids):
                                if id == 1:
                                    if max_area_id1 == -1 or areas[i] > areas[max_area_id1]:
                                        max_area_id1 = i
                                        largest_box_id1 = det_boxes[i]

                            if largest_box_id1 is not None:
                                largest_box = largest_box_id1
                                largest_id = 1
                            else:
                                max_area_index = np.argmax(areas)
                                largest_box = det_boxes[max_area_index]
                                largest_id = ids[max_area_index]

                            label = '%s:%.2f' % (dic_labels[largest_id], scores[max_area_index])

                            weight = largest_box[2] - largest_box[0]
                            height = largest_box[3] - largest_box[1]

                            Lm = (weight + height) / 2
                            distance = K / Lm

                            x_offset = ((center_x - (largest_box[2] + largest_box[0]) / 2) / weight) * fruit_width
                            y_offset = -(((largest_box[3] + largest_box[1]) / 2 - center_y) / height) * fruit_height

                            print(dic_labels[largest_id])
                            plot_one_box(largest_box.astype(np.int16), img0, color=(255, 0, 0), label=label,
                                         line_thickness=None)

                            rounded_distance = round(distance)
                            rounded_x_offset = round(x_offset)
                            rounded_y_offset = round(y_offset)

                            formatted_distance = '{:+05d}'.format(rounded_distance)
                            formatted_x_offset = '{:+05d}'.format(rounded_x_offset)
                            formatted_y_offset = '{:+05d}'.format(rounded_y_offset)

                            uart_message = str(
                                largest_id) + ',' + formatted_distance + ',' + formatted_x_offset + ',' + formatted_y_offset + '\r\n'
                            uart_message_bytes = uart_message.encode('ascii')
                            ser.write(uart_message_bytes)
                            print("串口发送:", uart_message)
                        else:
                            rounded_distance = round(0)
                            rounded_x_offset = round(0)
                            rounded_y_offset = round(0)

                            formatted_distance = '{:+05d}'.format(rounded_distance)
                            formatted_x_offset = '{:+05d}'.format(rounded_x_offset)
                            formatted_y_offset = '{:+05d}'.format(rounded_y_offset)

                            uart_message = str(
                                0) + ',' + formatted_distance + ',' + formatted_x_offset + ',' + formatted_y_offset + '\r\n'
                            uart_message_bytes = uart_message.encode('ascii')
                            ser.write(uart_message_bytes)
                            print("串口发送:", uart_message)
                        str_FPS = "FPS: %.2f" % (1. / (t2 - t1))
                        cv2.putText(img0, str_FPS, (50, 50), cv2.FONT_HERSHEY_COMPLEX, 1, (0, 255, 0), 3)

                        # 获取当前时间戳
                        # timestamp = time.strftime("%Y%m%d_%H%M%S")

                        # 保存图像，文件名包含时间戳
                        image_path = os.path.join(output_dir, f"image_taskindex:_{task_index}.jpg")
                        task_index = task_index + 1
                        cv2.imwrite(image_path, img0)
                        print(f"图像已保存为: {image_path}")
                    # cv2.imshow("video", img0)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    ser.close()


