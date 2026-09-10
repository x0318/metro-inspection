# pipeline.py
import cv2
import time
from ultralytics import YOLO

def pixel_to_physical(u_center, v_center, current_mileage):
    """
    【核心加分点：空间坐标解算接口】
    把缺陷在画面中的2D像素坐标 (u, v)，转换成地铁工程里通用的物理方位。
    假设我们的视频分辨率被缩放到了 640x480。
    """
    # 1. 根据像素点在图像中的左右位置，死死卡住它的“时钟方位”
    if u_center < 220:
        orientation = "9点钟方向 (左侧隧道管片 Lining)"
    elif u_center > 420:
        orientation = "3点钟方向 (右侧隧道管片 Lining)"
    else:
        # 如果在中间，再根据上下判断是道床还是拱顶
        if v_center > 300:
            orientation = "6点钟方向 (轨道道床 Trackbed)"
        else:
            orientation = "12点钟方向 (隧道顶部 Crown)"
            
    # 2. 把当前算出来的物理里程格式化（例如将 12450.32 米变成标准工务格式 K12+450.32m）
    km = int(current_mileage // 1000)
    meters = current_mileage % 1000
    mileage_str = f"K{km}+{meters:.2f}m"
    
    return mileage_str, orientation


def main():
    # ==========================================
    # 1. 初始化配置与大脑加载
    # ==========================================
    # 明确指向你刚刚经历50轮轰鸣诞生的最强大脑
    model_path = "runs/detect/practice_runs/crack_test/weights/best.pt"
    print(f"🧠 正在加载定制AI大脑: {model_path} ...")
    model = YOLO(model_path)
    
    # 打开视频流门锁
    video_path = "tunnel_test.mp4"
    cap = cv2.VideoCapture(video_path)
    
    if not cap.isOpened():
        print(f"❌ 错误：打不开视频文件 {video_path}，请检查文件名和路径！")
        return

    # 获取视频的原始帧率（每秒多少帧），如果获取不到，默认设为 25 帧
    fps = cap.get(cv2.CAP_PROP_FPS)
    if fps == 0 or fps > 100: fps = 25
    
    # 仿真参数设定
    frame_id = 0
    start_mileage = 12450.00  # 模拟巡检车从 K12+450.00 米处出发
    inspect_speed_kmh = 50.0   # 模拟巡检车时速 50 公里/小时
    
    # 换算：每帧图像过去，巡检车前进了多少米
    # 50 km/h = 13.88 米/秒。 每帧前进米数 = 13.88 / FPS
    speed_ms = inspect_speed_kmh / 3.6
    meters_per_frame = speed_ms / fps

    print("🚀 智能巡检流水线全线点火！按下键盘上的 'Q' 键可以随时退出。")

    # ==========================================
    # 2. 视频帧连续吞噬主循环
    # ==========================================
    while cap.isOpened():
        # ret: 布尔值，代表有没有成功读到图； frame: 那张图片的真实矩阵数据
        ret, frame = cap.read()
        if not ret:
            print("🏁 视频播放结束或图流中断，流水线自动停机。")
            break
            
        frame_id += 1
        
        # 为了配合 YOLO 的标准，同时让你的笔记本不卡，强行把画面缩放到 640x480
        frame = cv2.resize(frame, (640, 480))
        
        # 实时计算当前车子开到了多少米
        current_mileage = start_mileage + (frame_id * meters_per_frame)

        # ------------------------------------------
        # 核心步骤 A：YOLOv8 深度学习推理
        # ------------------------------------------
        # conf=0.20 代表只信任把握度大于 20% 的警报。verbose=False 让终端保持干净
        results = model.predict(frame, conf=0.20, verbose=False)
        
        # 提取所有的预测框
        boxes = results[0].boxes
        
        # ------------------------------------------
        # 核心步骤 B：解析警报并提取几何特征
        # ------------------------------------------
        for box in boxes:
            # 拿到方框的左上角和右下角像素坐标 [x1, y1, x2, y2]
            xyxy = box.xyxy[0].cpu().numpy()
            x1, y1, x2, y2 = int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])
            
            # 算出这个缺陷方框的几何中心点
            u_center = int((x1 + x2) / 2)
            v_center = int((y1 + y2) / 2)
            
            # 提取底气值（置信度）和类别ID
            conf_score = float(box.conf[0])
            cls_id = int(box.cls[0])
            class_name = model.names[cls_id] # 拿到名字，比如 '0' 或 'crack'
            
            # ------------------------------------------
            # 核心步骤 C：拦截坐标，反求物理时空方位
            # ------------------------------------------
            mileage_display, orientation_display = pixel_to_physical(u_center, v_center, current_mileage)
            
            # ------------------------------------------
            # 核心步骤 D：动态 UI 绘制（喷涂缺陷警报）
            # ------------------------------------------
            # 1. 画缺陷红框（BGR格式：(0, 0, 255) 代表纯红色，粗细为 2）
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
            
            # 2. 在框上方喷涂缺陷名字和底气值
            alert_txt = f"{class_name} ({conf_score:.1%})"
            cv2.putText(frame, alert_txt, (x1, y1 - 8), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # 3. 打印工程定位日志（这行字以后可以直接塞进 Excel 报告里）
            print(f"⚠️ [发现病害] 类型: {class_name} | 位置: {mileage_display} | 方位: {orientation_display}")

        # ------------------------------------------
        # 核心步骤 E：喷涂“中铁十一局大厂标准”巡检虚拟仪表盘
        # ------------------------------------------
        # 在画面左上角画一个半透明的黑底科技看板
        cv2.rectangle(frame, (10, 10), (320, 100), (0, 0, 0), -1) 
        
        # 实时刷写当前行驶总公里里程数
        km_now = inspect_speed_kmh
        km_str = f"K{inspect_speed_kmh // 1000}"
        
        cv2.putText(frame, f"SYS: MULTI-FIELD SEG INSPECTION", (20, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1) # 黄字
        
        # 计算当前漂亮的工程里程展示
        km_main = int(current_mileage // 1000)
        m_main = current_mileage % 1000
        cv2.putText(frame, f"Mileage: K{km_main}+{m_main:.2f}m", (20, 60), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)  # 绿字
        
        cv2.putText(frame, f"Speed: {inspect_speed_kmh} km/h | Frame: {frame_id}", (20, 85), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1) # 白字

        # ==========================================
        # 3. 实时出图与窗口防御机制
        # ==========================================
        # 啪地一下把涂抹好的科技感画面在屏幕上展示出来
        cv2.imshow("CRCC11 - Subway Intelligent Inspection Engine", frame)
        
        # 防御机制：每帧停顿 1 毫秒。如果用户按了键盘上的 'q' 或 'Q'，立刻强行安全退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            print("🛑 收到用户强行中止指令，流水线安全紧急关停！")
            break

    # 善后打扫工作
    cap.release()
    cv2.destroyAllWindows()
    print("✨ 流水线完全关闭，底盘空转演练圆满成功！")

if __name__ == "__main__":
    main()