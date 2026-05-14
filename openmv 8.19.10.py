import sensor, image, time,pyb
from pyb import UART

uart = pyb.UART(3, 115200)

# 初始化摄像头
sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)  # 设置为灰度模式
sensor.set_framesize(sensor.QVGA)       # 分辨率320x240
sensor.skip_frames(time=2000)           # 等待摄像头稳定
# sensor.set_vflip(True)
# sensor.set_hmirror(True)
LED1=pyb.LED(1)#pyb是模块，led是其中一个类
LED2=pyb.LED(2)
LED3=pyb.LED(3)
LED1.on()
LED2.on()
LED3.on()
BLACK_THRESHOLD = (0, 90)  # 推荐范围：0-50（值越小检测越严格的黑色）

# 定义色块检测参数
MIN_PIXELS = 50     # 最小像素数（过滤噪声）
MAX_PIXELS = 20000   # 最大像素数（过滤大面积区域）
AREA_THRESHOLD = 80  # 区域面积阈值（可选）
clock = time.clock()
max_density=0.35
max_solidity=0.65
max_convexity=0.7
min_area=50
max_area=20000

# 数据打包函数
def pack_data(x, y):
    # 限制坐标范围
    x = max(0, min(x, 319))  # QVGA 分辨率为 320x240
    y = max(0, min(y, 239))

    # 打包为 2 字节数据
    data = bytearray([
        0x2C,                     # 起始字节
        (x >> 8) & 0xFF,          # X 高字节
        x & 0xFF,                 # X 低字节
        (y >> 8) & 0xFF,          # Y 高字节
        y & 0xFF,                 # Y 低字节
        0x5B                      # 结束字节
    ])
    return data

def line_intersection(line1, line2):
    """
    计算两条直线的交点坐标
    """
    (x1, y1, x2, y2) = line1
    (x3, y3,x4, y4) = line2

    # 计算分母（判断是否平行）
    den = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if den == 0:
        # 分母为0，两直线平行或重合，无唯一交点
        return (abs(x2-x1)//2,abs(y2-y1)//2)

    # 计算分子
    t_num = (x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)
    s_num = (x1 - x3) * (y1 - y2) - (y1 - y3) * (x1 - x2)

    t = t_num / den
    s = s_num / den

    # 计算交点坐标
    x = x1 + t * (x2 - x1)
    y = y1 + t * (y2 - y1)

    return (x, y)
while True:
    img = sensor.snapshot()         # 捕获一帧图像
    clock.tick()

    # 查找符合阈值的色块
    blobs = img.find_blobs([BLACK_THRESHOLD],
                          pixels_threshold=MIN_PIXELS,
                          area_threshold=AREA_THRESHOLD,
                          x_stride=1,
                          y_stride=1,
                          margin=15,
                          )  # 合并相邻色块

    if blobs:
        valid_blobs = []
        for blob in blobs:
            if blob.density()<max_density:
                if blob.area()>min_area and blob.area()<max_area:
                    # if blob.w()>blob.h():
                        # print(blob.density(),blob.solidity(),blob.convexity())#像素数除以其边界框区域,使用最小区域旋转矩形与边界矩形来测量密度,对象的凸度
                        ##0.152698 0.167296 0.423312
                        # valid_blobs.append(blob)
                    if blob.solidity()<max_solidity:
                        if blob.convexity()<max_convexity:
                            valid_blobs.append(blob)


    if valid_blobs:
            min_density_blob = min(valid_blobs, key=lambda b: b.density())
            img.draw_line(min_density_blob.major_axis_line())
            img.draw_line(min_density_blob.minor_axis_line())
            intersection=line_intersection(min_density_blob.major_axis_line(), min_density_blob.minor_axis_line())

            lx1=abs(min_density_blob.major_axis_line()[0]-min_density_blob.major_axis_line()[2])
            ly1=abs(min_density_blob.major_axis_line()[1]-min_density_blob.major_axis_line()[3])
            l1=lx1**2+ly1**2
            l1=l1**(1/2)
            lx2=abs(min_density_blob.minor_axis_line()[0]-min_density_blob.minor_axis_line()[2])
            ly2=abs(min_density_blob.minor_axis_line()[1]-min_density_blob.minor_axis_line()[3])
            l2=lx2**2+ly2**2
            l2=l2**(1/2)
            #print(l2/l1)
            if 0.55<l2/l1 and 50 <= int(intersection[1]) <= 190:  # 直接在条件中加入Y坐标限制
            # if 0.6<l2/l1:
                #img.draw_rectangle(min_density_blob.rect(), color=(255,0,0))  # 红色矩形框
                img.draw_cross(min_density_blob.min_corners()[0][0], min_density_blob.min_corners()[0][1])
                img.draw_cross(min_density_blob.min_corners()[1][0], min_density_blob.min_corners()[1][1])
                img.draw_cross(min_density_blob.min_corners()[2][0], min_density_blob.min_corners()[2][1])
                img.draw_cross(min_density_blob.min_corners()[3][0], min_density_blob.min_corners()[3][1])
                img.draw_cross(int(min_density_blob.cxf()), int(min_density_blob.cyf()), color=(255,0,255))
                img.draw_circle(min_density_blob.enclosing_circle(),color=(255,255,0), thickness=2, fill=False)
                img.draw_cross(int(intersection[0]),int(intersection[1]), color=(255,0,0))
                # print(intersection)#靶心坐标
                #串口发送示例
                # 获取中心坐标
                center_x = int(intersection[0]*100)
                center_y = int(intersection[1]*100)

                # 打包数据并发送
                try:
                    data = pack_data(center_x, center_y)
                    uart.write(data)
                    print("Sent data: X={}, Y={}".format(center_x, center_y))
                except Exception as e:
                    print("UART write error:", e)
                time.sleep_ms(80)
    print(clock.fps())

