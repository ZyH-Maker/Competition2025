import subprocess
import re
import time
import numpy as np
import rclpy
from state_machine.state_topic import finishSub
import asyncio
from datetime import datetime

async def async_finish():
    # 使用 asyncio.to_thread 来在后台线程中执行阻塞函数
    await asyncio.to_thread(finishSub)
        

async def capture_move_coordinates(item_color, ready_item, program_path, threshold=3, required_matches=3, significant_change=10, timeout=60):
    """
    从 C++ 程序输出中提取 Circle_Red, Circle_Green 和 Circle_Blue 的稳定坐标。

    Args:
        item_color:本轮需要抓取的颜色， 需要判断数值位置是否适合抓取
        ready_item:已完成抓取的物块，不同再次识别，直接赋值不影响规划的位置
        program_path (str): C++ 程序的路径。
        threshold (float): 判断坐标接近的距离阈值。
        required_matches (int): 检测到稳定坐标所需的连续接近次数。
        significant_change (float): 判断显著变化的阈值（单位为 mm）。
        timeout (int): 程序的最大运行时间（秒）。

    Returns:
        list: 按顺序包含 Red, Green, Blue 的稳定坐标列表 [(x1, y1, z1), (x2, y2, z2), (x3, y3, z3)]。
    """
    process = subprocess.Popen(
        [program_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # 正则表达式匹配输出的 Circle 信息
    patterns = {
        "red": re.compile(r"Circle_Red: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
        "green": re.compile(r"Circle_Green: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
        "blue": re.compile(r"Circle_Blue: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
    }

    #rclpy.init(args=None)
    # #添加时间戳 后续删除
    # file_name = 'time_check.txt'
    # current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    # with open(file_name, 'a', encoding='utf-8') as file:
    #     file.write("开始\n")
    #     file.write(f'当前时间戳: {current_timestamp}\n')
    # print(f'文本和时间戳已保存到 {file_name}')
    # #添加时间戳 后续删除

    # rclpy.init(args=None)
    # finishSub()

    # #添加时间戳 后续删除
    # file_name = 'time_check.txt'
    # current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
    # with open(file_name, 'a', encoding='utf-8') as file:
    #     file.write("阻塞结束\n")
    #     file.write(f'当前时间戳: {current_timestamp}\n')
    # print(f'文本和时间戳已保存到 {file_name}')
    # #添加时间戳 后续删除

    color_coords = {"red": [], "green": [], "blue": []}
    index_to_color = {1: "red", 2: "green", 3: "blue"}
    stable_coords = {"red": None, "green": None, "blue": None}
    initial_coords = {"red": None, "green": None, "blue": None}
    tracking_active = False
    start_time = time.time()

    #对完成任务的颜色，直接赋值一个不会影响机械臂规划的位置
    for num in ready_item:
        stable_coords[index_to_color[num]] = (-2000.0+num*1000.0, 1000.0, 0.0)

    def is_close(coord1, coord2, threshold):
        """判断两个坐标是否接近"""
        return np.linalg.norm(np.array(coord1) - np.array(coord2)) < threshold

    def has_significant_change(coord1, coord2, change_threshold):
        """判断两个坐标是否有显著变化"""
        return (np.linalg.norm(np.array(coord1) - np.array(coord2)) > change_threshold and np.linalg.norm(np.array(coord1) - np.array(coord2))<60)
    
    def is_distance_valid(coords, min_distance):
        """检查最后三个坐标之间的距离是否大于 min_distance"""
        if len(coords) < 3:
            return True  # 如果没有足够的坐标，默认返回 True

        dist1 = np.linalg.norm(np.array(coords[-1]) - np.array(coords[-2]))
        dist2 = np.linalg.norm(np.array(coords[-2]) - np.array(coords[-3]))

        return dist1 > min_distance and dist2 > min_distance

    rclpy.init(args=None)
   # 创建信号监听任务
    signal_task = asyncio.create_task(async_finish())  
    # 主程序循环
    while not signal_task.done():
        process.stdout.readline()
        await asyncio.sleep(0.01)  # 这里可以根据需要设置循环的间隔  
    print("开始识别")

    try:
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                print("超时，停止读取输出")
                break

            output = process.stdout.readline()
            #print(output)
            if output == '' and process.poll() is not None:
                break

            for color, pattern in patterns.items():
                match = pattern.search(output)

                #如果这个颜色已经完成任务，则不需要识别
                for num in ready_item:
                    if index_to_color[num] == color:
                        match = None

                if match:
                    coord = tuple(map(float, match.groups()))

                    if not initial_coords[color]:
                        # 第一次识别到的坐标设为初始坐标
                        initial_coords[color] = coord
                        print(f"Initial {color.capitalize()} Coordinate: {coord}")
                        # #添加时间戳 后续删除
                        # file_name = 'time_check.txt'
                        # current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        # with open(file_name, 'a', encoding='utf-8') as file:
                        #     file.write("开始识别\n")
                        #     file.write(f'当前时间戳: {current_timestamp}\n')
                        # print(f'文本和时间戳已保存到 {file_name}')
                        # #添加时间戳 后续删除
                        continue

                    if not tracking_active and has_significant_change(coord, initial_coords[color], significant_change):
                        # 如果与初始坐标的差距显著，则开始记录数据
                        # #添加时间戳 后续删除
                        # # 指定要保存文本的文件名
                        # file_name = 'time_check.txt'
                        # # 获取当前时间戳
                        # current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        # # 打开文件并写入内容，如果文件不存在则会自动创建
                        # with open(file_name, 'a', encoding='utf-8') as file:
                        #     file.write("检测到转动开始\n")
                        #     file.write(f'当前时间戳: {current_timestamp}\n')
                        # print(f'文本和时间戳已保存到 {file_name}')
                        # #添加时间戳 后续删除
                        print(f"Significant change detected for {color.capitalize()} at {coord}, starting to track.")
                        tracking_active = True

                    if tracking_active:
                        if color_coords[color] and is_close(coord, color_coords[color][-1], threshold):
                            color_coords[color].append(coord)
                        else:
                            color_coords[color] = [coord]

                        # 检查是否达到所需的连续接近次数
                        if len(color_coords[color]) >= required_matches:
                            stable_coords[color] = coord
                            
                            print(f"Stable {color.capitalize()} Coordinate Detected: {coord}")

            # 如果所有颜色的稳定坐标都已检测到，则停止
            if all(stable_coords.values()):
                # 检查稳定坐标之间的距离是否符合要求
                all_coords = [stable_coords["red"], stable_coords["green"], stable_coords["blue"]]
                if is_distance_valid(all_coords, min_distance=100):
                    color_key = index_to_color[item_color]
                    if stable_coords[color_key][1] > 110 and stable_coords[color_key][0] > -170 and (stable_coords[color_key][1]**2+stable_coords[color_key][0]**2 < 122500):
                        break
                    else:
                        print("11111111111111!!")
                        for key in stable_coords.keys():
                            stable_coords[key] = None
                        for num in ready_item:
                            stable_coords[index_to_color[num]] = (-2000.0+num*1000.0, 1000.0, 0.0)

                else:
                    print("Stable coordinates do not satisfy the minimum distance requirement, restarting detection...")
                    # 清空已有数据，重新识别
                    stable_coords = {"red": None, "green": None, "blue": None}
                    color_coords = {"red": [], "green": [], "blue": []}
                    initial_coords = {"red": None, "green": None, "blue": None}
                    #tracking_active = False
                    #start_time = time.time()
                
    except KeyboardInterrupt:
        print("程序被中断，关闭 C++ 程序")

    finally:
        process.terminate()
        process.wait()
        print("C++ 程序已终止")

    # 返回最终的稳定坐标
    if all(stable_coords.values()):
        result = [stable_coords["red"], stable_coords["green"], stable_coords["blue"]]
        print(f"Final Stable Coordinates: {result}")
        return result
    else:
        print("未能检测到所有稳定坐标")
        return None

# # # # 调用示例
pos = asyncio.run(capture_move_coordinates(2, [3],"/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo_P2"))
print(f"Extracted Stable Coordinates:{pos}")
