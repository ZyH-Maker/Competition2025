import subprocess
import re
import time
import numpy as np
import rclpy
from state_machine.state_topic import finishSub
import asyncio

async def async_finish():
    # 使用 asyncio.to_thread 来在后台线程中执行阻塞函数
    await asyncio.to_thread(finishSub)

async def capture_stable_coordinates(program_path, threshold=1, required_matches=3, timeout=60):
    """
    从 C++ 程序输出中提取 Circle_Red, Circle_Green 和 Circle_Blue 的稳定坐标。

    Args:
        program_path (str): C++ 程序的路径。
        threshold (float): 判断坐标接近的距离阈值。
        required_matches (int): 检测到稳定坐标所需的连续接近次数。
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
     # 正则表达式匹配输出的 Rotation 和 Translate 信息
    patterns2 = re.compile(r"Rotation Angles \(deg\) and Translate \(mm\):\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)")



    # 存储每种颜色的接近坐标
    color_coords = {"red": [], "green": [], "blue": []}
    stable_coords = {"red": None, "green": None, "blue": None}
    transform_values = []
    stable_transform = None
    start_time = time.time()
    num = 0

    def is_close(coord1, coord2, threshold):
        """判断两个坐标是否接近"""
        return np.linalg.norm(np.array(coord1) - np.array(coord2)) < threshold

    #rclpy.init(args=None)
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
            if output == '' and process.poll() is not None:
                break

            # print(f"Program Output: {output.strip()}")
            if num == 0:
                for color, pattern in patterns.items():
                    match = pattern.search(output)
                    if match:
                        coord = tuple(map(float, match.groups()))
                        # print(f"Extracted {color.capitalize()} Coordinate: {coord}")

                        # 检查当前坐标是否接近上一个坐标
                        if color_coords[color] and is_close(coord, color_coords[color][-1], threshold):
                            color_coords[color].append(coord)
                        else:
                            color_coords[color] = [coord]  # 重置列表

                        # 检查是否达到所需的连续接近次数
                        if len(color_coords[color]) >= required_matches:
                            stable_coords[color] = coord
                            print(f"Stable {color.capitalize()} Coordinate Detected: {coord}")

                if all(stable_coords.values()):
                    num += 1
                          
       
            match2 = patterns2.search(output)
            if match2:
                rotation, x, y = map(float, match2.groups())
                transform = [rotation, x, y]
                print(f"Extracted Transform: {transform}")
                    
                if transform[2] > 400:
                        # print("start change")
                        # print(transform)
                        transform[1] -= 600*np.sin(-transform[0]*np.pi/180)
                        transform[2] -= 600*np.cos(-transform[0]*np.pi/180)
                        # print(transform)
                        # print("finish change")
                # 检查当前值是否接近上一个值
                if transform_values and is_close(transform, transform_values[-1], threshold) and transform[2] < 500:
                    transform_values.append(transform)
                else:
                    transform_values = [transform]  # 重置列表

                # 检查是否达到所需的连续接近次数
                if len(transform_values) >= required_matches and transform[2] < 500:
                    stable_transform = transform
                    print(f"Stable Transform Detected: {transform}")

                # 如果稳定结果已检测到，则停止
            if stable_transform and all(stable_coords.values()):
                break


    except KeyboardInterrupt:
        print("程序被中断，关闭 C++ 程序")

    finally:
        process.terminate()
        process.wait()
        print("C++ 程序已终止")

    if all(stable_coords.values()) and stable_transform:
        result = [stable_coords["red"], stable_coords["green"], stable_coords["blue"]]
        return (tuple(result[0]), tuple(result[1]), tuple(result[2]), stable_transform)
    else:
        print("未能检测到所有稳定坐标或变换")
        return None
    
async def capture_stable_coordinates2(program_path, threshold=1, required_matches=3, timeout=60):
    """
    从 C++ 程序输出中提取 Circle_Red, Circle_Green 和 Circle_Blue 的稳定坐标。

    Args:
        program_path (str): C++ 程序的路径。
        threshold (float): 判断坐标接近的距离阈值。
        required_matches (int): 检测到稳定坐标所需的连续接近次数。
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
     # 正则表达式匹配输出的 Rotation 和 Translate 信息
  


    # 存储每种颜色的接近坐标
    color_coords = {"red": [], "green": [], "blue": []}
    stable_coords = {"red": None, "green": None, "blue": None}

    start_time = time.time()
    num = 0

    def is_close(coord1, coord2, threshold):
        """判断两个坐标是否接近"""
        return np.linalg.norm(np.array(coord1) - np.array(coord2)) < threshold

    #rclpy.init(args=None)
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
            if output == '' and process.poll() is not None:
                break

            # print(f"Program Output: {output.strip()}")
           
            for color, pattern in patterns.items():
                match = pattern.search(output)
                if match:
                    coord = tuple(map(float, match.groups()))
                        # print(f"Extracted {color.capitalize()} Coordinate: {coord}")

                        # 检查当前坐标是否接近上一个坐标
                    if color_coords[color] and is_close(coord, color_coords[color][-1], threshold):
                        color_coords[color].append(coord)
                    else:
                        color_coords[color] = [coord]  # 重置列表

                        # 检查是否达到所需的连续接近次数
                    if len(color_coords[color]) >= required_matches:
                        stable_coords[color] = coord
                        print(f"Stable {color.capitalize()} Coordinate Detected: {coord}")
                        
                # 如果稳定结果已检测到，则停止
            if all(stable_coords.values()):
                break


    except KeyboardInterrupt:
        print("程序被中断，关闭 C++ 程序")

    finally:
        process.terminate()
        process.wait()
        print("C++ 程序已终止")

    if all(stable_coords.values()) :
        result = [stable_coords["red"], stable_coords["green"], stable_coords["blue"]]
        return result
    else:
        print("未能检测到所有稳定坐标或变换")
        return None


# stable_coordinates = capture_stable_coordinates("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo") 
#print(f"Extracted Stable Coordinates (Red, Green, Blue): {stable_coordinates}")
#program_path = "/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo"  # C++ 程序的路径
# program_path = "/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo2"
# result = asyncio.run(capture_stable_coordinates2(program_path,3 ,2, 60))
# if result:
#     print(f"Final Result: {result}")
# else:
#     print("未能获取稳定坐标和变换")