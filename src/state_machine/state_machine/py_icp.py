import subprocess
import re
import time
import numpy as np

def capture_stable_transform(program_path, threshold=2, required_matches=2, timeout=10):
    """
    从 C++ 程序输出中提取稳定的旋转角度和平移坐标。

    Args:
        program_path (str): C++ 程序的路径。
        threshold (float): 判断数值接近的阈值。
        required_matches (int): 检测到稳定结果所需的连续接近次数。
        timeout (int): 程序的最大运行时间（秒）。

    Returns:
        tuple: 包含旋转角度（顺时针，单位：度）和 x, y 平移距离（单位：mm）的元组 (rotation, x, y)。
    """
    process = subprocess.Popen(
        [program_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    # 正则表达式匹配输出的 Rotation 和 Translate 信息
    pattern = re.compile(r"Rotation Angles \(deg\) and Translate \(mm\):\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)")

    transform_values = []
    stable_transform = None
    start_time = time.time()

    def is_close(value1, value2, threshold):
        """判断两个变换信息是否接近"""
        return np.linalg.norm(np.array(value1) - np.array(value2)) < threshold

    try:
        while True:
            elapsed_time = time.time() - start_time
            if elapsed_time > timeout:
                print("超时，停止读取输出")
                break

            output = process.stdout.readline()
            if output == '' and process.poll() is not None:
                break

            print(f"Program Output: {output.strip()}")

            match = pattern.search(output)
            if match:
                rotation, x, y = map(float, match.groups())
                transform = [rotation, x, y]
                # print(f"Extracted Transform: {transform}")

                if transform[2] > 400:
                    # print("start change")
                    # print(transform)
                    transform[1] -= 600*np.sin(-transform[0]*np.pi/180)
                    transform[2] -= 600*np.cos(-transform[0]*np.pi/180)
                    # print(transform)
                    # print("finish change")
                  

                # 检查当前值是否接近上一个值
                if transform_values and is_close(transform, transform_values[-1], threshold) :
                    transform_values.append(transform)
                else:
                    transform_values = [transform]  # 重置列表

                # 检查是否达到所需的连续接近次数
                if len(transform_values) >= required_matches and transform[2]<400:
                    stable_transform = transform
                    print(f"Stable Transform Detected: {transform}")

            # 如果稳定结果已检测到，则停止
            if stable_transform:
                break

    except KeyboardInterrupt:
        print("程序被中断，关闭 C++ 程序")

    finally:
        process.terminate()
        process.wait()
        print("C++ 程序已终止")

    # 返回最终的稳定变换信息
    if stable_transform:
        print(f"Final Stable Transform: {stable_transform}")
        return stable_transform
    else:
        print("未能检测到稳定的变换信息")
        return None

# # # 调用示例
# program_path = "/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Ex_icp"  # C++ 程序的路径
# stable_transform = capture_stable_transform(program_path)
# print(f"Extracted Stable Transform: {stable_transform}")s