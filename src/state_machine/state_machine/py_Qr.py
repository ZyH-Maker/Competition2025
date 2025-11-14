import subprocess
import time
from state_machine.state_topic import finishSub
import asyncio

async def async_finish():
    # 使用 asyncio.to_thread 来在后台线程中执行阻塞函数
    await asyncio.to_thread(finishSub)


async def capture_qr_message_and_print_image(program_path, timeout=1000):
    """
    启动 C++ 程序并读取输出，如果连续两次非空输出内容完全相同，则打印图片并返回该 QR 信息。

    Args:
        program_path (str): C++ 程序的路径。
        img_path (str): 要打印的图片路径。
        timeout (int): 程序的最大运行时间（秒）。

    Returns:
        str: 截取到的 QR 信息（如果连续两次输出相同的内容）。
    """
    # 启动 C++ 程序
    process = subprocess.Popen(
        [program_path],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        bufsize=1,
        universal_newlines=True
    )

    start_time = time.time()
    #last_output = None
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

            # 逐行读取 C++ 程序的标准输出
            output = process.stdout.readline()

            if output == '' and process.poll() is not None:
                break

            output = output.strip()  # 去除首尾空格
            if output:  # 确保输出非空
                print(f"Received from C++: {output}")

                # 如果当前完整输出与上一次的完整输出相同
                #if output == last_output and output != '@':
                if output != '@' and output[0] == '@':
                    print(f"Extracted QR Message: {output}")
                        # 终止 C++ 程序并返回 QR 信息
                    process.terminate()
                    process.wait()
                    print("C++ 程序已终止")
                    return output

                #last_output = output

    except KeyboardInterrupt:
        print("程序被中断，关闭 C++ 程序")

    finally:
        # 确保 C++ 程序终止
        process.terminate()
        process.wait()
        print("C++ 程序已终止")


# # # # 调用示例
# # program_path = './../FineVision/Build/Bin/Exp_Qr'  # C++ 程序的路径
# qr_message = 'Not Detect.'
# qr_message = asyncio.run(capture_qr_message_and_print_image("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Qr")) 
# print(f"Final QR Message: {qr_message}")
