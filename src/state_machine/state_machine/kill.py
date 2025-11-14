import os
import signal
import time

def stopProcesses():
    try:
        with open('/home/fins/dev_ws/pids.txt', 'r') as f:
            pids = [int(line.strip()) for line in f.readlines()]

        for pid in pids:
            try:
                print(f"[PROCESS] Try to terminate process with PID: {pid}")
                # 请求进程退出
                os.killpg(os.getpgid(pid), signal.SIGTERM)
                
                # 等待一段时间让进程清理资源
                for _ in range(5):
                    try:
                        retcode = os.waitpid(pid, os.WNOHANG)  # 检查进程状态
                        if retcode[0] > 0:  # 如果返回值是大于0，表示进程已结束
                            print("Process has been terminated successfully.")
                            break
                    except ChildProcessError:
                        break  # 进程可能已经结束
                    time.sleep(0.1)
                else:
                    # 如果进程在5秒后仍然未终止，强制杀死它
                    os.killpg(os.getpgid(pid), signal.SIGKILL)
                    print("Process was not terminated gracefully, killed forcefully.")
                    
            except ProcessLookupError:
                print(f"Process {pid} not found.")
            except PermissionError:
                print(f"Permission denied to terminate process {pid}.")
            except Exception as e:
                print(f"Error terminating process: {e}")
    
    except Exception as e:
        print(f"Error stopping processes: {e}")

    # 清空 pids.txt 文件
    try:
        with open('/home/fins/dev_ws/pids.txt', 'w') as f:
            pass  # 以写入模式打开文件时，不写入任何内容即会清空文件
    except Exception as e:
        print(f"Error clearing pids.txt: {e}")

    return


stopProcesses()
