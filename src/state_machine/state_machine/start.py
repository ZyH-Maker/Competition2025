import subprocess
import time

# 定义要运行的命令
commands = [
    "ros2 run fines_serial fines_serial",
    "ros2 run mybot trajectory_buffer_node",
    "ros2 run state_machine run_state_machine"
]

# 遍历命令列表，为每个命令打开一个新的终端并运行
for command in commands:
    try:
        # 构建包含source命令和实际命令的字符串
        full_command = f'source ~/dev_ws/install/local_setup.bash; {command}; exec bash'
        # 使用gnome-terminal打开新终端并运行命令
        subprocess.Popen(['gnome-terminal', '--', 'bash', '-c', full_command])
    except FileNotFoundError:
        print("gnome-terminal未找到，请确保你的系统支持该终端模拟器。")
    time.sleep(0.5)

# source ~/dev_ws/install/local_setup.bash
# source ~/dev_ws_final/install/local_setup.bash
#    "ros2 run state_machine run_state_machine"
