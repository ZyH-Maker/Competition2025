import rclpy
from rclpy.node import Node
import smach
import smach_ros
import os
import subprocess
import signal
import time
import math
from state_machine.py_Qr import capture_qr_message_and_print_image
from state_machine.py_Circle import capture_stable_coordinates
from state_machine.py_homo import capture_move_coordinates
from state_machine.py_icp import capture_stable_transform
from state_machine.state_topic import ArmPosePub, offsetPub, FinishSub, chassisLock, chassisMovePub
import tkinter as tk
from tkinter import Toplevel
from PIL import Image, ImageTk, ImageDraw, ImageFont
import threading
from datetime import datetime


class SmachNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("启动 demo 节点")

#待机状态，显示启动按钮
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        #屏幕显示启动按钮
        #if 外部点击启动按钮
        return 'start'


#启动初始化，底盘锁定, 机械臂归零
class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['move'])

    def execute(self, userdata):
        '''
        #底盘锁定, 机械臂归零
        print("[WAITING]waiting foe ARM to finish initing...")
        finishSub()'''

        #添加时间戳 后续删除
        file_name = 'time_check.txt'
        with open(file_name, 'a', encoding='utf-8') as file:
            file.write("—————————————————一次新的尝试————————————————————\n")
        print(f'文本和时间戳已保存到 {file_name}')
        #添加时间戳 后续删除
        return 'move'


#底盘移动至下一个任务点,底盘锁定
class ChassisMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrive', 'end'],
                             input_keys=['round', 'job_index', 'path','job'],
                             output_keys=['round', 'job_index'])

    def execute(self, userdata):
        #底盘解锁
        #chassisLock(0.0)
        #time.sleep(1)
        #rclpy.spin_once(finish_node)
        global finish_node
        #旧版本 加上底盘纠偏后可改回
        # if userdata.round == 1 and userdata.job_index == 4:
        #     move_path = userdata.path[5]
        #     userdata.round += 1
        #     userdata.job_index = 2
        # else:
        #     move_path = userdata.path[userdata.job_index] 
        #     userdata.job_index += 1

        move_path = userdata.path[userdata.job_index+(userdata.round-1)*3]

        if userdata.round == 1 and userdata.job_index == 4:
            userdata.round += 1
            userdata.job_index = 2
        else:
            userdata.job_index += 1

        
        print(f"[MOVE]the move path to [{userdata.job[userdata.job_index]}] is {move_path}")
        index = 0
        while index < len(move_path):
            chassisMovePub(move_path[index])
            print(f"[WAITING]waiting for chassis to move to [{move_path[index]}]...")
            #time.sleep(10)
            rclpy.spin_once(finish_node)
            index += 1
        #底盘锁定
        #chassisLock(1.0)
        #time.sleep(1)
        #rclpy.spin_once(finish_node)

        if userdata.round == 2 and userdata.job_index == 5:
            return 'end'
        return 'arrive'
    

#相机视觉纠偏，返回误差值
class CameraOffset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toOffset'],
                             input_keys=['job_index'],
                             output_keys=['pos_error', 'offset'])

    def execute(self, userdata):
        # print(f"************{userdata.job_index}************")
        # if userdata.job_index == 3:
        #     pos_error = capture_stable_transform("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Ex_icp")
        #     userdata.pos_error = [pos_error[1]/1000] + [-pos_error[2]/1000] + [pos_error[0]/180*math.pi]
        #     userdata.offset = 1
        #     print("[OFFSET]need to offset")
        # if userdata.job_index == 4:
        #     pos_error = capture_stable_transform("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Ex_icp")
        #     userdata.pos_error = [(pos_error[1])/1000] + [-pos_error[2]/1000] + [pos_error[0]/180*math.pi]
        #     userdata.offset = 1
        #     print("[OFFSET]need to offset")
        return 'toOffset'
    

#将误差值传入下位机，里程计纠偏
class ChassisOffset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finishOffset'],
                             input_keys=['offset', 'pos_error'],
                             output_keys=['offset'])

    def execute(self, userdata):
        # if userdata.offset == 1:
        #     #将pos_error传给底盘纠偏
        #     offsetPub(userdata.pos_error)
        #     print("[WAITING]waiting for chassis to offset...")
        #     rclpy.spin_once(finish_node)
        #     userdata.offset -= 1 
        #     print("[OFFSET]finish offset")
        return 'finishOffset'
    

#相机开始识别，返回三个目标位置
class Camera(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['beginToWork', 'move'],
                             input_keys=['job_index', 'order', 'round', 'item_pos', 'item_num'],
                             output_keys=['order', 'item_pos'])

    def execute(self, userdata):
        global process_demo
        global process_arm
        global thread

        #相机识别二维码
        if userdata.job_index == 1:
            order_string = "@123+312"
            #order_string = capture_qr_message_and_print_image("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Qr")  
            order_string = order_string[1:]    
            order1, order2 = order_string.split('+')
            userdata.order = [[int(num) for num in order1], [int(num) for num in order2]]

            #显示图片 
            image = self.create_image_with_text(order_string)
            display_number = 0  # 指定显示器的编号，从0开始
            thread = threading.Thread(target=self.show_image_on_display, args=(image, display_number))
            thread.start()

            print(f"[JOB]the job order in this turn is {userdata.order}")
            return 'move'
        
        #相机识别转盘物料位置
        elif userdata.job_index == 2:
            #提前启动仿真节省时间
            if userdata.item_num == 0:
                process_demo = subprocess.Popen(["ros2", "launch", "mybot", "demo.launch.py"],
                                                preexec_fn=self.set_pgid)
                process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_circle_trajectory"],
                                                preexec_fn=self.set_pgid)
                time.sleep(3.5)

            userdata.item_pos = []
            order_current = userdata.order[userdata.round-1]
            all_pos = None
            #all_pos = [(-150.0, 200.0, 0.0), (0.0, 200.0, 0.0), (150.0, 200.0, 0.0)]

            # #添加时间戳 后续删除
            # file_name = 'time_check.txt'
            # current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            # with open(file_name, 'a', encoding='utf-8') as file:
            #     file.write("start_camera开始调用相机程序函数\n")
            #     file.write(f'当前时间戳: {current_timestamp}\n')
            # print(f'文本和时间戳已保存到 {file_name}')
            # #添加时间戳 后续删除
            while all_pos == None:
                all_pos = capture_move_coordinates(order_current[userdata.item_num],"/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo_P") 
            # # 指定文件路径
            # file_path = "info.txt"
            # # 将列表写入文件
            # with open(file_path, 'w') as file:
            #     for item in all_pos:
            #         file.write(f"{item}\n")  # 每个元素写入一行
            #     file.write("\n")
            # #添加时间戳 后续删除
            # file_name = 'time_check.txt'
            # current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            # with open(file_name, 'a', encoding='utf-8') as file:
            #     file.write("camera_return相机程序函数调用结束\n")
            #     file.write(f'当前时间戳: {current_timestamp}\n')
            # print(f'文本和时间戳已保存到 {file_name}')
            # #添加时间戳 后续删除

            for color in order_current:
                pos = all_pos[color-1]
                pos = [num/1000 for num in pos]                                        #将单位由毫米转化为米
                pos[1] += 0.015               #手动修正
                userdata.item_pos.append(pos)
            print(f"[JOB]the job is {order_current}, the item pos is {userdata.item_pos}")
            return 'beginToWork'
        
        #相机识别六环物料位置
        else:
            #提前启动仿真 节省时间
            if userdata.item_num == 0:
                process_demo = subprocess.Popen(["ros2", "launch", "mybot", "demo.launch.py"],
                                                preexec_fn=self.set_pgid)
            if userdata.job_index == 3:
                process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_processing_trajectory"],
                                            preexec_fn=self.set_pgid)
                time.sleep(3.5)
            elif userdata.job_index == 4 and userdata.round == 1:
                process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_storage_trajectory"],
                                                preexec_fn=self.set_pgid)
                time.sleep(3.5)
            elif userdata.job_index == 4 and userdata.round == 2:
                process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_second_storage_trajectory"],
                                                preexec_fn=self.set_pgid)
                time.sleep(3.5)
            else:
                print("[ERROR] No corresponding robotic arm file found")

            userdata.item_pos = []
            order_current = userdata.order[userdata.round-1]
            #all_pos = [(-150.0, 200.0, 0.0), (0.0, 200.0, 0.0), (150.0, 200.0, 0.0)]
            all_pos = capture_stable_coordinates("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo") 

            # # 指定文件路径
            # file_path = "colorinfo.txt"
            # # 将列表写入文件
            # with open(file_path, 'a') as file:
            #     file.write(f"---------{userdata.job_index}-------------\n")  # 每个元素写入一行
            #     for item in all_pos:
            #         file.write(f"{item}\n")  # 每个元素写入一行
            #     file.write("\n")

            for color in order_current:
                pos = all_pos[color-1]
                pos = [num/1000 for num in pos]                                        #将单位由毫米转化为米
                userdata.item_pos.append(pos)
            print(f"[JOB]the job is {order_current}, the item pos is {userdata.item_pos}")
            return 'beginToWork'
        
    #用于管理进程的函数
    def set_pgid(self):
        os.setpgid(0, 0)
        return

    #用于显示任务内容
    def show_image_on_display(self, image, screen_index):
        window = Toplevel()
        window.geometry(f"1920x480+{screen_index * 800}+0")
        photo = ImageTk.PhotoImage(image)
        label = tk.Label(window, image=photo)
        label.image = photo  
        label.pack()

        window.mainloop()

    #用于创建图像
    def create_image_with_text(self, text, image_size=(1920, 480), background_color=(255, 255, 255), text_color=(0, 0, 0), font_size=256):
        # 创建一个新的图像
        image = Image.new("RGB", image_size, background_color)
        draw = ImageDraw.Draw(image)
        # 加载字体
        font_path = "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"
        try:
            font = ImageFont.truetype(font_path, font_size)  # 使用系统字体
        except IOError:
            print("NO TARGET TEXT")
            font = ImageFont.load_default()  # 如果找不到指定字体，使用默认字体
        # 计算文本的大小
        text_width, text_height = draw.textsize(text, font=font)
        # 计算文本的位置，使其居中
        text_x = (image_size[0] - text_width) // 2
        text_y = (image_size[1] - text_height) // 2
        # 在图像上绘制文本
        draw.text((text_x, text_y), text, fill=text_color, font=font)
        # 保存图像
        return image


#将目标位置传给上位机，机械臂开始工作
class ArmMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toCamera', 'nextWork'],
                             input_keys=['job_index', 'item_pos', 'job', 'item_num', 'order', 'round'],
                             output_keys=['direction','item_num','order'])

    def execute(self, userdata):
        #启动机械臂仿真程序（移至相机）
        global process_demo

        if userdata.job_index == 2:
            result = self.armCirclePub(userdata)
            if result == 1:
                 return "toCamera"
        else:
            self.armPub(userdata)
        
        # #关闭仿真程序
        try:
            # 请求进程退出
            os.killpg(os.getpgid(process_demo.pid), signal.SIGTERM)
            print("[PROCESS]try to terminate process_demo")

            # 等待一段时间让进程清理资源
            for _ in range(5):
                retcode = process_demo.poll()
                if retcode is not None:
                    print("Process has been terminated successfully.")
                    break
                time.sleep(0.1)
            else:
                # 如果进程在5秒后仍然未终止，强制杀死它
                os.killpg(os.getpgid(process_demo.pid), signal.SIGKILL)
                print("Process was not terminated gracefully, killed forcefully.")

        except Exception as e:
            print(f"Error terminating process: {e}")

        return "nextWork"
    
    #用于管理进程的函数
    def set_pgid(self):
        os.setpgid(0, 0)
        return
    
    def armCirclePub(self, userdata):
        #启动机械臂规划程序
        global process_arm
        global finish_node

        #发送机械臂目标位置
        if userdata.item_num < 3:
            node_pose_pub = ArmPosePub(userdata.item_pos)
            rclpy.spin_once(node_pose_pub)
            #添加时间戳 后续删除
            file_name = 'time_check.txt'
            current_timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            with open(file_name, 'a', encoding='utf-8') as file:
                file.write("pos_pub Ros发送节点消息\n")
                file.write(f'当前时间戳: {current_timestamp}\n')
            print(f'文本和时间戳已保存到 {file_name}')
            #添加时间戳 后续删除
            node_pose_pub.destroy_node()
            # order = userdata.order[userdata.round-1]
            # userdata.order[userdata.round-1] = order[1:] + [order[0]]

            #等待动作完成
            print(f"[WAITING] waiting for arm to finish in job [{userdata.job[userdata.job_index]}]. The target item is [{userdata.order[userdata.round-1][userdata.item_num]}] now")
            rclpy.spin_once(finish_node)

            userdata.item_num += 1
            if userdata.item_num != 3:
                return 1

        #完成任务后终止程序
        userdata.item_num = 0
        try:
            # 请求进程退出
            os.killpg(os.getpgid(process_demo.pid), signal.SIGTERM)
            print("[PROCESS]try to terminate process_arm")
            time.sleep(1)

            # 等待一段时间，让进程有机会清理资源
            for _ in range(5): 
                retcode = process_arm.poll()
                if retcode is not None:
                    print("Process has been terminated successfully.")
                    break
                time.sleep(0.1)
            else:
                # 如果进程在5秒后仍然未终止，强制杀死它
                os.killpg(os.getpgid(process_arm.pid), signal.SIGKILL)
                print("Process was not terminated gracefully, killed forcefully.")

        except Exception as e:
            print(f"Error terminating process: {e}")
        return 0

    def armPub(self, userdata):
        #-------------暂时使用 后续删除----------
        # if userdata.job_index == 2:
        #     process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_circle"],
        #                                     preexec_fn=self.set_pgid)
        #-------------暂时使用 后续删除----------
        global process_arm
        global finish_node
        #启动机械臂规划程序(移至相机状态)

        #发送机械臂目标位置
        print(userdata.item_pos)
        node_pose_pub = ArmPosePub(userdata.item_pos)
        rclpy.spin_once(node_pose_pub)
        node_pose_pub.destroy_node()

        #等待动作完成后终止程序
        print(f"[WAITING] waiting for arm to finish in job [{userdata.job[userdata.job_index]}]")
        rclpy.spin_once(finish_node)
        try:
            # 请求进程退出
            os.killpg(os.getpgid(process_demo.pid), signal.SIGTERM)
            time.sleep(1)

            # 等待一段时间，让进程有机会清理资源
            for _ in range(5):  # 等待最多5秒
                retcode = process_arm.poll()
                if retcode is not None:
                    print("Process has been terminated successfully.")
                    break
                time.sleep(0.1)
            else:
                # 如果进程在5秒后仍然未终止，强制杀死它
                os.killpg(os.getpgid(process_arm.pid), signal.SIGKILL)
                print("Process was not terminated gracefully, killed forcefully.")

        except Exception as e:
            print(f"Error terminating process: {e}")
        return    


# main
def main(args=None):
    rclpy.init(args=args)
    node = SmachNode("smach_state_machine")
    global finish_node 
    global process_demo
    global process_arm
    global thread
    finish_node = FinishSub("finish_node")

    sm = smach.StateMachine(outcomes=['End', 'Error'])

    sm.userdata.order = [(1,2,3),(1,2,3)]
    #sm.userdata.order = None
    sm.userdata.round = 1
    sm.userdata.job_index = 0                 #0：启动区 1：二维码 2：原料 3：粗加工 4：储存 5：停止 当前处于工作状态
    #sm.userdata.job_index = 4 #测试用
    sm.userdata.job = ["启动区","二维码","原料","粗加工","储存","停止"]
    sm.userdata.path = [[[0.0, -0.155, 0.0], [-0.65, -0.155, 0.0]],      #由工作0到工作1的路径 到二维码
                        [[-1.45, -0.115, 0.0]],   #转盘
                        # [[-0.0, 0.0, 0.0],[-1.06, -0.155, 173/180*math.pi], [-1.23, -1.83, 173/180*math.pi]], #第一次加工区 测试用
                        #[[0.0, -0.155, 0.0]],      #由工作0到工作1的路径 到二维码 测试
                        [[-1.06, -0.155, 0.0],[-1.1, -1.83, 0.0], [-1.1, -1.83, 173/180*math.pi]], #第一次加工区 
                        [[-1.9, -1.7, 173/180*math.pi], [-1.9, -1.7, 83/180*math.pi], [-1.9, -0.96, 83/180*math.pi]], #第一次暂存区
                        [[-1.95, -0.05, 86/180*math.pi], [-1.95, -0.05, 0.0], [-1.45, -0.06, 0.0]], #第二次转盘
                        [[-1.06, -0.06, 0.0], [-1.26, -1.79, 0.0], [-1.26, -1.79, 173/180*math.pi]], #第二次加工区
                        [[-2.04, -1.79, 173/180*math.pi], [-2.0, -1.78, 83/180*math.pi], [-2.0, -0.96, 83/180*math.pi]], #第二次暂存区
                        [[-1.95, -0.05, 86/180*math.pi], [-1.95, -0.05, 0.0], [-0.1, -0.05, 0.0], [-0.1, 0.0, 0.01]] #回起点
                        ]
    
    sm.userdata.item_pos = []
    sm.userdata.item_num = 0     #用于转盘处标记物料 代表已完成物料数目
    sm.userdata.offset = 0
    #sm.userdata.offset = 1 #测试
    sm.userdata.pos_error = [0.0, 0.0, 0.0]

    

    with sm:
        # smach.StateMachine.add('Camera', Camera(), 
        #                        transitions={'beginToWork':'ArmMove', 'move':'ChassisMove'},
        #                        remapping={'job_index':'job_index', 'order':'order', 'round':'round',
        #                                   'item_pos':'item_pos', 'item_num':'item_num'})

        # #测试底盘纠偏用 
        # smach.StateMachine.add('CameraOffset', CameraOffset(), 
        #                        transitions={'toOffset':'ChassisOffset'},
        #                        remapping={'job_index':'job_index',
        #                                   'pos_error':'pos_error', 'offset':'offset'})
        
        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'start':'Init'})
        
        smach.StateMachine.add('Init', Init(), 
                               transitions={'move':'ChassisMove'})
        
        smach.StateMachine.add('ChassisMove', ChassisMove(), 
                               transitions={'arrive':'CameraOffset', 'end':'End'},
                               remapping={'round':'round', 'job_index':'job_index', 'path':'path', 'job':'job'})
        
        smach.StateMachine.add('CameraOffset', CameraOffset(), 
                               transitions={'toOffset':'ChassisOffset'},
                               remapping={'job_index':'job_index',
                                          'pos_error':'pos_error', 'offset':'offset'})
        
        smach.StateMachine.add('ChassisOffset', ChassisOffset(), 
                               transitions={'finishOffset':'Camera'},
                               #transitions={'finishOffset':'End'},
                               remapping={'offset':'offset', 'pos_error':'pos_error'})
        
        smach.StateMachine.add('Camera', Camera(), 
                               transitions={'beginToWork':'ArmMove', 'move':'ChassisMove'},
                               remapping={'job_index':'job_index', 'order':'order', 'round':'round',
                                          'item_pos':'item_pos', 'item_num':'item_num'})
        
        smach.StateMachine.add('ArmMove', ArmMove(), 
                               transitions={'toCamera':'Camera', 'nextWork':'ChassisMove'},
                               remapping={'job_index':'job_index','item_pos':'item_pos','job':'job', 
                                            'item_num':'item_num','order':'order','round':'round'})
        


    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()     #状态机可视化
    outcome = sm.execute()
    rclpy.spin(node)
    sis.stop()
    finish_node.destroy_node()
    node.destroy_node()

if __name__ == '__main__':
    main()
