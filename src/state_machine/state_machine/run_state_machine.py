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
from state_machine.py_Circle import capture_stable_coordinates, capture_stable_coordinates2
from state_machine.py_homo import capture_move_coordinates
from state_machine.py_icp import capture_stable_transform
from state_machine.state_topic import ArmPosePub, offsetPub, finishSub, chassisMovePub
import tkinter as tk
from tkinter import Toplevel
from PIL import Image, ImageTk, ImageDraw, ImageFont
import threading
import asyncio


class SmachNode(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("启动 demo 节点")

#待机状态，显示启动按钮
class Standby(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['start'])

    def execute(self, userdata):
        global process_demo
        global process_arm
        global order_string
        #启动转盘规划程序
        process_demo = subprocess.Popen(["ros2", "launch", "mybot", "demo.launch.py"],
                                        preexec_fn=self.set_pgid)
        time.sleep(0.1)
        process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_total_trajectory"],
                                        preexec_fn=self.set_pgid)
        
        #记录进程 ID
        with open('/home/fins/dev_ws/pids.txt', 'w') as f:
            f.write(f"{process_arm.pid}\n")
            f.write(f"{process_demo.pid}\n")

        #等待一键启动
        #finishSub()
        order_string = asyncio.run(capture_qr_message_and_print_image("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Qr")) 
        # time.sleep(1.5)
        #order_string = "@213+312"
        print("[TEST]finish camera QR") 
        return 'start'
    
    #用于管理进程的函数
    def set_pgid(self):
        os.setpgid(0, 0)
        return
    



#等待底盘移动至下一个任务点,同时提前启动机械臂规划程序
class ChassisMove(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrive', 'end'],
                             input_keys=['round', 'job_index','job', 'item_num'],
                             output_keys=['round', 'job_index'])

    def execute(self, userdata):
        if userdata.round == 1 and userdata.job_index == 3:
            userdata.round += 1
            userdata.job_index = 1
        else:
            userdata.job_index += 1

        print(f"[WAITING]waiting for chassis to move to [{userdata.job[userdata.job_index]}]...")

        self.armStart(userdata)

        if userdata.round == 2 and userdata.job_index == 4:
            return 'end'
        return 'arrive'
    
    def armStart(self, userdata):
        global process_demo
        global process_arm
        if userdata.job_index == 1 and userdata.round == 2:
            #启动转盘规划程序
            process_demo = subprocess.Popen(["ros2", "launch", "mybot", "demo.launch.py"],
                                            preexec_fn=self.set_pgid)
            time.sleep(0.1)
            process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_total_trajectory_second"],
                                            preexec_fn=self.set_pgid)
        
        # #启动六环规划程序
        # if userdata.job_index == 2 or userdata.job_index == 3:
        #     process_demo = subprocess.Popen(["ros2", "launch", "mybot", "demo.launch.py"],
        #                                     preexec_fn=self.set_pgid)
        #     time.sleep(0.1)
        #     if userdata.job_index == 2:
        #         process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_processing_trajectory"],
        #                                     preexec_fn=self.set_pgid)
        #     elif userdata.job_index == 3 and userdata.round == 1:
        #         process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_storage_trajectory"],
        #                                         preexec_fn=self.set_pgid)
        #     elif userdata.job_index == 3 and userdata.round == 2:
        #         process_arm = subprocess.Popen(["ros2", "run", "mybot", "arm_execute_second_storage_trajectory"],
        #                                         preexec_fn=self.set_pgid)
        #     else:
        #         print("[ERROR] No corresponding robotic arm file found")
        
        #第一次前往转盘有QR识别阻塞进程 除此之外需要sleep一秒防止finishSub出现错误
        if userdata.job_index != 1 or userdata.round == 2:
            time.sleep(1.0)

        #记录进程 ID
        with open('/home/fins/dev_ws/pids.txt', 'w') as f:
            f.write(f"{process_arm.pid}\n")
            f.write(f"{process_demo.pid}\n")
        return
        
    #用于管理进程的函数
    def set_pgid(self):
        os.setpgid(0, 0)
        return
    

#相机视觉纠偏，返回误差值
class CameraOffset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['toOffset'],
                             input_keys=['job_index', 'job', 'round'],
                             output_keys=['pos_error', 'offset'])

    def execute(self, userdata):
        # print(f"************{userdata.job_index}************")
        pos_error = None
        if userdata.job_index == 2:
            print(f"[WAITING] waiting for chassis to move to job [{userdata.job[userdata.job_index]}]")
            #finishSub()
            # pos_error = capture_stable_transform("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Ex_icp")
            # if pos_error == None:
            #     pos_error = [0.0, 0.0, 0.0]
            # # time.sleep(0.1)
            # # pos_error = [10.0, 10.0, 10.0]
            # print("[TEST]finish camera offset")
            # #userdata.pos_error = [pos_error[2]/1000] + [-pos_error[1]/1000] + [pos_error[0]/180*math.pi]
            # userdata.pos_error = [-pos_error[2]/1000] + [-pos_error[1]/1000] + [pos_error[0]/180*math.pi]
            userdata.offset = 1
            print("[OFFSET]need to offset")
        if userdata.job_index == 3:
            print(f"[WAITING] waiting for chassis to move to job [{userdata.job[userdata.job_index]}]")
            #finishSub()
            # if userdata.round == 1:
            #     pos_error = capture_stable_transform("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Ex_icp")
            # else:
            #     pos_error = capture_stable_transform("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Ex_icp2")
            # if pos_error == None:
            #     pos_error = [0.0, 0.0, 0.0]
            # # time.sleep(0.1)
            # # pos_error = [10.0, 10.0, 10.0]
            # print("[TEST]finish camera offset")
            # userdata.pos_error = [(pos_error[1])/1000] + [-pos_error[2]/1000] + [pos_error[0]/180*math.pi]
            userdata.offset = 1
            print("[OFFSET]need to offset")
        return 'toOffset'
    

#将误差值传入下位机，里程计纠偏
class ChassisOffset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['finishOffset'],
                             input_keys=['offset', 'pos_error'],
                             output_keys=['offset'])

    def execute(self, userdata):
        if userdata.offset == 1:
            #将pos_error传给底盘纠偏
            #offsetPub(userdata.pos_error)
            # print("[WAITING]waiting for chassis to offset...")
            # time.sleep(2.5)
            userdata.offset -= 1 
            print("[OFFSET]finish offset")
        return 'finishOffset'
    

#相机开始识别，返回三个目标位置
class Camera(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['beginToWork'],
                             input_keys=['job_index', 'order', 'round', 'item_pos', 'item_num', 'job'],
                             output_keys=['order', 'item_pos'])

    def execute(self, userdata):
        global thread
        global order_string

        if userdata.job_index == 1:
            #相机识别二维码
            if userdata.round == 1 and userdata.item_num == 0:
                order_string = order_string[1:]    
                order1, order2 = order_string.split('+')
                userdata.order = [[int(num) for num in order1], [int(num) for num in order2]]

                #显示图片 
                image = self.create_image_with_text(order_string)
                display_number = 0  # 指定显示器的编号，从0开始
                thread = threading.Thread(target=self.show_image_on_display, args=(image, display_number))
                thread.start()
                print(f"[JOB]the job order in this turn is {userdata.order}")           

            #相机识别转盘处
            userdata.item_pos = []
            order_current = userdata.order[userdata.round-1]
            all_pos = None

            if userdata.item_num == 0:
                #if userdata.round == 1:
                    #time.sleep(1.0)
                print(f"[WAITING] waiting for chassis to move to job [{userdata.job[userdata.job_index]}]")
                #finishSub()              #移至相机调用程序内
                with open('/home/fins/dev_ws/item_pos.txt', 'a') as f:
                    f.write("---------a new try in move------------\n")
                    #time.sleep(0.5)
            while all_pos == None:
                all_pos = asyncio.run(capture_move_coordinates(order_current[userdata.item_num], order_current[:userdata.item_num], "/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo_P2"))
                # time.sleep(2.0)
                # all_pos = [(-150.0, 200.0, 0.0), (0.0, 200.0, 0.0), (150.0, 200.0, 0.0)]
            print(f"[TEST]finish camera capture in [{userdata.job[userdata.job_index]}]") 

            for color in order_current:
                pos = all_pos[color-1]
                pos = [num/1000 for num in pos]                                        #将单位由毫米转化为米
                # if pos[1] >= 0.20:                  #手动修正
                # #     if pos[0] > 0.03:
                # #         pos[1] += 0.01   
                # #         pos[0] += 0.01            
                # #     else:
                # # #         pos[1] += 0.015
                #     pass
                # else:
                #     pos[1] -= 0.003
                userdata.item_pos.append(pos)
            print(f"[JOB]the job is {order_current}, the item pos is {userdata.item_pos}")
            with open('/home/fins/dev_ws/item_pos.txt', 'a') as f:
                f.write(f"{userdata.item_pos}\n")
                f.write(f"The target item is{userdata.item_pos[userdata.item_num]}\n")
        
        #相机识别六环物料位置
        else:
            userdata.item_pos = []
            order_current = userdata.order[userdata.round-1]
            with open('/home/fins/dev_ws/item_pos.txt', 'a') as f:
                    f.write("---------a new try in circle------------\n")
            if userdata.job_index == 3 and userdata.round == 2:
                all_pos = asyncio.run(capture_stable_coordinates2("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo2",3 ,2 ,60)) 
                # time.sleep(0.1)
                # all_pos = [(-150.0, 200.0, 0.0), (0.0, 200.0, 0.0), (150.0, 200.0, 0.0)]
            else:
                all_pos = asyncio.run(capture_stable_coordinates("/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo")) 
                # time.sleep(0.1)
                # all_pos = [(-150.0, 200.0, 0.0), (0.0, 200.0, 0.0), (150.0, 200.0, 0.0)]
            print(f"[TEST]finish camera capture in [{userdata.job[userdata.job_index]}]")

            for color in order_current:
                pos = all_pos[color-1]
                pos = [num/1000 for num in pos]                                        #将单位由毫米转化为米
                userdata.item_pos.append(pos)
            print(f"[JOB]the job is {order_current}, the item pos is {userdata.item_pos}")
            with open('/home/fins/dev_ws/item_pos.txt', 'a') as f:
                f.write(f"{userdata.item_pos}\n")
        return 'beginToWork'

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
                             output_keys=['item_num','order'])

    def execute(self, userdata):
        global process_demo

        if userdata.job_index == 1:
            result = self.armCirclePub(userdata)
            if result == 1:
                 return "toCamera"
        else:
            self.armPub(userdata)
        
        if userdata.job_index == 3:
            #关闭仿真程序
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
        #global process_arm

        #发送机械臂目标位置
        if userdata.item_num < 3:
            node_pose_pub = ArmPosePub(userdata.item_pos)
            rclpy.spin_once(node_pose_pub)
            node_pose_pub.destroy_node()
            # order = userdata.order[userdata.round-1]
            # userdata.order[userdata.round-1] = order[1:] + [order[0]]

            #等待动作完成
            print(f"[WAITING] waiting for arm to finish in job [{userdata.job[userdata.job_index]}]. The target item is [{userdata.order[userdata.round-1][userdata.item_num]}] now")
            #finishSub() #移至相机识别内

            userdata.item_num += 1
            if userdata.item_num != 3:
                return 1

        #完成任务后终止程序
        finishSub()
        userdata.item_num = 0
        # try:
        #     # 请求进程退出
        #     os.killpg(os.getpgid(process_arm.pid), signal.SIGTERM)
        #     print("[PROCESS]try to terminate process_arm")
        #     # 等待一段时间，让进程有机会清理资源
        #     for _ in range(5): 
        #         retcode = process_arm.poll()
        #         if retcode is not None:
        #             print("Process has been terminated successfully.")
        #             break
        #         time.sleep(0.1)
        #     else:
        #         # 如果进程在5秒后仍然未终止，强制杀死它
        #         os.killpg(os.getpgid(process_arm.pid), signal.SIGKILL)
        #         print("Process was not terminated gracefully, killed forcefully.")
        # except Exception as e:
        #     print(f"Error terminating process: {e}")
        return 0

    def armPub(self, userdata):
        global process_arm
        #启动机械臂规划程序(移至相机状态)

        #发送机械臂目标位置
        print(userdata.item_pos)
        node_pose_pub = ArmPosePub(userdata.item_pos)
        rclpy.spin_once(node_pose_pub)
        node_pose_pub.destroy_node()

        #等待动作完成后终止程序
        print(f"[WAITING] waiting for arm to finish in job [{userdata.job[userdata.job_index]}]")
        time.sleep(3.0)
        finishSub()
        if userdata.job_index == 3:
            try:
                # 请求进程退出
                os.killpg(os.getpgid(process_arm.pid), signal.SIGTERM)
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
    global process_demo
    global process_arm
    global thread

    sm = smach.StateMachine(outcomes=['End', 'Error'])

    sm.userdata.order = [(1,2,3),(1,2,3)]
    #sm.userdata.order = None
    sm.userdata.round = 1
    sm.userdata.job_index = 0                 #0：启动区 1：原料 2：粗加工 3：储存 4：停止 当前处于工作状态
    #sm.userdata.job_index = 2                 #测试用
    sm.userdata.job = ["启动区","原料","粗加工","储存","停止"]
    
    sm.userdata.item_pos = []
    sm.userdata.item_num = 0     #用于转盘处标记物料 代表已完成物料数目
    sm.userdata.offset = 0
    sm.userdata.pos_error = [0.0, 0.0, 0.0]

    

    with sm:  
        # smach.StateMachine.add('ChassisMove', ChassisMove(), 
        #                        transitions={'arrive':'CameraOffset', 'end':'End'},
        #                        remapping={'round':'round', 'job_index':'job_index', 'job':'job',
        #                                   'item_num':'item_num'})
           
        smach.StateMachine.add('Standby', Standby(), 
                               transitions={'start':'ChassisMove'})
        
        smach.StateMachine.add('ChassisMove', ChassisMove(), 
                               transitions={'arrive':'CameraOffset', 'end':'End'},
                               remapping={'round':'round', 'job_index':'job_index', 'job':'job',
                                          'item_num':'item_num'})
        
        smach.StateMachine.add('CameraOffset', CameraOffset(), 
                               transitions={'toOffset':'ChassisOffset'},
                               remapping={'job_index':'job_index', 'round':'round',
                                          'pos_error':'pos_error', 'offset':'offset'})
        
        smach.StateMachine.add('ChassisOffset', ChassisOffset(), 
                               transitions={'finishOffset':'Camera'},
                               #transitions={'finishOffset':'End'},
                               remapping={'offset':'offset', 'pos_error':'pos_error', 'job':'job'})
        
        smach.StateMachine.add('Camera', Camera(), 
                               transitions={'beginToWork':'ArmMove'},
                               remapping={'job_index':'job_index', 'order':'order', 'round':'round',
                                          'item_pos':'item_pos', 'item_num':'item_num', 'job':'job'})
        
        smach.StateMachine.add('ArmMove', ArmMove(), 
                               transitions={'toCamera':'Camera', 'nextWork':'ChassisMove'},
                               remapping={'job_index':'job_index','item_pos':'item_pos','job':'job', 
                                            'item_num':'item_num','order':'order','round':'round'})
        


    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()     #状态机可视化
    outcome = sm.execute()
    rclpy.spin(node)
    sis.stop()
    node.destroy_node()

if __name__ == '__main__':
    main()