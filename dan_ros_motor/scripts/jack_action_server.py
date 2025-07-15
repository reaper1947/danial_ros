#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.duration import Duration
from jack_driver import JackDriver
from next2_msgs.action import JackDaniel
from next2_msgs.msg import JackState
from rclpy.executors import MultiThreadedExecutor
import time
from next2_msgs.msg import RobotMode

class JackControlActionServer(Node):
    def __init__(self):
        super().__init__('jack_control_action_server')

        self.jack = JackDriver()
        self.jack.init_motor()

        self._action_server = ActionServer(
            self,
            JackDaniel,
            'jack_control',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        self._paused = 1
        self._pause_sub = self.create_subscription(
            RobotMode,
            '/robot_mode',
            self.mode_callback,
            10)
        
    def mode_callback(self, msg):
        self._paused = msg.robot_mode
        # self.get_logger().info(f"[Jack Pause] Set pause = {self._paused}")

    def goal_callback(self, goal_request):
        if self._paused != 2:
            self.get_logger().info(f'Not ready to operate')
            return GoalResponse.REJECT    
        self.get_logger().info(f'Received goal request: operation={goal_request.operation}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.jack.stop_motor()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):

        self.get_logger().info('Executing goal...')
        goal = goal_handle.request

        # Setup timeout
        start_time = self.get_clock().now()
        timeout = Duration(seconds=goal.timeout.sec, nanoseconds=goal.timeout.nanosec)

        # Determine direction
        if goal.operation == JackDaniel.Goal.FLOOR:
            target_check = lambda din2, din3: din2 == 1 and din3 == 1  # Hit floor
            status_text = "Moving down"
            speed = abs(goal.speed_rpm)
        elif goal.operation == JackDaniel.Goal.CEILING:
            target_check = lambda din2, din3: din2 == 0 and din3 == 0  # Hit ceiling
            status_text = "Moving up"
            speed = -abs(goal.speed_rpm)
        else:
            goal_handle.abort()
            return JackDaniel.Result(success=False, result_text="Invalid operation", final_state=JackState())


        while rclpy.ok():
            # self.get_logger().info(f"{str(self._paused)}")
            if goal_handle.is_cancel_requested:
                self.jack.stop_motor()
                goal_handle.canceled()
                return JackDaniel.Result(success=False, result_text="Canceled", final_state=self.get_jack_state())
            now = self.get_clock().now()
            if now - start_time > timeout:
                self.jack.stop_motor()
                goal_handle.abort()
                return JackDaniel.Result(
                    success=False,
                    result_text="Timeout reached",
                    final_state=self.get_jack_state())

            if self._paused != 2:
                if self._paused == 5:
                    self.get_logger().info("Emergency Trigger")
                    self.jack.stop_motor()
                    goal_handle.abort()
                    return JackDaniel.Result(success=False, result_text="Aborted Emergency", final_state=self.get_jack_state())
                self.get_logger().info("⚠️ Paused — motor stopped.")
                self.jack.stop_motor()
                time.sleep(0.1)

            else:
                din2, din3, _ = self.jack.read_din_status()
                if target_check(din2, din3):
                    self.jack.stop_motor()
                    goal_handle.succeed()
                    return JackDaniel.Result(
                        success=True,
                        result_text="Reached target",
                        final_state=self.get_jack_state())
               
                # Start moving
                self.jack.set_speed(speed)  
                feedback_msg = JackDaniel.Feedback(
                    status_text=status_text,
                    current_state=self.get_jack_state()
                )
                goal_handle.publish_feedback(feedback_msg)
                # self.get_logger().info('pub')
                time.sleep(0.1)
                # rclpy.spin_once(self, timeout_sec=0.01)
                # self.rate.sleep()

        goal_handle.abort()
        return JackDaniel.Result(success=False, result_text="Aborted externally", final_state=self.get_jack_state())

    def get_jack_state(self):
        din2, din3, _ = self.jack.read_din_status()
        state = JackState()

        # Position logic
        if din2 == 1 and din3 == 1:
            state.jack_position = JackState.JACK_POSITION_LOW
        elif din2 == 0 and din3 == 0:
            state.jack_position = JackState.JACK_POSITION_HEIGHT
        elif din2 == 0 and din3 == 1:
            state.jack_position = JackState.JACK_POSITOIN_BTW_HL
        else:
            state.jack_position = JackState.JACK_POSITION_UNKNOW

        # Sensors
        state.limit_low = din2
        state.limit_hieght = din3

        # You can optionally set motion status here based on motor logic
        # Placeholder:
        jack_last_speed = self.jack.get_speed()
        state.jack_state = JackState.JACK_STATE_MOVING_UP if jack_last_speed < 0 else (
                            JackState.JACK_STATE_MOVING_DOWN if jack_last_speed > 0 else JackState.JACK_STATE_STOP)
        state.actual_velocity = float(jack_last_speed)
        state.jack_status = JackState.JACK_STATUS_READY

        return state

def main(args=None):
    rclpy.init(args=args)
    action_server = JackControlActionServer()

    executor = MultiThreadedExecutor()
    executor.add_node(action_server)

    try:
        executor.spin()
    except KeyboardInterrupt:
        action_server.jack.stop_motor()  # stop jack safely
        action_server.jack._stop_flag = True
        action_server.jack.fault_monitor_thread.join(timeout=1.0)
          # Optional: log shutdown message
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
