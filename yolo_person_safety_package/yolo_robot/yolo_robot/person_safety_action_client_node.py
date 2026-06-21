#!/usr/bin/env python3

from typing import Optional

import rclpy

from action_msgs.msg import GoalStatus
from logistic_msg.action import MoveRobot
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool, String


GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}


class PersonSafetyActionClient(Node):
    """
    Bridge between person detection and the MoveRobot action server.

    Behavior:
    - /person_in_path == False: send action="start" when no goal is active.
    - /person_in_path == True: cancel the active MoveRobot goal.
    - After the path becomes clear: send a new start goal.
    """

    def __init__(self) -> None:
        super().__init__("person_safety_action_client")

        self.declare_parameter(
            "person_topic",
            "/person_in_path",
        )
        self.declare_parameter(
            "vision_topic",
            "/vision_command",
        )
        self.declare_parameter(
            "action_name",
            "/start_action",
        )
        self.declare_parameter(
            "start_action_text",
            "start",
        )
        self.declare_parameter(
            "blocking_text",
            "PERSON_BLOCKING",
        )
        self.declare_parameter(
            "clear_text",
            "PATH_CLEAR",
        )
        self.declare_parameter(
            "auto_start_when_clear",
            True,
        )
        self.declare_parameter(
            "resume_when_clear",
            True,
        )

        self.person_topic = str(
            self.get_parameter("person_topic").value
        )
        self.vision_topic = str(
            self.get_parameter("vision_topic").value
        )
        self.action_name = str(
            self.get_parameter("action_name").value
        )
        self.start_action_text = str(
            self.get_parameter("start_action_text").value
        ).strip()
        self.blocking_text = str(
            self.get_parameter("blocking_text").value
        ).strip().upper()
        self.clear_text = str(
            self.get_parameter("clear_text").value
        ).strip().upper()
        self.auto_start_when_clear = bool(
            self.get_parameter("auto_start_when_clear").value
        )
        self.resume_when_clear = bool(
            self.get_parameter("resume_when_clear").value
        )

        # Fail-safe default: blocked until first safety state arrives.
        self.person_blocking = True
        self.path_state_received = False

        self.goal_request_in_progress = False
        self.goal_in_progress = False
        self.cancel_in_progress = False

        self.cancel_when_accepted = False
        self.resume_pending = False
        self.start_pending = False

        self.goal_handle = None
        self.server_announced = False

        self.action_client = ActionClient(
            self,
            MoveRobot,
            self.action_name,
        )

        self.person_subscription = self.create_subscription(
            Bool,
            self.person_topic,
            self.person_state_callback,
            10,
        )

        self.vision_subscription = self.create_subscription(
            String,
            self.vision_topic,
            self.vision_command_callback,
            10,
        )

        self.server_timer = self.create_timer(
            1.0,
            self.check_action_server,
        )

        self.get_logger().info(
            "Person Safety Action Client started"
        )
        self.get_logger().info(
            f"Person topic: {self.person_topic}"
        )
        self.get_logger().info(
            f"Vision topic: {self.vision_topic}"
        )
        self.get_logger().info(
            f"Action server: {self.action_name}"
        )

    def check_action_server(self) -> None:
        if self.action_client.server_is_ready():
            if not self.server_announced:
                self.get_logger().info(
                    f"MoveRobot action server "
                    f"{self.action_name} is ready."
                )
                self.server_announced = True

            if (
                self.start_pending
                and self.path_state_received
                and not self.person_blocking
                and not self.goal_request_in_progress
                and not self.goal_in_progress
                and not self.cancel_in_progress
            ):
                self.start_pending = False
                self.send_start_goal()

        else:
            self.server_announced = False
            self.get_logger().warn(
                f"Waiting for {self.action_name} action server...",
                throttle_duration_sec=5.0,
            )

    def person_state_callback(self, msg: Bool) -> None:
        self.process_path_state(
            blocked=bool(msg.data),
            source=self.person_topic,
        )

    def vision_command_callback(self, msg: String) -> None:
        command = msg.data.strip().upper()

        if command == self.blocking_text:
            self.process_path_state(
                blocked=True,
                source=self.vision_topic,
            )

        elif command == self.clear_text:
            self.process_path_state(
                blocked=False,
                source=self.vision_topic,
            )

    def process_path_state(
        self,
        blocked: bool,
        source: str
    ) -> None:
        if (
            self.path_state_received
            and blocked == self.person_blocking
        ):
            return

        previous_state = self.person_blocking

        self.path_state_received = True
        self.person_blocking = blocked

        if blocked:
            self.get_logger().warn(
                f"PERSON BLOCKING received from {source}"
            )
            self.handle_person_blocking()

        else:
            self.get_logger().info(
                f"PATH CLEAR received from {source}"
            )
            self.handle_path_clear(previous_state)

    def handle_person_blocking(self) -> None:
        self.start_pending = False

        if self.resume_when_clear:
            self.resume_pending = True

        if (
            self.goal_request_in_progress
            and self.goal_handle is None
        ):
            self.cancel_when_accepted = True

            self.get_logger().warn(
                "Goal request is still being processed. "
                "It will be canceled immediately if accepted."
            )
            return

        if (
            self.goal_in_progress
            and self.goal_handle is not None
        ):
            self.cancel_active_goal()
            return

        self.get_logger().warn(
            "No active MoveRobot goal is available to cancel."
        )

    def handle_path_clear(
        self,
        previous_state: bool
    ) -> None:
        self.cancel_when_accepted = False

        if self.cancel_in_progress:
            if self.resume_when_clear:
                self.resume_pending = True

            self.get_logger().info(
                "Waiting for cancellation to finish before restarting."
            )
            return

        if self.goal_request_in_progress:
            self.get_logger().info(
                "A start goal request is already being processed."
            )
            return

        if self.goal_in_progress:
            self.get_logger().info(
                "Robot action is already running."
            )
            return

        should_start = (
            self.auto_start_when_clear
            or (
                previous_state
                and self.resume_when_clear
            )
        )

        if not should_start:
            self.get_logger().info(
                "Path is clear, but automatic start is disabled."
            )
            return

        self.resume_pending = False
        self.send_start_goal()

    def send_start_goal(self) -> None:
        if not self.path_state_received:
            self.get_logger().warn(
                "No person-safety state has been received yet."
            )
            return

        if self.person_blocking:
            self.get_logger().warn(
                "Cannot start while a person is blocking the path."
            )
            return

        if (
            self.goal_request_in_progress
            or self.goal_in_progress
            or self.cancel_in_progress
        ):
            self.get_logger().warn(
                "Another goal operation is currently active."
            )
            return

        if not self.action_client.server_is_ready():
            self.start_pending = True
            self.get_logger().error(
                f"{self.action_name} is not available. "
                "The start request will remain pending."
            )
            return

        goal_msg = MoveRobot.Goal()
        goal_msg.action = self.start_action_text

        self.goal_request_in_progress = True
        self.start_pending = False

        self.get_logger().info(
            f"Sending MoveRobot goal: "
            f"action='{goal_msg.action}' "
            f"to {self.action_name}"
        )

        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future) -> None:
        self.goal_request_in_progress = False

        try:
            goal_handle = future.result()

        except Exception as error:
            self.goal_handle = None
            self.goal_in_progress = False

            self.get_logger().error(
                f"Failed to send MoveRobot goal: {error}"
            )
            return

        if (
            goal_handle is None
            or not goal_handle.accepted
        ):
            self.goal_handle = None
            self.goal_in_progress = False

            self.get_logger().error(
                "MoveRobot action server rejected the goal."
            )
            return

        self.goal_handle = goal_handle
        self.goal_in_progress = True

        self.get_logger().info(
            "MoveRobot action server accepted the start goal."
        )

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            self.result_callback
        )

        if (
            self.person_blocking
            or self.cancel_when_accepted
        ):
            self.cancel_when_accepted = False

            self.get_logger().warn(
                "A person is blocking the path. "
                "Canceling the newly accepted goal."
            )

            self.cancel_active_goal()

    def feedback_callback(self, feedback_msg) -> None:
        feedback = feedback_msg.feedback

        self.get_logger().info(
            "Robot position: "
            f"x={feedback.pose_x:.3f}, "
            f"y={feedback.pose_y:.3f}"
        )

    def cancel_active_goal(self) -> None:
        if self.cancel_in_progress:
            self.get_logger().warn(
                "A cancellation request is already in progress."
            )
            return

        if self.goal_handle is None:
            if self.goal_request_in_progress:
                self.cancel_when_accepted = True

                self.get_logger().warn(
                    "Goal handle is not available yet. "
                    "The goal will be canceled after acceptance."
                )
            else:
                self.get_logger().warn(
                    "No active goal handle is available."
                )
            return

        self.cancel_in_progress = True

        self.get_logger().warn(
            f"Sending cancellation request to {self.action_name}..."
        )

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            self.cancel_response_callback
        )

    def cancel_response_callback(self, future) -> None:
        try:
            cancel_response = future.result()

        except Exception as error:
            self.cancel_in_progress = False

            self.get_logger().error(
                f"Failed to cancel MoveRobot goal: {error}"
            )
            return

        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().warn(
                "MoveRobot goal cancellation was accepted."
            )
        else:
            self.cancel_in_progress = False

            self.get_logger().error(
                "MoveRobot cancellation was rejected "
                "or the goal had already finished."
            )

    def result_callback(self, future) -> None:
        try:
            wrapped_result = future.result()
            result = wrapped_result.result
            status = wrapped_result.status

        except Exception as error:
            self.goal_handle = None
            self.goal_in_progress = False
            self.cancel_in_progress = False

            self.get_logger().error(
                f"Failed to receive MoveRobot result: {error}"
            )
            return

        status_name = GOAL_STATUS_NAMES.get(
            status,
            f"STATUS_{status}",
        )

        self.goal_handle = None
        self.goal_in_progress = False
        self.goal_request_in_progress = False
        self.cancel_in_progress = False
        self.cancel_when_accepted = False

        self.get_logger().info(
            "MoveRobot action finished. "
            f"status={status_name}, "
            f"final_x={result.pose_x:.3f}, "
            f"final_y={result.pose_y:.3f}"
        )

        if self.person_blocking:
            self.get_logger().warn(
                "Robot remains stopped because the path is blocked."
            )
            return

        if (
            self.resume_pending
            and self.resume_when_clear
        ):
            self.resume_pending = False

            self.get_logger().info(
                "Path is clear. Restarting after cancellation."
            )

            self.send_start_goal()

    def destroy_node(self) -> None:
        if hasattr(self, "server_timer"):
            self.server_timer.cancel()

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)

    node: Optional[PersonSafetyActionClient] = None

    try:
        node = PersonSafetyActionClient()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
