#!/usr/bin/env python

import threading
import rospy
import actionlib

from robot_motions_server_ros.msg import TaskRequestAction, TaskRequestResult


class TaskActionServer(object):
    def __init__(self, server_name):
        self._action_server = actionlib.ActionServer(
            server_name,
            TaskRequestAction,
            self._new_goal,
            cancel_cb=self._cancel_request,
            auto_start=False,
        )
        self._current_request = None
        self._new_request = None
        self._cancel_requested = False
        self._lock = threading.RLock()

        self._action_server.start()

    def _new_goal(self, goal):
        with self._lock:
            rospy.loginfo("TaskActionServer: New goal received")
            preempt = goal.get_goal().preempt

            if self._current_request is not None:
                rospy.loginfo(
                    "TaskActionServer: new goal requested with one already running"
                )
                if preempt:
                    rospy.loginfo(
                        "TaskActionServer: preempt requested, canceling current request"
                    )
                    self._current_request.set_cancel_requested()
                    self._cancel_requested = True
                else:
                    rospy.loginfo(
                        "TaskActionServer: preempt NOT requested, waiting for current task to finish"
                    )

            self._new_request = goal

    def _cancel_request(self, cancel):
        with self._lock:
            rospy.logdebug("TaskActionServer: cancel requested")
            # actionlib equality check does not handle None objects, so check if None
            # this can happen if cancel comes before goal can be accepted
            if (
                self._current_request is not None
                and cancel == self._current_request
            ):
                self._current_request.set_cancel_requested()
                self._cancel_requested = True
            else:
                # TODO: come up with a better response in this case
                rospy.logerr(
                    "TaskActionServer: requested cancel is not the same as current goal"
                )

    def set_succeeded(self, succeeded=True):
        with self._lock:
            if self._current_request is not None:
                rospy.logdebug("TaskActionServer: current request completed")
                response = TaskRequestResult(success=succeeded)
                self._current_request.set_succeeded(response)
            else:
                rospy.logerr(
                    "TaskActionServer: set succeeded called but no current request available"
                )
            self._current_request = None
            self._cancel_requested = False

    def set_failed(self):
        self.set_succeeded(succeeded=False)

    def set_aborted(self):
        self.set_succeeded(succeeded=False)

    def set_canceled(self):
        with self._lock:
            if self._current_request is not None:
                rospy.logdebug("TaskActionServer: setting canceled")
                self._current_request.set_canceled()
            else:
                rospy.logerr("TaskActionServer: No request to cancel")

            self._current_request = None
            self.set_cancel_requested = False

    def task_canceled(self):
        with self._lock:
            return self._cancel_requested

    def get_new_task(self):
        with self._lock:
            return self._new_request.get_goal()

    def set_accepted(self):
        with self._lock:
            rospy.loginfo("TaskActionServer: new goal accepted")
            self._new_request.set_accepted()
            self._current_request = self._new_request
            self._cancel_requested = False
            self._new_request = None

    def set_rejected(self):
        with self._lock:
            rospy.logwarn("TaskActionServer: rejecting task")
            self._new_request.set_rejected()
            self._current_request = None
            self._cancel_requested = False
            self._new_request = None

    def has_new_task(self):
        with self._lock:
            return self._new_request is not None
