#!/usr/bin/env python

import rospy
import traceback
import threading

from task_action_server import TaskActionServer
from motions_server.abstract_command_handler import TaskHandledState

from task_config.task_config import CommandHandler, SafetyResponder

global use_safety
use_safety = False

if use_safety:
    from iarc7_safety.SafetyClient import SafetyClient


class TaskManager(object):
    def __init__(self, action_server):
        self._action_server = action_server
        self._lock = threading.RLock()
        self._task = None

        # construct provided command handler
        self._handler = CommandHandler()

        # construct safety responder
        self._safety_responder = SafetyResponder()

        if use_safety:
            # safety client, used if Safety Enabled
            self._safety_client = SafetyClient("task_manager")

        try:
            # load params
            self._update_rate = rospy.Rate(rospy.get_param("~update_rate"))
            self._timeout = rospy.Duration(rospy.get_param("~startup_timeout"))
            self._force_cancel = rospy.get_param("~force_cancel")
        except KeyError:
            rospy.logfatal("TaskManager: Error getting params")
            raise

    def run(self):
        # this check is needed, as sometimes there are issues with simulated time
        while rospy.Time.now() == rospy.Time(0) and not rospy.is_shutdown():
            rospy.logwarn("TaskManager: waiting on time")
            rospy.sleep(0.005)

        if rospy.is_shutdown():
            raise rospy.ROSInterruptException()

        # check to make sure all dependencies are ready
        if not self._wait_until_ready(self._timeout):
            raise rospy.ROSInitException()

        # forming bond with safety client, if enabled
        if use_safety and not self._safety_client.form_bond():
            raise ROSInitException(
                "TaskManager: could not form bond with safety client"
            )

        while not rospy.is_shutdown():
            # main loop
            with self._lock:
                if use_safety and self._safety_client.is_fatal_active():
                    raise RuntimeError("TaskManager: Safety Fatal")

                if use_safety and self._safety_client.is_safety_active():
                    rospy.logerr("TaskManager: Activating safety response")
                    self._safety_responder.activate_safety_response()
                    return

                if self._task is None and self._action_server.has_new_task():
                    # new task is available
                    try:
                        request = self._action_server.get_new_task()
                        # use provided handler to check that request is valid
                        new_task = self._handler.check_request(request)
                        if new_task is not None:
                            self._task = new_task
                            self._action_server.set_accepted()
                        else:
                            rospy.logerr("TaskManager: new task is invalid")
                            self._action_server.set_rejected()
                    except Exception as e:
                        rospy.logfatal(
                            "TaskManager: Exception getting new task"
                        )
                        raise

                if (
                    self._task is not None
                    and self._action_server.task_canceled()
                ):
                    # there is a task running, but it is canceled
                    try:
                        success = self._task.cancel()
                        if not success:
                            rospy.logwarn(
                                "TaskManager: task refusing to cancel"
                            )
                            if self._force_cancel:
                                rospy.logwarn(
                                    "TaskManager: forcing task cancel"
                                )
                                self._action_server.set_canceled()
                                self._task = None
                        elif success:
                            self._action_server.set_canceled()
                            self._task = None
                    except Exception as e:
                        rospy.logfatal("TaskManager: Error canceling task")
                        raise

                if self._task is not None:
                    # can get next command and task state
                    task_state = None

                    try:
                        state, command = self._task.get_desired_command()
                    except Exception as e:
                        rospy.logerr(
                            "TaskManager: Error getting command from task. Aborting task"
                        )
                        rospy.logerr(str(e))
                        task_state = TaskHandledState.ABORTED

                    try:
                        if task_state is None:
                            task_state = self._handler.handle(command, state)
                    except Exception as e:
                        # this is a fatal error, as the handler should be able to
                        # handle commands of valid tasks without raising exceptions
                        rospy.logfatal(
                            "TaskManager: Error handling task command"
                        )
                        rospy.logfatal(str(e))
                        raise

                    if task_state == TaskHandledState.ABORTED:
                        rospy.logerr("TaskManager: aborting task")
                        self._action_server.set_aborted()
                        self._task = None
                    elif task_state == TaskHandledState.DONE:
                        rospy.loginfo("TaskManager: task has completed cleanly")
                        self._action_server.set_succeeded()
                        self._task = None
                    elif task_state == TaskHandledState.FAILED:
                        rospy.logerr("TaskManager: Task failed")
                        self._action_server.set_failed()
                        self._task = None
                    elif task_state == TaskHandledState.OKAY:
                        rospy.loginfo_throttle(1, "TaskManager: task running")
                    else:
                        raise ValueError(
                            "TaskManager: invalid task handled state returned from handler: "
                            + str(task_state)
                        )

            self._update_rate.sleep()

    def _wait_until_ready(self, timeout):
        # wait until dependencies are ready
        try:
            self._handler.wait_until_ready(timeout)
            self._safety_responder.wait_until_ready(timeout)
        except Exception:
            rospy.logfatal("TaskManager: Error waiting for dependencies")
            raise
        return True


if __name__ == "__main__":
    rospy.init_node("task_manager")

    rospy.loginfo("TaskManager: Task Manager starting up")

    server_name = rospy.get_param("~action_server_name")
    action_server = TaskActionServer(server_name)
    task_manager = TaskManager(action_server)

    try:
        task_manager.run()
    except Exception as e:
        rospy.logfatal("TaskManager: Error while running")
        rospy.logfatal(str(e))
        rospy.logfatal(traceback.format_exc())
        raise
    finally:
        rospy.signal_shutdown("TaskManager shutdown")
