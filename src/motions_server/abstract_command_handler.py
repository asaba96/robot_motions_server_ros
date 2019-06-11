#!/usr/bin/env python

"""
AbstractCommandHandler: abstract class that defines
an interface for a Task Manager to interact with
action requests, handle task checking and construction,
and handle task commands and states.
"""

from enum import Enum


class TaskHandledState(Enum):
    OKAY = 1  # task is running and is okay
    FAILED = 2  # task failed
    ABORTED = 3  # task needs aborted
    DONE = 4  # task finished cleanly


class AbstractCommandHandler(object):
    # abstract method
    def check_request(self, request):
        """
        Checks next request to see if it is valid and returns
        instace of task to handle that request

        Args:
            request (str): Name of next request

        Returns:
            task (object): Instance of task to handle request,
                None if request is invalid
        """
        raise NotImplementedError("Subcalss must implement abstact method")

    # abstract method
    def handle(self, command, state):
        """
        Handles the next command and state from the task

        Args:
            command (object): Next command from task
            state (object): Current state of the task

        Returns:
            task_state (TaskHandledState): State of the task
        """
        raise NotImplementedError("Subclass must implement abstract method")

    # abstract method
    def wait_until_ready(self, timeout):
        """
        Waits until all dependencies are ready or timeout is reached.
        If timeout reached, method is to raise an Exception

        Args:
            timeout (rospy.Duration): time to wait for dependencies
        """
        raise NotImplementedError("Subclass must implement abstract method")
