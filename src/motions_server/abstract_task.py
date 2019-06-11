#!/usr/bin/env python

"""
AbstractTask: abstract class defining tasks, which
are defined motion actions or commands.

NOTE: This file was created originally by Pitt's Robotics
and Automation Society's IARC team and has been modifed
slightly for its use here.

Link to original:
https://github.com/Pitt-RAS/iarc7_motion/blob/master/src/iarc7_motion/iarc_tasks/abstract_task.py

"""


class AbstractTask(object):
    # Abstract method
    def get_desired_command(self):
        """
        This is the "main" function of a task. This method is called
        to get the next command desired by the task that the rest of
        the stack is to implement/act upon.

        Returns:
            TaskState: instance of whatever state the task is in
            Task Command: desired command of the task
        """
        raise NotImplementedError("Subclass must implement abstract method")

    # Abstract method
    def cancel(self):
        """
        Cancels the task.

        Returns:
            canceled: if True, canceled succeeded, otherwise task still needs to run
        """
        raise NotImplementedError("Subclass must implement abstract method")
