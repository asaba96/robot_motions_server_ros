#!/usr/bin/env python

"""
AbstractSafetyResponder: abstract class that defines
an interface for a Task Manager to use when the system
enters a safety activated state.

The SafetyResponder is to initiate a safety response so
that the system does not fail fatally and recovers and
goes into a safe, grounded, not-moving state
"""


class AbstractSafetyResponder(object):
    # abstract method
    def activate_safety_response(self):
        """
        Called when Safety event is received. This method
        is responsible for putting the system into a safety
        response that safely brings the system down.
        """
        raise NotImplementedError("Subcalss must implement abstract method")

    # abstract method
    def wait_until_ready(self, timeout):
        """
        Waits until all dependencies are ready or timeout is reached.
        If timeout reached, method is to raise an Exception

        Args:
            timeout (rospy.Duration): time to wait for dependencies
        """
        raise NotImplementedError("Subclass must implement abstract method")
