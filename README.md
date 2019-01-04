# Robot Motions Server ROS

This package defines a "task" (any high-level action) server that utilizes the Actionlib to implement a node that: 
- Accepts action requests
- Interfaces with the user's defined Task Handler to 
    1. Check the validity of the task and 
    2. Construct and return a task that implements the base `AbstractTask` define here
- Manages the life-cycle of your task, so creation, deletion, cancelation, and reporting back to the Action client failures and success. 

To use this package, you need to define a Python package called `task_config` with the following: 
- A file called `task_config` where you define a `CommandHandler` and `SafetyResponder`class. 
    - `SafetyResponder` is used if Safety is enabled and the system goes into a safety response state. 
- These classes need to implement the interface laid out here
- Define each of your tasks that implements the `AbstractTask` laid out here. 

## Notes on the usage of this package: ##

### Params: ###
- Action Server Name: Name of the action server to send requests to
- Safety Enabled - utilize the Safety Node Monitoring packages (see here)
- Update Rate - update rate of main state logic
- Startup Timeout - timeout on startup of node
- Force Cancel - if a task refuses to cancel, should the MotionTaskServer force a cancel

### Action Server Message: ###
- See the `action` folder for the action types used for the Action Server

### Design Choices: ###
- Tasks are not trusted, so every task interaction is wrapped for protection; this means, unless the node is in a fatal state, a troublesome task will not bring down this node. 
- Once a cancelation requests comes in, depending on the `Force Cancel` param, the behavior is the following: 
    - IF TRUE: 
        - The task is canceled, regardless if it returned `False` on `task.cancel()`
    - IF FALSE: 
        - The current is canceled if it returned `True` on the `task.cancel()` call; otherwise, it is allowed to continue running. 
- If a new action comes in while the current one is running, the behavior of the Actionlib is to make the server aware there is a new task available (and overwrite the pending action if a third is requested before the first completes). 
    - However, the `TaskActionServer` will check that the action request has the `preempt` flag set to True
    - If so, the current task goes into a `Cancel Requested` state
    - If not, the current task is allowed to continue running until it completes and then the new task takes over
