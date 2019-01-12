# Robot Motions Server ROS
This package defines a "task" (any high-level action) server node, utilizing ROS Actionlib, that: 
- Accepts task action requests
- Interfaces with a user's defined Task Handler that
    1. Checks the validity of the task and 
    2. Constructs and returns an instance of a task that implements the base `AbstractTask` 
- Manages the life-cycle of your task, so creation, deletion, cancelation, abortion, failure, etc. 
- Reports back failures and successes to the Action Client(s)

To use this package, you need to define a Python package called `task_config` with the following: 
- A file called `task_config` where you define a `CommandHandler` and `SafetyResponder`class. 
    - `SafetyResponder` is used if Safety is enabled and the system goes into a safety response state. 
- These classes need to implement the interfaces laid out [here](https://github.com/asaba96/robot_motions_server_ros/blob/master/src/motions_server/abstract_command_handler.py) and [here](https://github.com/asaba96/robot_motions_server_ros/blob/master/src/motions_server/abstract_safety_responder.py)
- Each of your tasks also need to implement the `AbstractTask` defined [here](https://github.com/asaba96/robot_motions_server_ros/blob/master/src/motions_server/abstract_task.py)


## Notes on the usage of this package: ##

### Params: ###
- *Action Server Name* - Name of the action server to send requests to
- *Safety Enabled* - utilize the Iarc7 Safety Node Monitoring packages (see [here](https://github.com/Pitt-RAS/iarc7_safety) for Iarc7 Safety)
- *Update Rate* - update rate of main state logic
- *Startup Timeout* - timeout on startup of node
- *Force Cancel* - if a task refuses to cancel, should the TaskManager force a cancel

### Action Server Message: ###
- See the `action` folder for the action message types used for the Action Server

### Design Choices: ###
- Tasks are not trusted, so every task interaction is wrapped for protection; this means, unless the node is in a fatal state, a troublesome task will not bring down the `TaskManager` node
- Once a cancelation requests comes in, depending on the `Force Cancel` param, the behavior is the following: 
    - If `True`: 
        - The task is canceled, regardless if it returned `False` on `task.cancel()`
    - If `False`: 
        - The current task is canceled if it returned `True` on the `task.cancel()` call; otherwise, it is allowed to continue running
- If a new action comes in while the current one is running, the behavior of the Actionlib is to make the server aware there is a new task available (and overwrite the pending action if a third action is requested before the first completes). 
    - However, the `TaskActionServer` will check that the new action request has the `preempt` flag set to `True`
    - If so, the current task goes into a `Cancel Requested` state
    - If not, the current task is allowed to continue running until it completes and then the new task takes over
    
## Example Motion Task Config ##
See [here](https://github.com/asaba96/example_motion_task_config) for an example Motion Task Config package setup.
