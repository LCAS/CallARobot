# Send A Robot

Send A Robot (SAR) is used to initiate a task (limited to tall-robot edge-tasks currently) by specifying the location to perform the task on with the tunnel, row and edge ids (to be expanded further to include a field for field id). And specifying the task type (uv_treatment or data_collection) and specifying a robot of interest which is capable of completing the task. Some fields in this can be left empty. Notably the robot selection can be left as `closest`; if a row is selected the edge can be left as `all`; if a tunnel is specified the row can also be left as `all`.

The interface also includes an emergency stop button which will bypass most checks and cancel the task, canceling for any associated robots.

stages:
- INIT - called when page is first loaded
- sar_INIT - called when is 
- sar_BEGUN - set when task is requested
- sar_AWAIT_START - set when robot is assigned and begins navigation
- sar_AWAIT_TASK_COMPLETION - set when robot gets to start of edge task
- sar_COMPLETE - set when robot completes final stage in task
- sar_CANCEL - set when a task is cancelled on device
- sar_CANCEL_REMOTE - set when a task is cancelled remotely
- sar_EMERGENCY



            listen_for = {
                # 'INIT': None, #we reset user state?
                'sar_BEGUN': self.begun,
                # 'sar_AWAIT_START': None, #robot en-route
                # 'sar_AWAIT_TASK_COMPLETION': None, #action started
                # 'sar_COMPLETE': None, #task fin
                # 'sar_INIT': None, #interface reset
                'sar_CANCEL': self.cancel, #user cancel
                # 'sar_CANCEL_REMOTE': None, #remote cancel
                'sar_EMERGENCY': self.emergency_stop
            }