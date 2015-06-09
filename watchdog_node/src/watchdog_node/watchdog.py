import yaml
from monitors import MonitorType
from actions import ActionType
from threading import Timer, Lock
import rospy

class Watchdog(object):
    yaml_keys = ['name', 'description', 'restart_timeout', 'monitors', 'actions']
    def __init__(self, config):
        if not isinstance(config,  dict):
            raise Exception("Watchdog class must be created with dictionary "
                            "as parameter.")
        for key in self.yaml_keys:
            if key not in config:
                raise Exception("YAML config error: monitor/action pair "
                                "missing '{}' field".format(key))

        self._name = config['name']
        self._restart_timeout =  config['restart_timeout']
        self._restart_timer =  None
        self._monitors = []
        self._executing = Lock()
        for monitor in config['monitors']:
            self._monitors.append(MonitorType.create(monitor, self.execute))
        self._actions = []
        for action in config['actions']:
            self._actions.append(ActionType.create(action))
        
    
    def _start_monitors(self):
        for monitor in self._monitors:
            monitor.start()
            
    def _stop_monitors(self, restart_after_timeout=False):
        for monitor in self._monitors:
            monitor.stop()
            
    def _check(self):
        for monitor in self._monitors:
            if monitor.check():
                return True
        return False
    
    def _execute(self):
        rospy.logwarn("Executing '{}' watchdog.".format(self._name))
        self.shutdown()
        for action in self._actions:
            action.execute()
        if self._restart_timeout > -1:
            self._restart_timer =  Timer(self._restart_timeout,
                                         self.run)
            self._restart_timer.start()
    
    def _pause(self):
        self._check_timer.cancel()
    
    def run(self):
        rospy.loginfo("Starting '{}' watchdog".format(self._name))
        self._executable = True
        self._start_monitors()
    
    def shutdown(self):
        rospy.loginfo("Stopping '{}' watchdog".format(self._name))
        self._executable = False
        self._stop_monitors()
        if self._restart_timer is not None:
            self._restart_timer.cancel()
            
    def execute(self):
        with self._executing:
            if self._executable:
                self._executable = False
                self._execute()

