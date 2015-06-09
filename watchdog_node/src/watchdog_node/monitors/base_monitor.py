import rospy

class RegisterMonitor(type):
    def __init__(cls, name, bases, nmspc):
        super(RegisterMonitor, cls).__init__(name, bases, nmspc)
        if not hasattr(cls, 'registry'):
            cls.registry = {}
        cls.registry[cls.name] = cls
        # Remove base classes
        for b in bases:
            if hasattr(b,"name"):
                if cls.registry.has_key(b.name):
                    cls.registry.pop(b.name)
                    
    # Metamethods, called on class objects:
    def __iter__(cls):
        return iter(cls.registry)
    
    def __str__(cls):
        if cls in cls.registry:
            return cls.__name__
        return cls.__name__ + ": " + ", ".join([sc for sc in cls])
    
class MonitorType(object):
    __metaclass__ = RegisterMonitor
    name = "None"
    description = ""
    config_keys = []   # A list of (field_name, field_description) tuples.

    def __init__(self, monitor_config, invalid_cb):
        """ monitor_config: the config dictionary for this monitor """
        # Check the keys are present in the config
        for (key, description) in self.config_keys:
            if not monitor_config.has_key(key):
                raise Exception("'{}' monitor missing field '{}' in yaml".format(self.name,
                                                                                 key))
        self.__dict__.update(monitor_config)
        self._invalid_cb = invalid_cb

    
    @classmethod
    def get_monitor(cls, name):
        if not cls.registry.has_key(name):
            raise Exception("Unknown monitor type '%s'" % name)
        return cls.registry[name]

    @classmethod
    def create(cls, monitor_config, invalid_cb):
        try:
            monitor_type = monitor_config['monitor_type']
        except KeyError, e:
            raise Exception("Monitor config missing field 'monitor_type'")
            
        if not cls.registry.has_key(monitor_config['monitor_type']):
            raise Exception("Unknown monitor type '{}'".format(monitor_type))
        rospy.loginfo("Creating '{}' monitor".format(monitor_type))
        return cls.registry[monitor_type](monitor_config, invalid_cb)

    def set_invalid(self):
        """ Set that this monitor wants to trigger.  """
        self._invalid_cb()
        
    def start(self):
        """
        Start this monitor, creating subscriptions and threads as needed.
        Will be called when starting up, and when restarting
        after the watchdog has fired.
        """
        pass
    
    def stop(self):
        """
        Stops this monitor, cleaning up subscriptions and threads. Called when
        shutting down the watchdog when it has fired.
        """
        pass
    

