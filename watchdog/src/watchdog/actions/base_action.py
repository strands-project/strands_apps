import rospy

class RegisterAction(type):
    def __init__(cls, name, bases, nmspc):
        super(RegisterAction, cls).__init__(name, bases, nmspc)
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
    
class ActionType(object):
    __metaclass__ = RegisterAction
    name = "None"
    description = ""
    config_keys = []    # A list of (field_name, field_description) tuples.

    def __init__(self, action_config):
        """ action_config: the config dictionary for this action """
        # Check the keys are present in the config
        for key in self.config_keys:
            if not action_config.has_key(key):
                raise Exception("'{}' action missing field '{}' in yaml".format(self.name,
                                                                                 key))
        self.__dict__.update(action_config)

    
    @classmethod
    def get_action(cls, name):
        if not cls.registry.has_key(name):
            raise Exception("Unknown action type '%s'" % name)
        return cls.registry[name]

    @classmethod
    def create(cls, action_config):
        try:
            action_type = action_config['action_type']
        except KeyError, e:
            raise Exception("Action config missing field 'action_type'")
            
        if not cls.registry.has_key(action_config['action_type']):
            raise Exception("Unknown action type '{}'".format(action_type))
        rospy.loginfo("Creating '{}' action".format(action_type))
        return cls.registry[action_type](action_config)
        
    def execute(self):
        """
        Called when the watchdog is fired.
        All actions must implement this method.
        """
        pass

