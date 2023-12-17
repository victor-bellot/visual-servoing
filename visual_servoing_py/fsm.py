import sys


def str_to_fun(fun_string):
    module, _, function = fun_string.rpartition('.')
    if module:
        __import__(module)
        mod = sys.modules[module]
    else:
        mod = sys.modules['__main__']  # or whatever is the "default module"
    return getattr(mod, function)


class Fsm:
    def __init__(self):
        self.transitions = {}
        self.states = []
        self.events = []
        
        self.current_state = None
        self.current_event = None
        self.previous_state = None
        self.end_state = None

    def load_fsm_from_file(self, file_fsm):
        fsm_file = open(file_fsm)
        mode = None
        for line in fsm_file.readlines():
            line = line[0:-1]  # remove line break
            if line.startswith('#'):
                continue
            elif line.startswith("----- States"):
                mode = "st"
            elif line.startswith("----- Events"):
                mode = "ev"
            elif line.startswith("----- Transitions"):
                mode = "tr"
            elif line.startswith("---- Start State"):
                mode = "ss"
            elif line.startswith("---- Start Event"):
                mode = "se"
            elif line.startswith("---- End State"):
                mode = "es"
            else:
                if mode == "ss":
                    self.current_state = line
                    print("Set current state", line)
                elif mode == "es":
                    self.end_state = line
                    print("Set end state", line)
                elif mode == "se":
                    self.current_event = line
                    print("Set current event", line)
                elif mode == "tr":
                    sl = line.split(" ")
                    print("Add transition", sl)
                    action = str_to_fun(sl[3])
                    self.add_transition(sl[0], sl[1], sl[2], action)
                elif mode == "ev":
                    self.add_event(line)
                    print("Add event", line)
                elif mode == "st":
                    self.add_state(line)
                    print("Add state", line)
        fsm_file.close()

    def add_transition(self, from_state, to_state, event, funct):
        key = from_state+'.'+event
        self.transitions[key] = (to_state, funct)

    def add_state(self, state):
        self.states.append(state)

    def add_event(self, event):
        self.events.append(event)

    def set_state(self, state):
        self.current_state = state

    def set_end_state(self, state):
        self.end_state = state

    def set_event(self, event):
        self.current_event = event

    def run(self):
        event = self.current_event
        from_state = self.current_state

        key = from_state+'.'+event
        to_state, action = self.transitions[key]

        if from_state != to_state:
            transition_str = ("Transition : %s ---%s---> %s ; Action : %s"
                              % (from_state, event, to_state, action.__name__))
            print(transition_str)

        self.previous_state = from_state
        self.current_state = to_state

        return action
