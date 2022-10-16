import ctypes as ct
import codecs
from sys import platform


if platform == "linux" or platform == "linux2":
    se = ct.CDLL("./libesminiLib.so")
elif platform == "darwin":
    se = ct.CDLL("./libesminiLib.dylib")
elif platform == "win32":
    se = ct.CDLL("./esminiLib.dll")
else:
    print("Unsupported platform: {}".format(platform))
    quit()

# asciidoc tag::StoryboardStateCallback[]

# Storyboard element types (see OSCTypeDefs/OSCAction.hpp)
sbe_type = {
    0:"UNDEFINED_ELEMENT_TYPE",
    1:"STORY",
    2:"ACT",
    3:"MANEUVER_GROUP",
    4:"MANEUVER",
    5:"EVENT",
    6:"ACTION"
}

# Storyboard element states (see OSCTypeDefs/OSCAction.hpp)
sbe_state = {
    0:"UNDEFINED_ELEMENT_STATE",
    1:"STANDBY",
    2:"RUNNING",
    3:"COMPLETE"
}

# Define callback for scenario storyboard element state changes
# according to function type: void (*fnPtr)(const char* name, int type, int state)
# required by SE_RegisterStoryBoardElementStateChangeCallback()
def callback(name, type, state):
    # just print the received info
    print("callback: {} ({}) state: {}".format(codecs.decode(name), sbe_type[type], sbe_state[state]))

callback_type = ct.CFUNCTYPE(None, ct.c_char_p, ct.c_int, ct.c_int)
callback_func = callback_type(callback)

# Initialize esmini before register the callback
se.SE_AddPath(b"../../../resources/xosc")  # search path if run from original location
se.SE_Init(b"../resources/xosc/cut-in.xosc", 0, 1, 0, 1)

# register the callback function
se.SE_RegisterStoryBoardElementStateChangeCallback(callback_func)

# asciidoc end::StoryboardStateCallback[]

for i in range(800):
    se.SE_Step()
