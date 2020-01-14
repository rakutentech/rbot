# This contains action info which can be used for runtime decision making
# 0 priority is lowest
uint8 priority  # priority is valid across actions
uint8 id        # id is relevant only for the same action
uint8 type      # type of movement done
uint8 NONE=0                 # do nothing
uint8 EMERGENCY_OVERRIDE=1   # move after emergency stop
uint8 EMERGENCY_STOP=2       # speciies position = const or velocities = 0
uint8 COLLISION_AVOIDANCE=3  # to be used only if collision is imminent
uint8 HUMAN_OVERRIDE=4       # human overruling the ai commands
uint8 AI_CONTROL=5           # commands by ai
uint8 HUMAN_ASSIST=6         # human working with the ai, ai takes priority
uint8 __MAX__=7              # just a marker to sanitize actions
