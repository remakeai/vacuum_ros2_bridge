# LED state command
# States (0-18):
#   0: Off
#   1-2: Blue stable (idle)
#   3: Orange stable (warning)
#   4: Orange breathing (charging)
#   6: Orange blink (charging active)
#   7: Red stable (error)
#   8: Red blink (error attention)
#  11: Blue blink (processing)
#  12: Blue breathing (standby)
#  17: Purple stable (special/factory)
uint8 state
