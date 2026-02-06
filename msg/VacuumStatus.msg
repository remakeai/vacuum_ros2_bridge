# Complete vacuum robot status
std_msgs/Header header

# Battery
float32 battery_voltage      # Volts (13.5-15.5V typical)
uint8 battery_percent        # 0-100%
bool is_charging             # Currently charging
bool is_docked               # On charging dock

# Attachments
bool dustbox_attached        # Dustbox present
uint8 water_tank_level       # 0-100% (2-in-1 mop box only)

# Buttons
bool start_button_pressed
bool dock_button_pressed
