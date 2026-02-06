# Actuator status (vacuum, brushes, etc.)
std_msgs/Header header

uint8 vacuum_speed       # 0-100%
uint8 main_brush_speed   # 0-100%
uint8 side_brush_speed   # 0-100%
uint8 water_pump_speed   # 0-100% (2-in-1 mop box)
bool lidar_enabled       # LiDAR motor running
uint8 led_state          # LED mode (0-18, see COMMANDS.md)
