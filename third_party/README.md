# Hardware integration packages for jushenzhineng_v3
#
# livox_runtime/     Livox Mid360 capture exe + SDK DLL + mid360s_config.json
# plc_finished_app/  Verified gantry PLC console / engineering Modbus core
#
# Digital twin default mode is still mock. Switch in config/system_config.py:
#
#   "runtime": { "device_mode": "mock" }   # local demo
#   "runtime": { "device_mode": "real" }   # hardware adapters
#
# Real mode notes
# - Radar: devices/real_livox_radar_adapter.py -> services/livox_service.py
# - PLC connect: devices/real_plc_adapter.py -> plc_readonly_core / engineering core
# - Robot motion: devices/real_gantry_robot_adapter.py
#     requires allow_real_motion=true, world_to_gantry.placeholder=false,
#     then writes XYZR via devices/gantry_modbus_motion.py (same registers as console)
# - Camera: devices/real_arm_camera_adapter.py is a D435i stub; install pyrealsense2
#
# Standalone PLC console (axis jog / absolute move debugging):
#   cd third_party/plc_finished_app
#   python plc_finished_console.py
#
# Product PLC module (receives JSON motion cmds from the main app):
#   python -m plc_console
#   Protocol: contracts/plc_motion_cmd_v1.schema.json
#   Main app never writes axes directly; it publishes commands and optional IPC push.
