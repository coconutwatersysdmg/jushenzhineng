from services.sensor_calibration_service import SensorCalibrationService

svc=SensorCalibrationService()
intr=svc.camera_intrinsic('CAM_PICK')
assert round(intr['fx'],4)==904.6748
r=svc.pixel_depth_to_world('CAM_PICK',intr['cx'],intr['cy'],1000.0,{'x_mm':100,'y_mm':200,'z_mm':300,'roll_deg':0,'pitch_deg':0,'yaw_deg':0},depth_mode='raw')
# depth_scale=0.001 => raw 1000 -> 1000mm; principal point => camera x/y = 0
assert all(abs(a-b)<1e-6 for a,b in zip(r['world_xyz_mm'],[100,200,1300]))
print('V8 CALIBRATION TEST PASSED',r['world_xyz_mm'])
