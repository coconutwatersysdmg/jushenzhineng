from pathlib import Path
from copy import deepcopy
import numpy as np
import cv2

from controllers.flow_controller import FlowController

TMP=Path('workdir/test_v8_inputs'); TMP.mkdir(parents=True,exist_ok=True)
rgb=np.zeros((720,1280,3),dtype=np.uint8); rgb[:]=180
depth=np.full((720,1280),1000,dtype=np.uint16)
rgb_path=TMP/'dummy.jpg'; dep_path=TMP/'dummy_depth.png'; cv2.imwrite(str(rgb_path),rgb); cv2.imwrite(str(dep_path),depth)

class FakeAlgorithms:
    def pre_pick_offset(self,image_path,cargo): return {'success':True,'should_fork':True,'overhang_percent':1.0,'message':'允许插取'}
    def pallet_hole_recognize(self,rgb_path,depth_path,cargo): return {'success':True,'left_xyz_mm':[-100,0,1000],'right_xyz_mm':[100,0,1000],'message':'插孔OK'}
    def radar_process(self,locate_result,cargo): return deepcopy(locate_result)
    def corner_image_recognize(self,images,ids,cargo): return {'success':True,'image_points':{pid:{'x':640.0,'y':360.0,'confidence':0.9} for pid in ids},'message':'角点OK'}
    def post_place_offset(self,image_path,cargo): return {'success':True,'offset_distance_mm':3.0,'signed_center_offset_mm':-3.0,'message':'货物托盘偏移OK'}

class FakeCornerWorld:
    def convert(self,image_points,capture_meta,radar_result,depth_mode='raw'):
        ids=radar_result['corner_ids']
        if len(ids)==6:
            pts={
                'P1':{'x':-1200,'y':3000,'z':1650},'P2':{'x':1200,'y':3000,'z':1650},
                'P3':{'x':-1200,'y':7380,'z':1650},'P4':{'x':1200,'y':7380,'z':1650},
                'P5':{'x':-1200,'y':15000,'z':1450},'P6':{'x':1200,'y':15000,'z':1450},
            }
        else:
            pts={'P1':{'x':-1200,'y':3000,'z':1450},'P2':{'x':1200,'y':3000,'z':1450},'P3':{'x':-1200,'y':15000,'z':1450},'P4':{'x':1200,'y':15000,'z':1450}}
        return {'success':True,'corner_ids':ids,'world_points':{k:pts[k] for k in ids},'corner_points_xyz_mm':[[pts[k]['x'],pts[k]['y'],pts[k]['z']] for k in ids],'source':'fake_camera_world'}

c=FlowController(algorithms=FakeAlgorithms()); c.corner_world=FakeCornerWorld()
c.set_plan([{'cargo_code':'T','cargo_name':'测试货物','quantity':2,'length_mm':1200,'width_mm':1000,'height_mm':900,'pallet_reference_width_mm':1200,'radar_demo_corner_count':6}])
vals={'pallet_rgb':str(rgb_path),'pallet_depth':str(dep_path),'images':{'pre_pick_offset':str(rgb_path),'neighbor_pose':str(rgb_path),'region_deviation':str(rgb_path),'face_a':str(rgb_path),'face_b':str(rgb_path),'post_place_offset':str(rgb_path)},'corner_images':{},'corner_depths':{}}
for i in range(1,7): vals['corner_images'][f'P{i}']=str(rgb_path); vals['corner_depths'][f'P{i}']=str(dep_path)
c.set_debug_inputs(vals); c.start()
steps=[]
while not c.finished:
    steps.append((c.round_index+1,c.current_step[0]))
    c.execute_next()
assert len(steps)==27,steps
assert [code for rnd,code in steps if code in {"PRE_PLACE_MONITOR","POST_PLACE_BOTTOM"}]==[
    "PRE_PLACE_MONITOR","POST_PLACE_BOTTOM","PRE_PLACE_MONITOR","POST_PLACE_BOTTOM"
],steps
first=[x[1] for x in steps if x[0]==1]; second=[x[1] for x in steps if x[0]==2]
for forbidden in ('RADAR_TO_CAMERA','CAPTURE_CORNERS','CORNER_RECOGNITION','CAMERA_TO_WORLD','INITIAL_SPACE_PLAN','PARALLEL_LOCATE'):
    assert forbidden not in second
assert 'PICK_ONLY' in second and 'NEIGHBOR_POSE' in second and 'TARGET_CONFIRM' in second
snap=c.snapshot(); assert snap['completed']==2 and snap['finished']
assert snap['twin']['truck']['board_mode']=='high_low'
print('V8 FLOW TEST PASSED',len(first),len(second),len(steps))
print('repeat steps:',second)
