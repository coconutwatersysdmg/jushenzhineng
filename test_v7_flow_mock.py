from pathlib import Path
from PIL import Image
from controllers.flow_controller import FlowController

class MockAlgorithms:
    def pre_pick_offset(self, image_path, cargo): return {'success':True,'should_fork':True,'overhang_percent':1.2,'message':'允许插取'}
    def post_place_offset(self, image_path, cargo): return {'success':True,'signed_center_offset_mm':8.0,'offset_distance_mm':8.0,'message':'偏移8mm'}
    def pallet_hole_recognize(self,*args,**kwargs): return {'success':True,'left_xyz_mm':[-100,0,1000],'right_xyz_mm':[100,0,1000],'message':'孔位完成'}
    def radar_process(self, locate_result, cargo): return locate_result
    def corner_image_recognize(self, image_paths_by_point, radar_result, cargo):
        return {'success':True,'image_points':{k:{'x':320.0,'y':240.0,'confidence':0.9} for k in radar_result['corner_ids']}}

def main():
    root=Path(__file__).resolve().parent/'workdir'/'test_v7_images'; root.mkdir(parents=True,exist_ok=True)
    p=root/'dummy.jpg'; Image.new('RGB',(640,480),'white').save(p)
    ctl=FlowController(algorithms=MockAlgorithms())
    ctl.set_plan([{'cargo_code':'C1','cargo_name':'货物','quantity':2,'length_mm':1200,'width_mm':1000,'height_mm':900,'pallet_reference_width_mm':1200}])
    images={'pre_pick_offset':str(p),'post_place_offset':str(p),'face_a':str(p),'face_b':str(p),'state_current':str(p)}
    ctl.set_debug_inputs({'images':images,'corner_images':{f'P{i}':str(p) for i in range(1,7)}})
    ctl.start(); steps=0
    while not ctl.finished and steps<50:
        try: ctl.execute_next()
        except RuntimeError as e:
            print('error',e); raise
        steps+=1
    print('steps',steps,'completed',len(ctl.completed),'results',len(ctl.results),'finished',ctl.finished)
    assert ctl.finished and len(ctl.completed)==2
    assert steps==23, steps  # v8: 14-step first round + 9-step repeat
    assert list(ctl.twin.snapshot()['cameras'])==['CAM_PICK']
    print('V7 FLOW TEST PASSED')

if __name__=='__main__': main()
