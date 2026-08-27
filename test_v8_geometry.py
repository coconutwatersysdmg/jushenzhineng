from services.camera_board_geometry_service import CameraBoardGeometryService
svc=CameraBoardGeometryService()
flat={'P1':[-1200,3000,1450],'P2':[1200,3000,1450],'P3':[-1200,15000,1450],'P4':[1200,15000,1450]}
r=svc.analyze(flat,['P1','P2','P3','P4'])
assert r['board_mode']=='flat' and len(r['regions'])==20
assert r['regions'][0]['blind_code']=='A01' and r['regions'][1]['blind_code']=='B01'
assert r['regions'][0]['center_world_xyz_mm'][1] > r['regions'][-1]['center_world_xyz_mm'][1]
hl={'P1':[-1200,3000,1650],'P2':[1200,3000,1650],'P3':[-1200,7380,1650],'P4':[1200,7380,1650],'P5':[-1200,15000,1450],'P6':[1200,15000,1450]}
r=svc.analyze(hl,[f'P{i}' for i in range(1,7)])
assert r['board_mode']=='high_low'
assert abs(r['height_difference_mm']-200)<1e-6
assert r['borrow_plan']['enabled'] and abs(r['borrow_plan']['borrowed_from_second_mm']-420)<1e-6
print('V8 GEOMETRY TEST PASSED',r['height_difference_mm'],r['borrow_plan'])
