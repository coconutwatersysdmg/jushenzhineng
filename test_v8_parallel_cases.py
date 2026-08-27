from controllers.flow_controller import FlowController


def run_case(pick_ok, radar_ok):
    c=FlowController(); c.set_plan([{'cargo_code':'X','quantity':1}]); c.start(); c.round_data={}
    calls={'pick':0,'radar':0}
    def p():
        calls['pick']+=1; r={'success':pick_ok,'message':'pick'}; c.round_data['pick_result']=r; return r
    def r():
        calls['radar']+=1; x={'success':radar_ok,'message':'radar'}; c.round_data['radar_result']=x; return x
    c._pick_task=p; c._radar_task=r
    try:
        out=c._parallel_locate(); failed=False
    except RuntimeError:
        out=None; failed=True
    assert failed == (not (pick_ok and radar_ok))
    assert c.round_data['pick_result']['success']==pick_ok
    assert c.round_data['radar_result']['success']==radar_ok
    return c,calls

for pair in [(True,True),(False,True),(True,False),(False,False)]:
    c,calls=run_case(*pair)
    print(pair,'preserved',c.round_data,'calls',calls)

# 成功分支不得重跑：先 pick失败/radar成功，第二次只重试pick。
c,_=run_case(False,True)
radar_before=c.round_data['radar_result'].copy(); counter={'pick':0,'radar':0}
def p2(): counter['pick']+=1; r={'success':True,'message':'pick retry ok'}; c.round_data['pick_result']=r; return r
def r2(): counter['radar']+=1; raise AssertionError('radar should not rerun')
c._pick_task=p2; c._radar_task=r2
out=c._parallel_locate()
assert out['pick']=='SUCCESS' and out['radar']=='SUCCESS' and counter=={'pick':1,'radar':0}
assert c.round_data['radar_result']==radar_before
print('V8 PARALLEL 4-CASE TEST PASSED')
