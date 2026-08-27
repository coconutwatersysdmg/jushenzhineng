# -*- coding: utf-8 -*-
from controllers.flow_controller import FlowController


controller = FlowController()
controller.set_plan([{
    "cargo_code": "INVENTORY",
    "cargo_name": "整批预置货物",
    "quantity": 3,
    "length_mm": 1200,
    "width_mm": 1000,
    "height_mm": 900,
    "pallet_reference_width_mm": 1200,
}])

# The idle scene already contains every task cargo at the truck head.
initial = controller.twin.snapshot()["cargo_inventory"]
assert len(initial) == 3, initial
assert all(item["status"] == "STAGED" for item in initial), initial
assert all(float(item["pose"]["y_mm"]) <= 1000.0 for item in initial), initial
assert all(float(item["pose"]["x_mm"]) == -2400.0 for item in initial), initial
assert len({float(item["pose"]["y_mm"]) for item in initial}) == 3, initial
assert len({item["instance_id"] for item in initial}) == 3

controller.start()
while controller.round_index == 0:
    controller.execute_next()

# Starting round two must retain the first placed load, select the second at
# its original staging pose, and leave the third visible at the head.
second_round = controller.twin.snapshot()["cargo_inventory"]
by_status = {status: [c for c in second_round if c["status"] == status] for status in ("PLACED", "CURRENT", "STAGED")}
assert len(by_status["PLACED"]) == 1, second_round
assert len(by_status["CURRENT"]) == 1, second_round
assert len(by_status["STAGED"]) == 1, second_round
assert by_status["PLACED"][0]["instance_id"] == initial[0]["instance_id"]
assert by_status["CURRENT"][0]["instance_id"] == initial[1]["instance_id"]
assert float(by_status["PLACED"][0]["pose"]["y_mm"]) > 13000.0, by_status["PLACED"]

print("V8_CARGO_INVENTORY_OK", [(c["instance_id"], c["status"], c["pose"]["y_mm"]) for c in second_round])
