# -*- coding: utf-8 -*-
import sqlite3
from pathlib import Path
from tempfile import TemporaryDirectory

from controllers.flow_controller import FlowController
from services.vehicle_database_service import VehicleDatabaseService


with TemporaryDirectory() as folder:
    db_path=Path(folder)/"vehicle_test.db"
    controller=FlowController()
    controller.vehicle_db.close()
    controller.vehicle_db=VehicleDatabaseService(db_path)
    controller.loading_session_id=""; controller._db_message_cursor=0
    controller.set_plan([{
        "task_code":"DB-TEST-001","cargo_code":"DB-CARGO","cargo_name":"车辆中心数据库验证货物","quantity":1,
        "length_mm":1200,"width_mm":1000,"height_mm":900,"pallet_reference_width_mm":1200,
    }])
    probe=sqlite3.connect(str(db_path))
    assert probe.execute("SELECT inventory_status,is_loaded FROM cargo_inventory").fetchone()==("AVAILABLE",0)
    controller.start()
    assert probe.execute("SELECT inventory_status,is_loaded FROM cargo_inventory").fetchone()==("RESERVED",0)
    controller.execute_next(); controller.execute_next()
    assert probe.execute("SELECT inventory_status,is_loaded FROM cargo_inventory").fetchone()==("LOADING",0)
    probe.close()
    while not controller.finished:
        controller.execute_next()
    session_id=controller.loading_session_id
    status=controller.vehicle_db.status(session_id)

    conn=sqlite3.connect(str(db_path)); conn.row_factory=sqlite3.Row
    session=dict(conn.execute("SELECT * FROM loading_session WHERE session_id=?",(session_id,)).fetchone())
    overview=dict(conn.execute("SELECT * FROM v_vehicle_loading_overview WHERE session_id=?",(session_id,)).fetchone())
    columns={row[0] for row in conn.execute("SELECT DISTINCT column_code FROM loading_region WHERE session_id=?",(session_id,))}
    assets={row[0] for row in conn.execute("SELECT DISTINCT asset_type FROM module_io_asset")}
    inventory=dict(conn.execute("SELECT * FROM v_cargo_inventory_status").fetchone())
    cargo_images=[dict(row) for row in conn.execute("SELECT * FROM cargo_image ORDER BY cargo_image_id")]
    foreign_key_errors=list(conn.execute("PRAGMA foreign_key_check"))
    assert session["truck_id"]=="TRUCK-01" and session["task_code"]=="DB-TEST-001",session
    assert session["status"]=="COMPLETED" and session["last_step_code"]=="RETURN",session
    assert overview["cargo_total"]==1 and overview["cargo_placed"]==1 and overview["placement_count"]==1,overview
    assert inventory["inventory_status"]=="LOADED" and inventory["is_loaded"]==1,inventory
    assert inventory["loaded_truck_id"]=="TRUCK-01" and inventory["loaded_session_id"]==session_id,inventory
    assert inventory["loaded_region_id"] and inventory["primary_image_path"],inventory
    assert cargo_images and any(image["image_type"]=="DEPTH" for image in cargo_images),cargo_images
    assert all(image["image_path"] for image in cargo_images),cargo_images
    assert columns=={"A","B"},columns
    for table in ("vehicle","loading_session","cargo_inventory","cargo_item","cargo_image","truck_board_snapshot","loading_region","workflow_step_run","placement_record","module_run","module_io_asset","plc_message","device_motion"):
        assert conn.execute(f"SELECT COUNT(*) FROM {table}").fetchone()[0]>0,table
    assert "IMAGE" in assets and "POINT_CLOUD" in assets,assets
    assert not foreign_key_errors,foreign_key_errors
    conn.close(); controller.vehicle_db.close()
    print("V8_VEHICLE_DATABASE_OK",session_id,status["counts"],overview)
