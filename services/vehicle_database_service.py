# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import os
import re
import sqlite3
import subprocess
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from threading import RLock
from typing import Any, Mapping
from uuid import uuid4

import pymysql
from pymysql.constants import CLIENT


PROJECT_ROOT=Path(__file__).resolve().parents[1]
DEFAULT_DB_PATH=PROJECT_ROOT/"workdir"/"vehicle_loading.db"
SCHEMA_PATH=PROJECT_ROOT/"database"/"vehicle_centered_schema.sql"
MYSQL_SCHEMA_PATH=PROJECT_ROOT/"database"/"vehicle_centered_schema_mysql.sql"


def _now() -> str:
    return datetime.now().isoformat(timespec="milliseconds")


def _json(value: Any) -> str:
    return json.dumps(value if value is not None else {},ensure_ascii=False,default=str)


def _mysql_sql(sql: str) -> str:
    """Translate the small SQLite SQL subset used by this service to MySQL 8."""
    result=sql.replace("?","%s")
    result=re.sub(r"\bINSERT\s+OR\s+IGNORE\s+INTO\b","INSERT IGNORE INTO",result,flags=re.IGNORECASE)
    result=re.sub(r"\bINSERT\s+OR\s+REPLACE\s+INTO\b","REPLACE INTO",result,flags=re.IGNORECASE)
    result=re.sub(
        r"\bON\s+CONFLICT\s*\([^)]*\)\s+DO\s+UPDATE\s+SET\b",
        "ON DUPLICATE KEY UPDATE",
        result,
        flags=re.IGNORECASE,
    )
    result=re.sub(r"\bexcluded\.([A-Za-z_][A-Za-z0-9_]*)",r"VALUES(\1)",result,flags=re.IGNORECASE)
    return result


class _MySQLCompatConnection:
    """Expose the sqlite3 methods used below while committing MySQL transactions safely."""

    def __init__(self,connection: pymysql.Connection):
        self.connection=connection

    def ping(self):
        self.connection.ping(reconnect=True)

    def execute(self,sql: str,params=()):
        self.ping(); cursor=self.connection.cursor(); cursor.execute(_mysql_sql(sql),params); return cursor

    def executemany(self,sql: str,params):
        self.ping(); cursor=self.connection.cursor(); cursor.executemany(_mysql_sql(sql),params); return cursor

    def executescript(self,sql: str):
        self.ping(); cursor=self.connection.cursor(); cursor.execute(sql)
        while cursor.nextset():
            pass
        return cursor

    def commit(self): self.connection.commit()
    def rollback(self): self.connection.rollback()
    def close(self): self.connection.close()

    def __enter__(self):
        self.ping(); return self

    def __exit__(self,exc_type,exc_value,traceback):
        if exc_type is None: self.commit()
        else: self.rollback()
        return False


def _start_project_mysql(config: Mapping[str,Any]) -> None:
    executable=Path(str(config.get("server_executable") or ""))
    defaults_file=Path(str(config.get("defaults_file") or ""))
    if not executable.is_absolute(): executable=PROJECT_ROOT/executable
    if not defaults_file.is_absolute(): defaults_file=PROJECT_ROOT/defaults_file
    if not executable.is_file() or not defaults_file.is_file():
        raise RuntimeError(f"MySQL 自动启动配置无效：{executable} / {defaults_file}")
    creationflags=getattr(subprocess,"CREATE_NO_WINDOW",0)
    subprocess.Popen(
        [str(executable),f"--defaults-file={defaults_file}"],
        cwd=str(PROJECT_ROOT),stdin=subprocess.DEVNULL,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,
        creationflags=creationflags,
    )


def _mysql_connection(config: Mapping[str,Any]) -> pymysql.Connection:
    options={
        "host":str(config.get("host") or "127.0.0.1"),
        "port":int(config.get("port") or 3306),
        "user":str(config.get("user") or "root"),
        "password":str(os.environ.get(str(config.get("password_env") or "JUSHEN_MYSQL_PASSWORD")) or config.get("password") or ""),
        "database":str(config.get("database") or "jushenzhineng"),
        "charset":"utf8mb4","autocommit":False,"connect_timeout":5,
        "client_flag":CLIENT.MULTI_STATEMENTS,
    }
    try:
        return pymysql.connect(**options)
    except pymysql.MySQLError as first_error:
        if not bool(config.get("auto_start")):
            raise RuntimeError(f"MySQL连接失败：{options['host']}:{options['port']}/{options['database']}") from first_error
        _start_project_mysql(config)
        deadline=time.monotonic()+12.0; last_error=first_error
        while time.monotonic()<deadline:
            time.sleep(0.25)
            try: return pymysql.connect(**options)
            except pymysql.MySQLError as exc: last_error=exc
        raise RuntimeError(f"MySQL自动启动后仍无法连接：{options['host']}:{options['port']}/{options['database']}") from last_error


class VehicleDatabaseService:
    """Vehicle-centred persistence with one business implementation for SQLite/MySQL."""

    def __init__(self,db_path: str | Path=DEFAULT_DB_PATH,mysql_config: Mapping[str,Any] | None=None):
        self._lock=RLock()
        if mysql_config:
            self.backend="mysql"
            self.mysql_config=dict(mysql_config)
            self.db_path=None
            self.location=(f"mysql://{self.mysql_config.get('host','127.0.0.1')}:"
                           f"{int(self.mysql_config.get('port',3306))}/{self.mysql_config.get('database','jushenzhineng')}")
            self._conn=_MySQLCompatConnection(_mysql_connection(self.mysql_config))
            if bool(self.mysql_config.get("initialize_schema",False)):
                self._conn.executescript(MYSQL_SCHEMA_PATH.read_text(encoding="utf-8"))
                self._conn.commit()
            self._conn.execute("SELECT 1 FROM vehicle LIMIT 1")
        else:
            self.backend="sqlite"
            self.db_path=Path(db_path)
            if not self.db_path.is_absolute(): self.db_path=PROJECT_ROOT/self.db_path
            self.db_path.parent.mkdir(parents=True,exist_ok=True)
            self.location=str(self.db_path.resolve())
            self._conn=sqlite3.connect(str(self.db_path),timeout=15.0,check_same_thread=False)
            self._conn.row_factory=sqlite3.Row
            self._conn.execute("PRAGMA foreign_keys=ON")
            self._conn.execute("PRAGMA journal_mode=WAL")
            self._conn.execute("PRAGMA busy_timeout=15000")
            self._conn.executescript(SCHEMA_PATH.read_text(encoding="utf-8"))
            self._migrate_schema()
            self._conn.commit()

    def _migrate_schema(self):
        """Apply small additive migrations for databases created by earlier v8 builds."""
        cargo_columns={str(row[1]) for row in self._conn.execute("PRAGMA table_info(cargo_item)")}
        if "inventory_id" not in cargo_columns:
            self._conn.execute("ALTER TABLE cargo_item ADD COLUMN inventory_id TEXT REFERENCES cargo_inventory(inventory_id)")

    def close(self):
        with self._lock:
            self._conn.close()

    def start_session(self,truck: Mapping[str,Any],plan: list[dict],task_code: str="") -> str:
        session_id=f"LOAD-{datetime.now().strftime('%Y%m%d-%H%M%S')}-{uuid4().hex[:8].upper()}"
        truck_id=str(truck.get("truck_id") or "TRUCK-01")
        pose=truck.get("pose") or {}; now=_now()
        attributes={"pose":pose,"axis":{"x":"right","y":"vehicle_forward","z":"up"}}
        with self._lock,self._conn:
            self._conn.execute(
                """INSERT INTO vehicle(truck_id,plate_no,vehicle_type,coordinate_frame,length_mm,width_mm,deck_height_mm,attributes_json,created_at,updated_at)
                   VALUES(?,?,?,?,?,?,?,?,?,?)
                   ON CONFLICT(truck_id) DO UPDATE SET plate_no=excluded.plate_no,vehicle_type=excluded.vehicle_type,
                   length_mm=excluded.length_mm,width_mm=excluded.width_mm,deck_height_mm=excluded.deck_height_mm,
                   attributes_json=excluded.attributes_json,updated_at=excluded.updated_at""",
                (truck_id,truck.get("plate_no"),truck.get("vehicle_type"),"world",truck.get("length_mm"),truck.get("width_mm"),
                 pose.get("z_mm"),_json(attributes),now,now),
            )
            self._conn.execute(
                "INSERT INTO loading_session(session_id,truck_id,task_code,status,plan_json,started_at) VALUES(?,?,?,?,?,?)",
                (session_id,truck_id,task_code or session_id,"RUNNING",_json(plan),now),
            )
        return session_id

    def finish_session(self,session_id: str,status: str,error_message: str=""):
        if not session_id: return
        with self._lock,self._conn:
            self._conn.execute(
                "UPDATE loading_session SET status=?,finished_at=?,error_message=? WHERE session_id=?",
                (str(status),_now(),str(error_message or ""),session_id),
            )
            if str(status).upper()=="RESET":
                self._conn.execute(
                    """UPDATE cargo_inventory SET inventory_status='AVAILABLE',current_session_id=NULL,updated_at=?
                       WHERE current_session_id=? AND is_loaded=0""",
                    (_now(),session_id),
                )

    def register_inventory(self,cargos: list[dict]):
        """Register physical stock before a loading session reserves it."""
        now=_now()
        with self._lock,self._conn:
            for cargo in cargos or []:
                inventory_id=str(cargo.get("inventory_id") or cargo.get("stock_id") or "")
                if not inventory_id: continue
                self._conn.execute(
                    """INSERT INTO cargo_inventory(inventory_id,cargo_code,cargo_name,batch_no,length_mm,width_mm,height_mm,inventory_status,is_loaded,attributes_json,created_at,updated_at)
                       VALUES(?,?,?,?,?,?,?,'AVAILABLE',0,?,?,?)
                       ON CONFLICT(inventory_id) DO UPDATE SET cargo_code=excluded.cargo_code,cargo_name=excluded.cargo_name,
                       batch_no=excluded.batch_no,length_mm=excluded.length_mm,width_mm=excluded.width_mm,height_mm=excluded.height_mm,
                       attributes_json=excluded.attributes_json,updated_at=excluded.updated_at""",
                    (inventory_id,cargo.get("cargo_code"),cargo.get("cargo_name"),cargo.get("batch_no"),cargo.get("length_mm"),cargo.get("width_mm"),cargo.get("height_mm"),_json(cargo),now,now),
                )

    def sync_state(self,session_id: str,twin: Mapping[str,Any]):
        if not session_id: return
        now=_now(); inventory=list(twin.get("cargo_inventory") or [])
        active=twin.get("cargo") or {}
        if active.get("instance_id") and not any(x.get("instance_id")==active.get("instance_id") for x in inventory):
            inventory.append(active)
        with self._lock,self._conn:
            session_row=self._conn.execute("SELECT truck_id FROM loading_session WHERE session_id=?",(session_id,)).fetchone()
            truck_id=str(session_row[0]) if session_row else ""
            for cargo in inventory:
                cargo_id=str(cargo.get("instance_id") or cargo.get("cargo_id") or "")
                if not cargo_id: continue
                inventory_id=str(cargo.get("inventory_id") or cargo.get("stock_id") or cargo_id)
                cargo_status=str(cargo.get("status") or "AVAILABLE").upper()
                is_loaded=1 if cargo_status=="PLACED" else 0
                inventory_status={"PLACED":"LOADED","CARRIED":"LOADING","CURRENT":"RESERVED","STAGED":"RESERVED"}.get(cargo_status,"AVAILABLE")
                existing=self._conn.execute(
                    "SELECT is_loaded,loaded_session_id FROM cargo_inventory WHERE inventory_id=?",
                    (inventory_id,),
                ).fetchone()
                if existing and int(existing[0] or 0)==1 and str(existing[1] or "")!=session_id:
                    raise RuntimeError(f"库存 {inventory_id} 已装载，不能重复加入新装载会话")
                region_row=self._conn.execute(
                    "SELECT region_id FROM loading_region WHERE session_id=? AND cargo_id=? ORDER BY updated_at DESC LIMIT 1",
                    (session_id,cargo_id),
                ).fetchone()
                loaded_region_id=str(region_row[0]) if region_row else None
                self._conn.execute(
                    """INSERT INTO cargo_inventory(inventory_id,cargo_code,cargo_name,batch_no,length_mm,width_mm,height_mm,inventory_status,is_loaded,current_session_id,loaded_truck_id,loaded_session_id,loaded_region_id,loaded_at,primary_image_path,attributes_json,created_at,updated_at)
                       VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                       ON CONFLICT(inventory_id) DO UPDATE SET cargo_code=excluded.cargo_code,cargo_name=excluded.cargo_name,
                       batch_no=excluded.batch_no,length_mm=excluded.length_mm,width_mm=excluded.width_mm,height_mm=excluded.height_mm,
                       inventory_status=excluded.inventory_status,is_loaded=excluded.is_loaded,current_session_id=excluded.current_session_id,
                       loaded_truck_id=CASE WHEN excluded.is_loaded=1 THEN excluded.loaded_truck_id ELSE cargo_inventory.loaded_truck_id END,
                       loaded_session_id=CASE WHEN excluded.is_loaded=1 THEN excluded.loaded_session_id ELSE cargo_inventory.loaded_session_id END,
                       loaded_region_id=CASE WHEN excluded.is_loaded=1 THEN COALESCE(excluded.loaded_region_id,cargo_inventory.loaded_region_id) ELSE cargo_inventory.loaded_region_id END,
                       loaded_at=CASE WHEN excluded.is_loaded=1 THEN COALESCE(cargo_inventory.loaded_at,excluded.loaded_at) ELSE cargo_inventory.loaded_at END,
                       attributes_json=excluded.attributes_json,updated_at=excluded.updated_at""",
                    (inventory_id,cargo.get("cargo_code"),cargo.get("cargo_name"),cargo.get("batch_no"),cargo.get("length_mm"),cargo.get("width_mm"),cargo.get("height_mm"),
                     inventory_status,is_loaded,session_id,truck_id if is_loaded else None,session_id if is_loaded else None,loaded_region_id,_now() if is_loaded else None,
                     cargo.get("primary_image_path"),_json(cargo),now,now),
                )
                self._conn.execute(
                    """INSERT INTO cargo_item(session_id,cargo_id,inventory_id,cargo_code,cargo_name,sequence_no,length_mm,width_mm,height_mm,status,attached_to,pose_json,attributes_json,updated_at)
                       VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                       ON CONFLICT(session_id,cargo_id) DO UPDATE SET status=excluded.status,attached_to=excluded.attached_to,
                       inventory_id=excluded.inventory_id,pose_json=excluded.pose_json,attributes_json=excluded.attributes_json,updated_at=excluded.updated_at""",
                    (session_id,cargo_id,inventory_id,cargo.get("cargo_code"),cargo.get("cargo_name"),cargo.get("sequence"),cargo.get("length_mm"),
                     cargo.get("width_mm"),cargo.get("height_mm"),cargo.get("status"),cargo.get("attached_to"),_json(cargo.get("pose") or {}),_json(cargo),now),
                )
            self._upsert_regions_unlocked(session_id,((twin.get("truck") or {}).get("regions") or []),now)

    def _upsert_regions_unlocked(self,session_id: str,regions: list[dict],now: str):
        for region in regions:
            region_id=str(region.get("region_id") or region.get("blind_code") or "")
            if not region_id: continue
            center=region.get("center_world_xyz_mm") or [0,0,0]
            self._conn.execute(
                """INSERT INTO loading_region(session_id,region_id,blind_code,section_name,column_code,row_index,center_x_mm,center_y_mm,center_z_mm,corners_json,status,cargo_id,requires_support_block,support_height_mm,updated_at)
                   VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?,?)
                   ON CONFLICT(session_id,region_id) DO UPDATE SET status=excluded.status,cargo_id=excluded.cargo_id,
                   center_x_mm=excluded.center_x_mm,center_y_mm=excluded.center_y_mm,center_z_mm=excluded.center_z_mm,
                   corners_json=excluded.corners_json,requires_support_block=excluded.requires_support_block,
                   support_height_mm=excluded.support_height_mm,updated_at=excluded.updated_at""",
                (session_id,region_id,region.get("blind_code"),region.get("section"),region.get("column"),region.get("local_row_index"),
                 float(center[0]),float(center[1]),float(center[2]),_json(region.get("corners_world_xyz_mm") or []),
                 region.get("status") or "AVAILABLE",region.get("cargo_id"),int(bool(region.get("requires_support_block"))),
                 float(region.get("support_height_compensation_mm",0) or 0),now),
            )

    def record_board_snapshot(self,session_id: str,round_no: int,truck: Mapping[str,Any],geometry: Mapping[str,Any]):
        with self._lock,self._conn:
            self._conn.execute(
                "INSERT INTO truck_board_snapshot(session_id,round_no,board_mode,decision_source,corners_json,geometry_json,created_at) VALUES(?,?,?,?,?,?,?)",
                (session_id,int(round_no),geometry.get("board_mode"),geometry.get("decision_source"),_json(truck.get("corners") or {}),_json(geometry),_now()),
            )

    def record_step(self,session_id: str,record: Mapping[str,Any]):
        if not session_id: return
        with self._lock,self._conn:
            self._conn.execute(
                "INSERT INTO workflow_step_run(session_id,round_no,cargo_id,step_code,step_name,status,message,result_json,occurred_at) VALUES(?,?,?,?,?,?,?,?,?)",
                (session_id,int(record.get("round",0) or 0),record.get("cargo_id"),record.get("step_code"),record.get("step_name"),
                 record.get("status"),record.get("message"),_json(record.get("data")),record.get("time") or _now()),
            )
            self._conn.execute(
                "UPDATE loading_session SET last_step_code=?,last_step_status=?,error_message=? WHERE session_id=?",
                (record.get("step_code"),record.get("status"),record.get("message") if record.get("status")=="failed" else "",session_id),
            )

    def record_placement(self,session_id: str,round_no: int,cargo_id: str,data: Mapping[str,Any]):
        target=data.get("target") or {}; placed=data.get("placed_cargo") or {}; region_id=target.get("region_id")
        with self._lock,self._conn:
            self._conn.execute(
                """INSERT INTO placement_record(session_id,round_no,cargo_id,region_id,selected_side,target_pose_json,actual_pose_json,plc_ack_json,status,placed_at)
                   VALUES(?,?,?,?,?,?,?,?,?,?)
                   ON CONFLICT(session_id,cargo_id) DO UPDATE SET region_id=excluded.region_id,selected_side=excluded.selected_side,
                   target_pose_json=excluded.target_pose_json,actual_pose_json=excluded.actual_pose_json,plc_ack_json=excluded.plc_ack_json,status=excluded.status,placed_at=excluded.placed_at""",
                (session_id,int(round_no),cargo_id,region_id,data.get("selected_side"),_json(target.get("final_world_pose") or {}),
                 _json(placed.get("pose") or {}),_json(data.get("plc_ack") or {}),"SUCCESS",_now()),
            )
            session_row=self._conn.execute("SELECT truck_id FROM loading_session WHERE session_id=?",(session_id,)).fetchone()
            self._conn.execute(
                """UPDATE cargo_inventory SET inventory_status='LOADED',is_loaded=1,current_session_id=?,loaded_truck_id=?,
                   loaded_session_id=?,loaded_region_id=?,loaded_at=COALESCE(loaded_at,?),updated_at=?
                   WHERE inventory_id=(SELECT inventory_id FROM cargo_item WHERE session_id=? AND cargo_id=?)""",
                (session_id,str(session_row[0]) if session_row else None,session_id,region_id,_now(),_now(),session_id,cargo_id),
            )

    @staticmethod
    def _assets(value: Any,prefix: str=""):
        out=[]
        if isinstance(value,Mapping):
            for key,item in value.items(): out.extend(VehicleDatabaseService._assets(item,f"{prefix}.{key}" if prefix else str(key)))
        elif isinstance(value,(list,tuple)):
            for index,item in enumerate(value): out.extend(VehicleDatabaseService._assets(item,f"{prefix}[{index}]"))
        elif isinstance(value,str):
            suffix=Path(value).suffix.lower()
            kind={".jpg":"IMAGE",".jpeg":"IMAGE",".png":"IMAGE",".bmp":"IMAGE",".webp":"IMAGE",".pcd":"POINT_CLOUD",".json":"JSON"}.get(suffix)
            if kind: out.append((prefix,kind,value))
        return out

    @staticmethod
    def _image_type(field_path: str,direction: str) -> str:
        field=str(field_path).lower()
        if "depth" in field: return "DEPTH"
        if direction=="OUTPUT" or "result" in field or "annotat" in field: return "RESULT"
        if "pallet" in field or "hole" in field: return "PALLET"
        if "corner" in field: return "CORNER"
        return "RGB"

    def record_module_run(self,session_id: str,evidence: Mapping[str,Any],cargo_id: str=""):
        if not session_id: return
        run_id=str(evidence.get("evidence_id") or f"MODULE-{uuid4().hex}")
        inputs=deepcopy(evidence.get("inputs") or {}); output=deepcopy(evidence.get("output") or {})
        with self._lock,self._conn:
            self._conn.execute(
                """INSERT OR REPLACE INTO module_run(module_run_id,session_id,module_id,module_name,status,model_invoked,model_or_implementation,model_path,elapsed_ms,input_json,output_json,note,evidence_path,occurred_at)
                   VALUES(?,?,?,?,?,?,?,?,?,?,?,?,?,?)""",
                (run_id,session_id,evidence.get("module_id"),evidence.get("module_name"),evidence.get("status"),int(bool(evidence.get("model_invoked"))),
                 evidence.get("model_or_implementation"),evidence.get("model_path"),evidence.get("elapsed_ms"),_json(inputs),_json(output),
                 evidence.get("note"),evidence.get("evidence_path"),evidence.get("time") or _now()),
            )
            for direction,value in (("INPUT",inputs),("OUTPUT",output)):
                for field_path,asset_type,file_path in self._assets(value):
                    self._conn.execute(
                        "INSERT OR IGNORE INTO module_io_asset(module_run_id,direction,field_path,asset_type,file_path) VALUES(?,?,?,?,?)",
                        (run_id,direction,field_path,asset_type,file_path),
                    )
                    if asset_type=="IMAGE" and cargo_id:
                        inventory_row=self._conn.execute(
                            "SELECT inventory_id FROM cargo_item WHERE session_id=? AND cargo_id=?",
                            (session_id,cargo_id),
                        ).fetchone()
                        if inventory_row:
                            inventory_id=str(inventory_row[0]); image_type=self._image_type(field_path,direction)
                            duplicate=self._conn.execute(
                                """SELECT 1 FROM cargo_image WHERE inventory_id=? AND session_id=? AND module_run_id=?
                                   AND direction=? AND image_path=? LIMIT 1""",
                                (inventory_id,session_id,run_id,direction,file_path),
                            ).fetchone()
                            if duplicate: continue
                            has_primary=self._conn.execute("SELECT 1 FROM cargo_image WHERE inventory_id=? AND is_primary=1 LIMIT 1",(inventory_id,)).fetchone()
                            is_primary=0 if has_primary else 1
                            self._conn.execute(
                                """INSERT OR IGNORE INTO cargo_image(inventory_id,session_id,cargo_id,module_run_id,direction,image_type,field_path,image_path,is_primary,created_at)
                                   VALUES(?,?,?,?,?,?,?,?,?,?)""",
                                (inventory_id,session_id,cargo_id,run_id,direction,image_type,field_path,file_path,is_primary,evidence.get("time") or _now()),
                            )
                            if is_primary:
                                self._conn.execute("UPDATE cargo_inventory SET primary_image_path=?,updated_at=? WHERE inventory_id=?",(file_path,_now(),inventory_id))

    def record_motion(self,session_id: str,robot_id: str,pose: Mapping[str,Any],task: str):
        if not session_id: return
        with self._lock,self._conn:
            self._conn.execute(
                "INSERT INTO device_motion(session_id,robot_id,task,x_mm,y_mm,z_mm,roll_deg,pitch_deg,yaw_deg,occurred_at) VALUES(?,?,?,?,?,?,?,?,?,?)",
                (session_id,robot_id,task,float(pose.get("x_mm",0) or 0),float(pose.get("y_mm",0) or 0),float(pose.get("z_mm",0) or 0),
                 float(pose.get("roll_deg",0) or 0),float(pose.get("pitch_deg",0) or 0),float(pose.get("yaw_deg",0) or 0),_now()),
            )

    def record_messages(self,session_id: str,messages: list[dict]):
        if not session_id or not messages: return
        with self._lock,self._conn:
            self._conn.executemany(
                "INSERT INTO plc_message(session_id,source,status,message,payload_json,occurred_at) VALUES(?,?,?,?,?,?)",
                [(session_id,m.get("source"),m.get("status"),m.get("message"),_json(m.get("data")),m.get("time") or _now()) for m in messages],
            )

    def status(self,session_id: str="") -> dict:
        with self._lock:
            tables=("vehicle","loading_session","cargo_inventory","cargo_item","cargo_image","loading_region","workflow_step_run","placement_record","module_run","module_io_asset","plc_message","device_motion")
            counts={name:int(self._conn.execute(f"SELECT COUNT(*) FROM {name}").fetchone()[0]) for name in tables}
            return {"enabled":True,"backend":self.backend,"location":self.location,"path":self.location,"session_id":session_id,"counts":counts}


def create_vehicle_database_service(config: Mapping[str,Any] | None=None) -> VehicleDatabaseService:
    config=dict(config or {})
    driver=str(os.environ.get("JUSHEN_DB_DRIVER") or config.get("driver") or "sqlite").strip().lower()
    if driver=="mysql":
        mysql_config=dict(config)
        mysql_config["host"]=os.environ.get("JUSHEN_MYSQL_HOST") or mysql_config.get("host") or "127.0.0.1"
        mysql_config["port"]=int(os.environ.get("JUSHEN_MYSQL_PORT") or mysql_config.get("port") or 3306)
        mysql_config["user"]=os.environ.get("JUSHEN_MYSQL_USER") or mysql_config.get("user") or "root"
        mysql_config["database"]=os.environ.get("JUSHEN_MYSQL_DATABASE") or mysql_config.get("database") or "jushenzhineng"
        return VehicleDatabaseService(mysql_config=mysql_config)
    db_path=Path(str(os.environ.get("JUSHEN_VEHICLE_DB_PATH") or config.get("path") or DEFAULT_DB_PATH))
    return VehicleDatabaseService(db_path)
