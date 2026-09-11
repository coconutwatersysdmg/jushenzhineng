# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import json
import os
import sqlite3
from pathlib import Path

import pymysql

from config.system_config import get_system_config


PROJECT_ROOT=Path(__file__).resolve().parents[1]
DEFAULT_DB=PROJECT_ROOT/"runtime"/"vehicle_loading.db"


def _system_database_config():
    try: return dict(get_system_config().get("database") or {})
    except Exception: return {}


def _query_summary(conn,placeholder: str,truck_id: str):
    where=f" WHERE truck_id={placeholder}" if truck_id else ""
    params=(truck_id,) if truck_id else ()
    with conn.cursor() as cursor:
        cursor.execute("SELECT * FROM v_vehicle_loading_overview"+where+" ORDER BY started_at DESC",params)
        overview=list(cursor.fetchall())
        latest=overview[0]["session_id"] if overview else ""
        cursor.execute("SELECT * FROM v_cargo_inventory_status ORDER BY updated_at DESC")
        inventory=list(cursor.fetchall())
        regions=[]; modules=[]
        if latest:
            cursor.execute(
                f"SELECT * FROM v_vehicle_region_status WHERE session_id={placeholder} ORDER BY row_index,column_code",
                (latest,),
            )
            regions=list(cursor.fetchall())
            cursor.execute(
                f"SELECT truck_id,session_id,module_id,module_name,status,model_invoked,elapsed_ms,evidence_path,occurred_at "
                f"FROM v_vehicle_module_io WHERE session_id={placeholder} ORDER BY occurred_at",
                (latest,),
            )
            modules=list(cursor.fetchall())
    return overview,inventory,regions,modules


def main():
    config=_system_database_config()
    parser=argparse.ArgumentParser(description="按车辆查看装载数据库摘要")
    parser.add_argument("--driver",choices=("sqlite","mysql"),default=str(config.get("driver") or "sqlite"))
    parser.add_argument("--db",default=str(DEFAULT_DB))
    parser.add_argument("--truck-id",default="")
    parser.add_argument("--host",default=str(config.get("host") or "127.0.0.1"))
    parser.add_argument("--port",type=int,default=int(config.get("port") or 3306))
    parser.add_argument("--user",default=str(config.get("user") or "root"))
    parser.add_argument("--database",default=str(config.get("database") or "jushenzhineng"))
    args=parser.parse_args()
    if args.driver=="mysql":
        password=str(os.environ.get(str(config.get("password_env") or "JUSHEN_MYSQL_PASSWORD")) or config.get("password") or "")
        conn=pymysql.connect(host=args.host,port=args.port,user=args.user,password=password,database=args.database,
                             charset="utf8mb4",cursorclass=pymysql.cursors.DictCursor)
        try: overview,inventory,regions,modules=_query_summary(conn,"%s",args.truck_id)
        finally: conn.close()
        location=f"mysql://{args.host}:{args.port}/{args.database}"
    else:
        db_path=Path(args.db).resolve()
        if not db_path.is_file(): raise SystemExit(f"数据库不存在：{db_path}")
        raw=sqlite3.connect(f"file:{db_path.as_posix()}?mode=ro",uri=True); raw.row_factory=sqlite3.Row
        class SQLiteCursor:
            def __init__(self,connection): self.connection=connection; self._cursor=None
            def __enter__(self): return self
            def __exit__(self,*_): return False
            def execute(self,sql,params=()): self._cursor=self.connection.execute(sql,params); return self
            def fetchall(self): return [dict(row) for row in self._cursor.fetchall()]
        class SQLiteInspectConnection:
            def __init__(self,connection): self.connection=connection
            def cursor(self): return SQLiteCursor(self.connection)
        try: overview,inventory,regions,modules=_query_summary(SQLiteInspectConnection(raw),"?",args.truck_id)
        finally: raw.close()
        location=str(db_path)
    print(json.dumps({"database":location,"overview":overview,"inventory":inventory,"latest_session_regions":regions,"latest_session_modules":modules},ensure_ascii=False,indent=2,default=str))


if __name__=="__main__": main()
