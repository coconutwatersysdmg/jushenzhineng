# -*- coding: utf-8 -*-
from __future__ import annotations

import argparse
import getpass
import os
import re
import sqlite3
from pathlib import Path

import pymysql
from pymysql.constants import CLIENT


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_SQLITE = PROJECT_ROOT / "workdir" / "vehicle_loading.db"
MYSQL_SCHEMA = PROJECT_ROOT / "database" / "vehicle_centered_schema_mysql.sql"

TABLES = (
    "vehicle",
    "loading_session",
    "cargo_inventory",
    "cargo_item",
    "truck_board_snapshot",
    "loading_region",
    "workflow_step_run",
    "placement_record",
    "module_run",
    "module_io_asset",
    "cargo_image",
    "plc_message",
    "device_motion",
)
VIEWS = (
    "v_vehicle_loading_overview",
    "v_vehicle_region_status",
    "v_vehicle_module_io",
    "v_cargo_inventory_status",
)
IDENTIFIER = re.compile(r"^[A-Za-z0-9_]+$")


def connection_args(args: argparse.Namespace, password: str, database: str | None = None) -> dict:
    options = {
        "host": args.host,
        "port": args.port,
        "user": args.user,
        "password": password,
        "charset": "utf8mb4",
        "autocommit": False,
        "connect_timeout": 8,
        "client_flag": CLIENT.MULTI_STATEMENTS,
    }
    if database:
        options["database"] = database
    return options


def execute_schema(conn: pymysql.Connection) -> None:
    sql = MYSQL_SCHEMA.read_text(encoding="utf-8")
    with conn.cursor() as cursor:
        cursor.execute(sql)
        while cursor.nextset():
            pass
    conn.commit()


def sqlite_columns(conn: sqlite3.Connection, table: str) -> list[str]:
    return [str(row[1]) for row in conn.execute(f"PRAGMA table_info(`{table}`)")]


def mysql_table_count(conn: pymysql.Connection, table: str) -> int:
    with conn.cursor() as cursor:
        cursor.execute(f"SELECT COUNT(*) FROM `{table}`")
        return int(cursor.fetchone()[0])


def migrate(args: argparse.Namespace, password: str) -> dict:
    sqlite_path = Path(args.sqlite).resolve()
    if not sqlite_path.is_file():
        raise SystemExit(f"SQLite 数据库不存在：{sqlite_path}")
    if not IDENTIFIER.fullmatch(args.database):
        raise SystemExit("MySQL 数据库名只能包含字母、数字和下划线")

    server = pymysql.connect(**connection_args(args, password))
    try:
        with server.cursor() as cursor:
            cursor.execute(
                f"CREATE DATABASE IF NOT EXISTS `{args.database}` "
                "CHARACTER SET utf8mb4 COLLATE utf8mb4_0900_ai_ci"
            )
        server.commit()
    finally:
        server.close()

    source = sqlite3.connect(f"file:{sqlite_path.as_posix()}?mode=ro", uri=True)
    source.row_factory = sqlite3.Row
    target = pymysql.connect(**connection_args(args, password, args.database))
    try:
        execute_schema(target)
        existing = {table: mysql_table_count(target, table) for table in TABLES}
        occupied = {table: count for table, count in existing.items() if count}
        if occupied and not args.replace:
            detail = ", ".join(f"{name}={count}" for name, count in occupied.items())
            raise SystemExit(f"目标数据库已有数据（{detail}）；如需覆盖请显式添加 --replace")

        with target.cursor() as cursor:
            cursor.execute("SET FOREIGN_KEY_CHECKS=0")
            if args.replace:
                for table in reversed(TABLES):
                    cursor.execute(f"DELETE FROM `{table}`")

            copied: dict[str, int] = {}
            for table in TABLES:
                columns = sqlite_columns(source, table)
                rows = source.execute(f"SELECT * FROM `{table}`").fetchall()
                copied[table] = len(rows)
                if not rows:
                    continue
                column_sql = ", ".join(f"`{column}`" for column in columns)
                placeholders = ", ".join(["%s"] * len(columns))
                cursor.executemany(
                    f"INSERT INTO `{table}` ({column_sql}) VALUES ({placeholders})",
                    [tuple(row[column] for column in columns) for row in rows],
                )
            cursor.execute("SET FOREIGN_KEY_CHECKS=1")
        target.commit()

        target_counts = {table: mysql_table_count(target, table) for table in TABLES}
        mismatches = {
            table: {"sqlite": copied[table], "mysql": target_counts[table]}
            for table in TABLES
            if copied[table] != target_counts[table]
        }
        with target.cursor() as cursor:
            cursor.execute(
                "SELECT COUNT(*) FROM information_schema.REFERENTIAL_CONSTRAINTS "
                "WHERE CONSTRAINT_SCHEMA=%s",
                (args.database,),
            )
            foreign_keys = int(cursor.fetchone()[0])
            cursor.execute(
                "SELECT TABLE_NAME FROM information_schema.VIEWS "
                "WHERE TABLE_SCHEMA=%s ORDER BY TABLE_NAME",
                (args.database,),
            )
            views = [str(row[0]) for row in cursor.fetchall()]
        if mismatches:
            raise RuntimeError(f"迁移后记录数不一致：{mismatches}")
        if sorted(views) != sorted(VIEWS):
            raise RuntimeError(f"视图校验失败：{views}")
        return {
            "mysql": f"{args.user}@{args.host}:{args.port}/{args.database}",
            "sqlite": str(sqlite_path),
            "table_counts": target_counts,
            "foreign_key_count": foreign_keys,
            "views": views,
        }
    except BaseException:
        target.rollback()
        raise
    finally:
        source.close()
        target.close()


def main() -> None:
    parser = argparse.ArgumentParser(description="将车辆装载 SQLite 数据迁移到 MySQL 8")
    parser.add_argument("--sqlite", default=str(DEFAULT_SQLITE))
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=3306)
    parser.add_argument("--user", default="root")
    parser.add_argument("--database", default="jushenzhineng")
    parser.add_argument("--replace", action="store_true", help="清空目标业务表后重新迁移")
    args = parser.parse_args()
    password = os.environ.get("JUSHEN_MYSQL_PASSWORD")
    if password is None:
        password = getpass.getpass(f"MySQL password for {args.user}@{args.host}: ")
    result = migrate(args, password)
    print("MYSQL_MIGRATION_OK")
    print(f"target={result['mysql']}")
    print(f"source={result['sqlite']}")
    for table, count in result["table_counts"].items():
        print(f"{table}={count}")
    print(f"foreign_keys={result['foreign_key_count']}")
    print("views=" + ",".join(result["views"]))


if __name__ == "__main__":
    main()
