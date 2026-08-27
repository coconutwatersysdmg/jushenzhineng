PRAGMA foreign_keys = ON;

CREATE TABLE IF NOT EXISTS vehicle (
    truck_id TEXT PRIMARY KEY,
    plate_no TEXT,
    vehicle_type TEXT,
    coordinate_frame TEXT NOT NULL DEFAULT 'world',
    length_mm REAL,
    width_mm REAL,
    deck_height_mm REAL,
    attributes_json TEXT NOT NULL DEFAULT '{}',
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS loading_session (
    session_id TEXT PRIMARY KEY,
    truck_id TEXT NOT NULL REFERENCES vehicle(truck_id),
    task_code TEXT,
    status TEXT NOT NULL,
    plan_json TEXT NOT NULL,
    started_at TEXT NOT NULL,
    finished_at TEXT,
    last_step_code TEXT,
    last_step_status TEXT,
    error_message TEXT
);
CREATE INDEX IF NOT EXISTS idx_loading_session_truck_time ON loading_session(truck_id, started_at DESC);

CREATE TABLE IF NOT EXISTS cargo_inventory (
    inventory_id TEXT PRIMARY KEY,
    cargo_code TEXT,
    cargo_name TEXT,
    batch_no TEXT,
    length_mm REAL,
    width_mm REAL,
    height_mm REAL,
    inventory_status TEXT NOT NULL DEFAULT 'AVAILABLE',
    is_loaded INTEGER NOT NULL DEFAULT 0 CHECK(is_loaded IN (0,1)),
    current_session_id TEXT REFERENCES loading_session(session_id),
    loaded_truck_id TEXT REFERENCES vehicle(truck_id),
    loaded_session_id TEXT REFERENCES loading_session(session_id),
    loaded_region_id TEXT,
    loaded_at TEXT,
    primary_image_path TEXT,
    attributes_json TEXT NOT NULL DEFAULT '{}',
    created_at TEXT NOT NULL,
    updated_at TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_inventory_status ON cargo_inventory(inventory_status, is_loaded);
CREATE INDEX IF NOT EXISTS idx_inventory_loaded_truck ON cargo_inventory(loaded_truck_id, loaded_at DESC);

CREATE TABLE IF NOT EXISTS cargo_item (
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    cargo_id TEXT NOT NULL,
    inventory_id TEXT REFERENCES cargo_inventory(inventory_id),
    cargo_code TEXT,
    cargo_name TEXT,
    sequence_no INTEGER,
    length_mm REAL,
    width_mm REAL,
    height_mm REAL,
    status TEXT,
    attached_to TEXT,
    pose_json TEXT NOT NULL DEFAULT '{}',
    attributes_json TEXT NOT NULL DEFAULT '{}',
    updated_at TEXT NOT NULL,
    PRIMARY KEY (session_id, cargo_id)
);
CREATE INDEX IF NOT EXISTS idx_cargo_session_status ON cargo_item(session_id, status);

CREATE TABLE IF NOT EXISTS truck_board_snapshot (
    board_snapshot_id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    round_no INTEGER NOT NULL,
    board_mode TEXT,
    decision_source TEXT,
    corners_json TEXT NOT NULL,
    geometry_json TEXT NOT NULL,
    created_at TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS loading_region (
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    region_id TEXT NOT NULL,
    blind_code TEXT,
    section_name TEXT,
    column_code TEXT,
    row_index INTEGER,
    center_x_mm REAL,
    center_y_mm REAL,
    center_z_mm REAL,
    corners_json TEXT NOT NULL DEFAULT '[]',
    status TEXT NOT NULL,
    cargo_id TEXT,
    requires_support_block INTEGER NOT NULL DEFAULT 0,
    support_height_mm REAL NOT NULL DEFAULT 0,
    updated_at TEXT NOT NULL,
    PRIMARY KEY (session_id, region_id)
);
CREATE INDEX IF NOT EXISTS idx_region_session_position ON loading_region(session_id, row_index, column_code);
CREATE INDEX IF NOT EXISTS idx_region_session_status ON loading_region(session_id, status);

CREATE TABLE IF NOT EXISTS workflow_step_run (
    step_run_id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    round_no INTEGER NOT NULL,
    cargo_id TEXT,
    step_code TEXT NOT NULL,
    step_name TEXT,
    status TEXT NOT NULL,
    message TEXT,
    result_json TEXT NOT NULL DEFAULT '{}',
    occurred_at TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_step_session_time ON workflow_step_run(session_id, step_run_id);

CREATE TABLE IF NOT EXISTS placement_record (
    placement_id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    round_no INTEGER NOT NULL,
    cargo_id TEXT NOT NULL,
    region_id TEXT,
    selected_side TEXT,
    target_pose_json TEXT NOT NULL,
    actual_pose_json TEXT NOT NULL,
    plc_ack_json TEXT NOT NULL DEFAULT '{}',
    status TEXT NOT NULL,
    placed_at TEXT NOT NULL,
    UNIQUE(session_id, cargo_id)
);

CREATE TABLE IF NOT EXISTS module_run (
    module_run_id TEXT PRIMARY KEY,
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    module_id TEXT NOT NULL,
    module_name TEXT,
    status TEXT NOT NULL,
    model_invoked INTEGER NOT NULL DEFAULT 0,
    model_or_implementation TEXT,
    model_path TEXT,
    elapsed_ms REAL,
    input_json TEXT NOT NULL,
    output_json TEXT NOT NULL,
    note TEXT,
    evidence_path TEXT,
    occurred_at TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_module_session_time ON module_run(session_id, occurred_at);
CREATE INDEX IF NOT EXISTS idx_module_vehicle_lookup ON module_run(module_id, occurred_at);

CREATE TABLE IF NOT EXISTS module_io_asset (
    asset_id INTEGER PRIMARY KEY AUTOINCREMENT,
    module_run_id TEXT NOT NULL REFERENCES module_run(module_run_id) ON DELETE CASCADE,
    direction TEXT NOT NULL CHECK(direction IN ('INPUT','OUTPUT')),
    field_path TEXT NOT NULL,
    asset_type TEXT NOT NULL,
    file_path TEXT NOT NULL,
    UNIQUE(module_run_id, direction, field_path, file_path)
);

CREATE TABLE IF NOT EXISTS cargo_image (
    cargo_image_id INTEGER PRIMARY KEY AUTOINCREMENT,
    inventory_id TEXT NOT NULL REFERENCES cargo_inventory(inventory_id) ON DELETE CASCADE,
    session_id TEXT REFERENCES loading_session(session_id) ON DELETE SET NULL,
    cargo_id TEXT,
    module_run_id TEXT REFERENCES module_run(module_run_id) ON DELETE SET NULL,
    direction TEXT NOT NULL CHECK(direction IN ('INPUT','OUTPUT')),
    image_type TEXT NOT NULL,
    field_path TEXT NOT NULL,
    image_path TEXT NOT NULL,
    is_primary INTEGER NOT NULL DEFAULT 0 CHECK(is_primary IN (0,1)),
    created_at TEXT NOT NULL,
    UNIQUE(inventory_id, session_id, module_run_id, direction, field_path, image_path)
);
CREATE INDEX IF NOT EXISTS idx_cargo_image_inventory ON cargo_image(inventory_id, created_at DESC);
CREATE INDEX IF NOT EXISTS idx_cargo_image_session ON cargo_image(session_id, cargo_id);

CREATE TABLE IF NOT EXISTS plc_message (
    plc_message_id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    source TEXT,
    status TEXT,
    message TEXT,
    payload_json TEXT NOT NULL DEFAULT '{}',
    occurred_at TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_plc_session_time ON plc_message(session_id, plc_message_id);

CREATE TABLE IF NOT EXISTS device_motion (
    motion_id INTEGER PRIMARY KEY AUTOINCREMENT,
    session_id TEXT NOT NULL REFERENCES loading_session(session_id) ON DELETE CASCADE,
    robot_id TEXT NOT NULL,
    task TEXT,
    x_mm REAL NOT NULL,
    y_mm REAL NOT NULL,
    z_mm REAL NOT NULL,
    roll_deg REAL NOT NULL,
    pitch_deg REAL NOT NULL,
    yaw_deg REAL NOT NULL,
    occurred_at TEXT NOT NULL
);
CREATE INDEX IF NOT EXISTS idx_motion_session_robot_time ON device_motion(session_id, robot_id, motion_id);

DROP VIEW IF EXISTS v_vehicle_loading_overview;
CREATE VIEW v_vehicle_loading_overview AS
SELECT
    s.truck_id,
    s.session_id,
    s.status AS session_status,
    s.started_at,
    s.finished_at,
    COUNT(DISTINCT c.cargo_id) AS cargo_total,
    COUNT(DISTINCT CASE WHEN c.status = 'PLACED' THEN c.cargo_id END) AS cargo_placed,
    COUNT(DISTINCT p.placement_id) AS placement_count,
    COUNT(DISTINCT m.module_run_id) AS module_run_count
FROM loading_session s
LEFT JOIN cargo_item c ON c.session_id = s.session_id
LEFT JOIN placement_record p ON p.session_id = s.session_id
LEFT JOIN module_run m ON m.session_id = s.session_id
GROUP BY s.truck_id, s.session_id, s.status, s.started_at, s.finished_at;

DROP VIEW IF EXISTS v_vehicle_region_status;
CREATE VIEW v_vehicle_region_status AS
SELECT s.truck_id, r.session_id, r.blind_code, r.section_name, r.column_code,
       r.row_index, r.center_x_mm, r.center_y_mm, r.center_z_mm,
       r.status, r.cargo_id, r.updated_at
FROM loading_region r
JOIN loading_session s ON s.session_id = r.session_id;

DROP VIEW IF EXISTS v_vehicle_module_io;
CREATE VIEW v_vehicle_module_io AS
SELECT s.truck_id, m.session_id, m.module_run_id, m.module_id, m.module_name,
       m.status, m.model_invoked, m.elapsed_ms, m.input_json, m.output_json,
       m.evidence_path, m.occurred_at
FROM module_run m
JOIN loading_session s ON s.session_id = m.session_id;

DROP VIEW IF EXISTS v_cargo_inventory_status;
CREATE VIEW v_cargo_inventory_status AS
SELECT i.inventory_id, i.cargo_code, i.cargo_name, i.batch_no,
       i.inventory_status, i.is_loaded, i.current_session_id,
       i.loaded_truck_id, i.loaded_session_id, i.loaded_region_id,
       i.loaded_at, i.primary_image_path,
       COUNT(img.cargo_image_id) AS image_count,
       i.updated_at
FROM cargo_inventory i
LEFT JOIN cargo_image img ON img.inventory_id = i.inventory_id
GROUP BY i.inventory_id, i.cargo_code, i.cargo_name, i.batch_no,
         i.inventory_status, i.is_loaded, i.current_session_id,
         i.loaded_truck_id, i.loaded_session_id, i.loaded_region_id,
         i.loaded_at, i.primary_image_path, i.updated_at;
