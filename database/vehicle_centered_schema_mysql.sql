SET NAMES utf8mb4;
SET FOREIGN_KEY_CHECKS = 1;

CREATE TABLE IF NOT EXISTS vehicle (
    truck_id VARCHAR(128) PRIMARY KEY,
    plate_no VARCHAR(128),
    vehicle_type VARCHAR(128),
    coordinate_frame VARCHAR(64) NOT NULL DEFAULT 'world',
    length_mm DOUBLE,
    width_mm DOUBLE,
    deck_height_mm DOUBLE,
    attributes_json JSON NOT NULL,
    created_at DATETIME(3) NOT NULL,
    updated_at DATETIME(3) NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS loading_session (
    session_id VARCHAR(128) PRIMARY KEY,
    truck_id VARCHAR(128) NOT NULL,
    task_code VARCHAR(255),
    status VARCHAR(64) NOT NULL,
    plan_json JSON NOT NULL,
    started_at DATETIME(3) NOT NULL,
    finished_at DATETIME(3),
    last_step_code VARCHAR(128),
    last_step_status VARCHAR(64),
    error_message TEXT,
    KEY idx_loading_session_truck_time (truck_id, started_at DESC),
    CONSTRAINT fk_session_vehicle FOREIGN KEY (truck_id) REFERENCES vehicle(truck_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS cargo_inventory (
    inventory_id VARCHAR(128) PRIMARY KEY,
    cargo_code VARCHAR(128),
    cargo_name VARCHAR(255),
    batch_no VARCHAR(128),
    length_mm DOUBLE,
    width_mm DOUBLE,
    height_mm DOUBLE,
    inventory_status VARCHAR(64) NOT NULL DEFAULT 'AVAILABLE',
    is_loaded TINYINT(1) NOT NULL DEFAULT 0,
    current_session_id VARCHAR(128),
    loaded_truck_id VARCHAR(128),
    loaded_session_id VARCHAR(128),
    loaded_region_id VARCHAR(128),
    loaded_at DATETIME(3),
    primary_image_path TEXT,
    attributes_json JSON NOT NULL,
    created_at DATETIME(3) NOT NULL,
    updated_at DATETIME(3) NOT NULL,
    CONSTRAINT chk_inventory_is_loaded CHECK (is_loaded IN (0, 1)),
    KEY idx_inventory_status (inventory_status, is_loaded),
    KEY idx_inventory_loaded_truck (loaded_truck_id, loaded_at DESC),
    KEY idx_inventory_current_session (current_session_id),
    KEY idx_inventory_loaded_session (loaded_session_id),
    CONSTRAINT fk_inventory_current_session FOREIGN KEY (current_session_id) REFERENCES loading_session(session_id),
    CONSTRAINT fk_inventory_loaded_vehicle FOREIGN KEY (loaded_truck_id) REFERENCES vehicle(truck_id),
    CONSTRAINT fk_inventory_loaded_session FOREIGN KEY (loaded_session_id) REFERENCES loading_session(session_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS cargo_item (
    session_id VARCHAR(128) NOT NULL,
    cargo_id VARCHAR(128) NOT NULL,
    inventory_id VARCHAR(128),
    cargo_code VARCHAR(128),
    cargo_name VARCHAR(255),
    sequence_no INT,
    length_mm DOUBLE,
    width_mm DOUBLE,
    height_mm DOUBLE,
    status VARCHAR(64),
    attached_to VARCHAR(128),
    pose_json JSON NOT NULL,
    attributes_json JSON NOT NULL,
    updated_at DATETIME(3) NOT NULL,
    PRIMARY KEY (session_id, cargo_id),
    KEY idx_cargo_session_status (session_id, status),
    KEY idx_cargo_inventory (inventory_id),
    CONSTRAINT fk_cargo_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE,
    CONSTRAINT fk_cargo_inventory FOREIGN KEY (inventory_id) REFERENCES cargo_inventory(inventory_id)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS truck_board_snapshot (
    board_snapshot_id BIGINT PRIMARY KEY AUTO_INCREMENT,
    session_id VARCHAR(128) NOT NULL,
    round_no INT NOT NULL,
    board_mode VARCHAR(128),
    decision_source VARCHAR(255),
    corners_json JSON NOT NULL,
    geometry_json JSON NOT NULL,
    created_at DATETIME(3) NOT NULL,
    KEY idx_board_snapshot_session (session_id),
    CONSTRAINT fk_board_snapshot_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS loading_region (
    session_id VARCHAR(128) NOT NULL,
    region_id VARCHAR(128) NOT NULL,
    blind_code VARCHAR(128),
    section_name VARCHAR(128),
    column_code VARCHAR(32),
    row_index INT,
    center_x_mm DOUBLE,
    center_y_mm DOUBLE,
    center_z_mm DOUBLE,
    corners_json JSON NOT NULL,
    status VARCHAR(64) NOT NULL,
    cargo_id VARCHAR(128),
    requires_support_block TINYINT(1) NOT NULL DEFAULT 0,
    support_height_mm DOUBLE NOT NULL DEFAULT 0,
    updated_at DATETIME(3) NOT NULL,
    PRIMARY KEY (session_id, region_id),
    KEY idx_region_session_position (session_id, row_index, column_code),
    KEY idx_region_session_status (session_id, status),
    CONSTRAINT chk_region_support CHECK (requires_support_block IN (0, 1)),
    CONSTRAINT fk_region_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS workflow_step_run (
    step_run_id BIGINT PRIMARY KEY AUTO_INCREMENT,
    session_id VARCHAR(128) NOT NULL,
    round_no INT NOT NULL,
    cargo_id VARCHAR(128),
    step_code VARCHAR(128) NOT NULL,
    step_name VARCHAR(255),
    status VARCHAR(64) NOT NULL,
    message TEXT,
    result_json JSON NOT NULL,
    occurred_at DATETIME(3) NOT NULL,
    KEY idx_step_session_time (session_id, step_run_id),
    CONSTRAINT fk_step_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS placement_record (
    placement_id BIGINT PRIMARY KEY AUTO_INCREMENT,
    session_id VARCHAR(128) NOT NULL,
    round_no INT NOT NULL,
    cargo_id VARCHAR(128) NOT NULL,
    region_id VARCHAR(128),
    selected_side VARCHAR(32),
    target_pose_json JSON NOT NULL,
    actual_pose_json JSON NOT NULL,
    plc_ack_json JSON NOT NULL,
    status VARCHAR(64) NOT NULL,
    placed_at DATETIME(3) NOT NULL,
    UNIQUE KEY uq_placement_session_cargo (session_id, cargo_id),
    CONSTRAINT fk_placement_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS module_run (
    module_run_id VARCHAR(128) PRIMARY KEY,
    session_id VARCHAR(128) NOT NULL,
    module_id VARCHAR(128) NOT NULL,
    module_name VARCHAR(255),
    status VARCHAR(64) NOT NULL,
    model_invoked TINYINT(1) NOT NULL DEFAULT 0,
    model_or_implementation VARCHAR(255),
    model_path TEXT,
    elapsed_ms DOUBLE,
    input_json JSON NOT NULL,
    output_json JSON NOT NULL,
    note TEXT,
    evidence_path TEXT,
    occurred_at DATETIME(3) NOT NULL,
    CONSTRAINT chk_module_invoked CHECK (model_invoked IN (0, 1)),
    KEY idx_module_session_time (session_id, occurred_at),
    KEY idx_module_vehicle_lookup (module_id, occurred_at),
    CONSTRAINT fk_module_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS module_io_asset (
    asset_id BIGINT PRIMARY KEY AUTO_INCREMENT,
    module_run_id VARCHAR(128) NOT NULL,
    direction VARCHAR(16) NOT NULL,
    field_path TEXT NOT NULL,
    asset_type VARCHAR(64) NOT NULL,
    file_path TEXT NOT NULL,
    CONSTRAINT chk_asset_direction CHECK (direction IN ('INPUT', 'OUTPUT')),
    UNIQUE KEY uq_module_asset (module_run_id, direction, field_path(128), file_path(256)),
    CONSTRAINT fk_asset_module FOREIGN KEY (module_run_id) REFERENCES module_run(module_run_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS cargo_image (
    cargo_image_id BIGINT PRIMARY KEY AUTO_INCREMENT,
    inventory_id VARCHAR(128) NOT NULL,
    session_id VARCHAR(128),
    cargo_id VARCHAR(128),
    module_run_id VARCHAR(128),
    direction VARCHAR(16) NOT NULL,
    image_type VARCHAR(64) NOT NULL,
    field_path TEXT NOT NULL,
    image_path TEXT NOT NULL,
    is_primary TINYINT(1) NOT NULL DEFAULT 0,
    created_at DATETIME(3) NOT NULL,
    CONSTRAINT chk_cargo_image_direction CHECK (direction IN ('INPUT', 'OUTPUT')),
    CONSTRAINT chk_cargo_image_primary CHECK (is_primary IN (0, 1)),
    UNIQUE KEY uq_cargo_image (inventory_id, session_id, module_run_id, direction, field_path(64), image_path(128)),
    KEY idx_cargo_image_inventory (inventory_id, created_at DESC),
    KEY idx_cargo_image_session (session_id, cargo_id),
    KEY idx_cargo_image_module (module_run_id),
    CONSTRAINT fk_cargo_image_inventory FOREIGN KEY (inventory_id) REFERENCES cargo_inventory(inventory_id) ON DELETE CASCADE,
    CONSTRAINT fk_cargo_image_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE SET NULL,
    CONSTRAINT fk_cargo_image_module FOREIGN KEY (module_run_id) REFERENCES module_run(module_run_id) ON DELETE SET NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS plc_message (
    plc_message_id BIGINT PRIMARY KEY AUTO_INCREMENT,
    session_id VARCHAR(128) NOT NULL,
    source VARCHAR(128),
    status VARCHAR(64),
    message TEXT,
    payload_json JSON NOT NULL,
    occurred_at DATETIME(3) NOT NULL,
    KEY idx_plc_session_time (session_id, plc_message_id),
    CONSTRAINT fk_plc_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

CREATE TABLE IF NOT EXISTS device_motion (
    motion_id BIGINT PRIMARY KEY AUTO_INCREMENT,
    session_id VARCHAR(128) NOT NULL,
    robot_id VARCHAR(128) NOT NULL,
    task VARCHAR(255),
    x_mm DOUBLE NOT NULL,
    y_mm DOUBLE NOT NULL,
    z_mm DOUBLE NOT NULL,
    roll_deg DOUBLE NOT NULL,
    pitch_deg DOUBLE NOT NULL,
    yaw_deg DOUBLE NOT NULL,
    occurred_at DATETIME(3) NOT NULL,
    KEY idx_motion_session_robot_time (session_id, robot_id, motion_id),
    CONSTRAINT fk_motion_session FOREIGN KEY (session_id) REFERENCES loading_session(session_id) ON DELETE CASCADE
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4 COLLATE=utf8mb4_0900_ai_ci;

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
