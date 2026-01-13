# SNAP_7 KNOWLEDGE BASE

**Type:** Python ROS 2 Node
**Protocol:** Siemens S7 (via python-snap7)

## OVERVIEW
Bridges ROS 2 and Siemens PLCs. Monitors PLC state to trigger vision tasks and writes back results (coordinates, status). Handles connection stability and error recovery.

## STRUCTURE
```
snap_7/
├── snap_7/
│   └── plc_client_node.py  # Main driver
├── launch/                 # Launch configuration
└── test/                   # Connection tests
```

## WHERE TO LOOK
| Task | Location | Notes |
|------|----------|-------|
| **Main Loop** | `plc_client_node.py` | `poll_plc` method |
| **DB Read** | `plc_client_node.py` | `read_db_area` |
| **DB Write** | `plc_client_node.py` | `write_real_to_db` / `write_db_area` |
| **State Machine** | `plc_client_node.py` | `cam_bringup`, `norm_bringup` |

## PLC MAPPING
- **Trigger**: DB address `110` (monitored).
- **Result**: Written to configured offsets (Real/Int).
- **Heartbeat**: Periodic read/write to ensure link.

## STATE MACHINE
1. **Connect**: Retries with backoff.
2. **Monitor**: Polls specific DB address.
3. **Trigger**: On value change (default: 110) -> Calls `norm_calc`.
4. **Response**: Formats ROS result -> Writes to PLC DB.
