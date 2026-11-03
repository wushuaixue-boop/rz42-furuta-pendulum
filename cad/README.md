# CAD

`step/`: every part as a single solid in STEP AP214, units mm, in its own part coordinates.
Open in SolidWorks (they import as solids; use FeatureWorks for features), Fusion 360 or FreeCAD.

| File | Part | Qty | Type |
|---|---|---|---|
| RZ42-001_Base | base (R56.5 circle ∩ 105 mm square, 85 mm high, 4 heat-insert bosses) | 1 | print |
| RZ42-002_Lid | lid, 3 mm, motor pilot Ø22.4, 8 M3 countersunk holes, Ø6 cable hole | 1 | print |
| RZ42-003_Arm_608 | arm, D-shaft hub (set screw + insert), AS5600 slot, two inserts for the housing | 1 | print |
| RZ42-004_Pendulum_608 | pendulum, Ø8.1 journal + shoulder, magnet pocket, coin slot (pause the print to insert the coin) | 1 | print |
| RZ42-005_BearingHousing_608 | 608ZZ housing, Ø22.1 seat, 2 × M3 countersunk into the arm | 1 | print |
| RZ42-101_Motor_42x48 | ZDT 42×48 closed-loop stepper (Ø5 D-shaft, 24 mm) | 1 | bought |
| RZ42-102_Driver_ZDT-Y42-V2.0 | ZDT Y42 V2.0 driver board (outline and holes from the vendor DWG) | 1 | bought |
| RZ42-103_Battery_70x55x20 | battery pack envelope | 1 | bought |
| RZ42-104_Bearing_608 | 608ZZ (8×22×7), shielded, not sealed | 1 | bought |
| RZ42-105_Magnet_D6x2.5 | diametric NdFeB magnet for the AS5600 | 1 | bought |
| RZ42-106_HeatInsert_M3x5x4 | M3×5×4 heat insert | 7 | bought |
| RZ42-107_Encoder_AS5600_Module | AS5600 module envelope (23×23×1.6) | 1 | bought |

Fasteners: M3×8 countersunk ×6, M3×6 ×4 (motor), M3×12 set screw ×1.

**Assembly reference (world frame: origin at the base bottom centre, +z up, motor axis = z, arm at 0° along −y):**
* lid underside at z = 85;
* motor flange at z = 85;
* arm underside at z = 91;
* pendulum axis at z = 105, 62 mm from the motor axis.

**Motion check:** arm every 5° × pendulum every 15° (5,328 pose pairs) with the 608 version gave zero interference.

`generator/zdt_v5.cpp` builds all parts with OpenCASCADE 7.x and writes the STEP files, including the interference sweep. Build it against the OpenCASCADE `TK*` libraries (modeling, boolean, STEP and XCAF toolkits).
