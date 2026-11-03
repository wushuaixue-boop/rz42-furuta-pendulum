// v2 (2026-11-02): post removed, M3x5x4 heat-set inserts, new arm (D-bore + set screw, 608 seat,
// AS5600 slot) and pendulum (608 journal, magnet pocket, coin pocket).
// Parametric B-rep model of the rotary-inverted-pendulum enclosure re-designed
// for a ZDT 42 closed-loop stepper (42x42x48 body + ZDT Y42 V2.0 driver board on
// the back) and a 70x55x20 battery. Built with OpenCASCADE, exported as STEP
// (AP214, mm) for SolidWorks and STL for printing.
//
// Frame (assembly / base): origin = centre of the base underside, +z up.
// The arm and pendulum are the ORIGINAL repo parts (unchanged, STL only).

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Common.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepFilletAPI_MakeFillet.hxx>
#include <BRepFilletAPI_MakeChamfer.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <BRep_Tool.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GC_MakeSegment.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <StlAPI_Writer.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <Interface_Static.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <TDataStd_Name.hxx>
#include <TopExp_Explorer.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopTools_ListOfShape.hxx>
#include <TopLoc_Location.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <Quantity_Color.hxx>
#include <gp.hxx>
#include <BRepClass3d_SolidClassifier.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <BRepBuilderAPI_MakeVertex.hxx>
#include <fstream>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>
#include <map>

// ============================================================ PARAMETERS (mm)
// --- motor (ZDT 42 closed-loop; NEMA17 front face standard)
static const double MOTOR_W       = 42.3;   // body square
static const double MOTOR_L       = 48.0;   // body length (user: 42x48)
static const double MOTOR_CHAMF   = 4.0;    // body corner chamfer
static const double MOTOR_BOSS_D  = 22.0;   // pilot boss
static const double MOTOR_BOSS_H  = 2.0;
static const double SHAFT_D       = 5.0;
static const double SHAFT_L       = 24.0;   // from flange face (check your motor!)
static const double HOLE_PITCH    = 31.0;   // M3 face holes
// --- ZDT Y42 V2.0 board (from ZDT_Y42_V2.0.DWG; DWG origin offset 46.9)
static const double PCB_T         = 1.6;
static const double PCB_GAP       = 9.0;    // motor rear face -> board (component side faces motor)
// --- battery (user: 70 x 55 x 20), lying flat
static const double BAT_X = 70.0, BAT_Y = 55.0, BAT_Z = 20.0;
// --- enclosure: footprint = circle(R) ∩ square(2*HALF). The hanging pendulum
// sweeps an annulus r = 59..65 mm around the motor axis below the lid, so every
// part of the base must stay inside r < 59 (original square corners reach 60).
static const double WALL      = 2.0;
static const double FLOOR     = 3.0;
static const double ENC_R     = 56.5;       // outer radius (2.5 mm clear of the pendulum)
static const double ENC_HALF  = 52.5;       // outer half-width of the 4 flats (flat width ~42)
static const double BOX_W     = 2 * ENC_HALF;
static const double BAT_GAP   = 3.0;        // battery top -> board underside
static const double BOX_H     = FLOOR + 20.0 + BAT_GAP + 1.6 /*pcb*/ + 9.0 /*gap*/ + 48.0 /*motor*/ + 0.4;  // = 85
static const double LID_T     = 3.0;        // lid plate (original 2.0; +1 for motor screws)
static const double LIP_H     = 4.0, LIP_T = 2.0, LIP_CLEAR = 0.2;
static const double BOSS_R    = 4.5;
static const double BOSS_XY   = (ENC_R - WALL - BOSS_R + 0.8) / std::sqrt(2.0);   // on the diagonals
// heat-set insert M3 x OD5 x L4 (the only threads used in printed parts)
static const double INS_HOLE_D = 4.2, INS_HOLE_DEPTH = 5.0;   // hole for the insert (L4 + 1 mm for displaced plastic)
static const double INS_TIP_D  = 3.2, INS_TIP_DEPTH = 5.0;    // screw-tip clearance below the insert
static const double CLR_D     = 3.4;        // M3 clearance
static const double CSK_D     = 6.4;        // 90 deg countersink dia
static const double CENTER_HOLE_D = 22.4;   // for motor boss
static const double WIRE_HOLE_D = 6.0, WIRE_HOLE_Y = 28.0;          // encoder wires into the box (post removed)
// battery bay (battery lies flat on the floor under the motor: 70 along x, 55 along y)
static const double BAT_CLR   = 0.5;
static const double STOP_T    = 2.0, STOP_H = 8.0;
// openings
static const double CABLE_W   = 16.0, CABLE_H = 10.0;                 // +x flat, at the XH8 plug height
static const double CHG_W     = 14.0, CHG_H = 7.0, CHG_Z = 13.0;      // +x flat, battery charge lead
static const double SWITCH_D  = 20.2, SWITCH_Z = 60.0;                // -x flat, 20 mm round rocker
static const double VENT_W    = 2.5,  VENT_Z0 = 45.0, VENT_Z1 = 75.0; // +-y flats

// derived
static const double IN_HALF   = ENC_HALF - WALL;
static const double LID_Z     = BOX_H;                        // lid underside = base top
static const double FACE_Z    = LID_Z;                        // motor flange face against lid underside
static const double LID_TOP   = LID_Z + LID_T;
static const double ARM_GAP   = 3.0;        // arm underside above lid top
static const double ARM_Z     = LID_TOP + ARM_GAP;

// ============================================================ helpers
static TopoDS_Shape box(double x0, double y0, double z0, double x1, double y1, double z1) {
    return BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), gp_Pnt(x1, y1, z1)).Shape();
}
static TopoDS_Shape cylZ(double x, double y, double z0, double r, double h) {
    return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(x, y, z0), gp_Dir(0, 0, 1)), r, h).Shape();
}
static TopoDS_Shape cylAxis(gp_Pnt p, gp_Dir d, double r, double h) {
    return BRepPrimAPI_MakeCylinder(gp_Ax2(p, d), r, h).Shape();
}
static bool isVertical(const TopoDS_Edge& e) {
    TopoDS_Vertex a, b; TopExp::Vertices(e, a, b);
    if (a.IsNull() || b.IsNull()) return false;
    gp_Pnt p = BRep_Tool::Pnt(a), q = BRep_Tool::Pnt(b);
    return std::fabs(p.X() - q.X()) < 1e-6 && std::fabs(p.Y() - q.Y()) < 1e-6 && std::fabs(p.Z() - q.Z()) > 1e-6;
}
// box centred on (cx,cy) with vertical edges filleted (r>0) or chamfered (r<0)
static TopoDS_Shape roundedBox(double cx, double cy, double z0, double w, double d, double h, double r) {
    TopoDS_Shape b = box(cx - w / 2, cy - d / 2, z0, cx + w / 2, cy + d / 2, z0 + h);
    if (r > 0) {
        BRepFilletAPI_MakeFillet f(b);
        for (TopExp_Explorer ex(b, TopAbs_EDGE); ex.More(); ex.Next())
            if (isVertical(TopoDS::Edge(ex.Current()))) f.Add(r, TopoDS::Edge(ex.Current()));
        return f.Shape();
    } else if (r < 0) {
        BRepFilletAPI_MakeChamfer c(b);
        for (TopExp_Explorer ex(b, TopAbs_EDGE); ex.More(); ex.Next())
            if (isVertical(TopoDS::Edge(ex.Current()))) c.Add(-r, TopoDS::Edge(ex.Current()));
        return c.Shape();
    }
    return b;
}
static TopoDS_Shape fuseAll(const TopoDS_Shape& base, const std::vector<TopoDS_Shape>& tools) {
    TopTools_ListOfShape args, tl; args.Append(base);
    for (auto& t : tools) tl.Append(t);
    BRepAlgoAPI_Fuse op; op.SetArguments(args); op.SetTools(tl); op.SetRunParallel(true); op.Build();
    if (!op.IsDone()) { fprintf(stderr, "fuse failed\n"); exit(2); }
    return op.Shape();
}
static TopoDS_Shape cutAll(const TopoDS_Shape& base, const std::vector<TopoDS_Shape>& tools) {
    TopTools_ListOfShape args, tl; args.Append(base);
    for (auto& t : tools) tl.Append(t);
    BRepAlgoAPI_Cut op; op.SetArguments(args); op.SetTools(tl); op.SetRunParallel(true); op.Build();
    if (!op.IsDone()) { fprintf(stderr, "cut failed\n"); exit(2); }
    return op.Shape();
}
static TopoDS_Shape clean(const TopoDS_Shape& s) {
    ShapeUpgrade_UnifySameDomain u(s, true, true, false); u.Build(); return u.Shape();
}
static TopoDS_Shape moved(const TopoDS_Shape& s, double dx, double dy, double dz) {
    gp_Trsf t; t.SetTranslation(gp_Vec(dx, dy, dz));
    return BRepBuilderAPI_Transform(s, t, true).Shape();
}
// countersunk M3 hole, head at z=ztop, through to zbot
static std::vector<TopoDS_Shape> cskHole(double x, double y, double zbot, double ztop) {
    double rb = CSK_D / 2, rs = CLR_D / 2, h = rb - rs;   // 90 degrees
    return {
        cylZ(x, y, zbot - 1, rs, ztop - zbot + 2),
        BRepPrimAPI_MakeCone(gp_Ax2(gp_Pnt(x, y, ztop - h), gp_Dir(0, 0, 1)), rs, rb, h).Shape(),
        cylZ(x, y, ztop - 0.001, rb, 1.0),
    };
}
// horizontal stadium through a wall. axis = 'x' or 'y'. (u = in-wall horizontal coord)
static std::vector<TopoDS_Shape> stadiumH(char axis, double wall0, double wall1, double u, double z, double width, double height) {
    // width along the wall, height vertical; rounded ends on the short side
    std::vector<TopoDS_Shape> v;
    if (width >= height) {
        double r = height / 2, half = width / 2 - r;
        if (axis == 'x') {
            v.push_back(box(wall0, u - half, z - r, wall1, u + half, z + r));
            v.push_back(cylAxis(gp_Pnt(wall0, u - half, z), gp_Dir(1, 0, 0), r, wall1 - wall0));
            v.push_back(cylAxis(gp_Pnt(wall0, u + half, z), gp_Dir(1, 0, 0), r, wall1 - wall0));
        } else {
            v.push_back(box(u - half, wall0, z - r, u + half, wall1, z + r));
            v.push_back(cylAxis(gp_Pnt(u - half, wall0, z), gp_Dir(0, 1, 0), r, wall1 - wall0));
            v.push_back(cylAxis(gp_Pnt(u + half, wall0, z), gp_Dir(0, 1, 0), r, wall1 - wall0));
        }
    } else {  // vertical slot
        double r = width / 2, half = height / 2 - r;
        if (axis == 'x') {
            v.push_back(box(wall0, u - r, z - half, wall1, u + r, z + half));
            v.push_back(cylAxis(gp_Pnt(wall0, u, z - half), gp_Dir(1, 0, 0), r, wall1 - wall0));
            v.push_back(cylAxis(gp_Pnt(wall0, u, z + half), gp_Dir(1, 0, 0), r, wall1 - wall0));
        } else {
            v.push_back(box(u - r, wall0, z - half, u + r, wall1, z + half));
            v.push_back(cylAxis(gp_Pnt(u, wall0, z - half), gp_Dir(0, 1, 0), r, wall1 - wall0));
            v.push_back(cylAxis(gp_Pnt(u, wall0, z + half), gp_Dir(0, 1, 0), r, wall1 - wall0));
        }
    }
    return v;
}
static void append(std::vector<TopoDS_Shape>& a, const std::vector<TopoDS_Shape>& b) { a.insert(a.end(), b.begin(), b.end()); }

// ============================================================ parts
// ---------- footprint: circle(r) ∩ square(half), extruded z0..z0+h
static TopoDS_Shape roundSquare(double r, double half, double z0, double h) {
    BRepAlgoAPI_Common c(cylZ(0, 0, z0, r, h), box(-half, -half, z0 - 1, half, half, z0 + h + 1));
    return c.Shape();
}

// ============================================================ v2 parameters: arm / pendulum
static const double ARM_R      = 14.0;   // hub radius = beam half-width
static const double ARM_H      = 28.0;
static const double ARM_END    = 58.0;   // arm end face (bearing flush)
static const double PIV_X      = 62.0;   // pendulum plate mid-plane radius (URDF arm_to_pendulum x)
static const double PIV_Z      = 14.0;   // pivot axis height above arm underside (URDF z)
static const double BORE_DEPTH = 20.0;
static const double D_FLAT     = 2.05;   // distance centre -> flat in the arm bore (shaft flat at 2.0)
static const double SET_Z      = 8.0;    // set-screw height
// bearing variant (argv[2]): "624x16" = 4x16x6 (default for now), "608" = 8x22x7
static double B608_OD = 16.0, B608_ID = 4.0, B608_W = 6.0, SEAT_D = 16.1;
static double JOURNAL_D  = 4.05;   // press fit in the inner ring
static double SHOULDER_D = 6.0, SHOULDER_W = 1.0;
static double WIN_D = 10.0;        // window through the end wall
static bool   MAG_IN_POCKET = false;   // 608: magnet sits in a pocket in the journal; small bearing: glued on the end face
static double REINF_D = 1.8;
// split bearing housing (printed separately, bore vertical), screwed onto the arm top with 2x M3 into inserts
static const double X_SPLIT = 49.0;          // arm end face = housing inner face
static const double TAB_X0 = 33.0, TAB_T = 3.0;   // housing top tab lies on the arm top from TAB_X0 to X_SPLIT
static const double TAB_SCREW_X = 37.0, TAB_SCREW_Y = 7.0;
static const double ARM_WIN_D = 8.0;         // window in the arm end wall for the AS5600 IC       // axial hole for a 1.75 mm filament / wire reinforcement (small journal only)
static const char* BRG_NAME = "4x16x6";
static const double MAG_D = 6.0, MAG_T = 2.5;            // diametric magnet for AS5600
static const double AS_BOARD = 23.0, AS_T = 1.6, AS_IC_T = 1.5;  // generic AS5600 module (check yours)
static const double PEN_T = 6.0;         // pendulum plate thickness
static const double COIN_D = 26.2, COIN_T = 2.2, COIN_Z = 64.0;  // UK 2p 25.91 x 2.03 (also fits CNY 1 yuan)

// derived along the arm x axis (set in setVariant)
static double SEAT_X0, JOURNAL_X0, MAG_X0, WALL_X0;   // MAG_X0 = magnet face towards the sensor
static void setVariant(const std::string& v) {
    if (v == "608") {
        B608_OD = 22.0; B608_ID = 8.0; B608_W = 7.0; SEAT_D = 22.1; JOURNAL_D = 8.1; SHOULDER_D = 10.5;
        WIN_D = 16.0; MAG_IN_POCKET = true; REINF_D = 0; BRG_NAME = "608";
    }
    SEAT_X0 = ARM_END - B608_W;
    JOURNAL_X0 = SEAT_X0 - 0.5;
    MAG_X0 = MAG_IN_POCKET ? JOURNAL_X0 : JOURNAL_X0 - MAG_T;
    WALL_X0 = MAG_X0 - 1.5 /*air gap*/ - AS_IC_T + 0.1;   // board face sits 0.1 off the wall face
}

// ---------- BASE
static TopoDS_Shape makeBase() {
    TopoDS_Shape outer = roundSquare(ENC_R, ENC_HALF, 0, BOX_H);
    TopoDS_Shape inner = roundSquare(ENC_R - WALL, ENC_HALF - WALL, FLOOR, BOX_H);
    TopoDS_Shape s = cutAll(outer, {inner});
    std::vector<TopoDS_Shape> add;
    for (int sx : {-1, 1}) for (int sy : {-1, 1})
        add.push_back(cylZ(sx * BOSS_XY, sy * BOSS_XY, FLOOR - 0.5, BOSS_R, BOX_H - FLOOR + 0.5));
    double hx = BAT_X / 2 + BAT_CLR, hy = BAT_Y / 2 + BAT_CLR, L = 12.0;
    for (int sx : {-1, 1}) for (int sy : {-1, 1}) {
        double x0 = sx * hx, y0 = sy * hy;
        add.push_back(box(std::min(x0, x0 + sx * STOP_T), std::min(y0 - sy * L, y0 + sy * STOP_T), FLOOR - 0.5,
                          std::max(x0, x0 + sx * STOP_T), std::max(y0 - sy * L, y0 + sy * STOP_T), FLOOR + STOP_H));
        add.push_back(box(std::min(x0 - sx * L, x0 + sx * STOP_T), std::min(y0, y0 + sy * STOP_T), FLOOR - 0.5,
                          std::max(x0 - sx * L, x0 + sx * STOP_T), std::max(y0, y0 + sy * STOP_T), FLOOR + STOP_H));
    }
    s = fuseAll(s, add);
    std::vector<TopoDS_Shape> cut;
    for (int sx : {-1, 1}) for (int sy : {-1, 1}) {      // M3 heat-set inserts for the lid screws
        cut.push_back(cylZ(sx * BOSS_XY, sy * BOSS_XY, BOX_H - INS_HOLE_DEPTH, INS_HOLE_D / 2, INS_HOLE_DEPTH + 1));
        cut.push_back(cylZ(sx * BOSS_XY, sy * BOSS_XY, BOX_H - INS_HOLE_DEPTH - INS_TIP_DEPTH, INS_TIP_D / 2, INS_TIP_DEPTH + 0.5));
    }
    double w0 = IN_HALF - 1, w1 = ENC_HALF + 1;
    double plugZ = BOX_H - MOTOR_L - PCB_GAP + 3.0;
    append(cut, stadiumH('x', w0, w1, 0.0, plugZ, CABLE_W, CABLE_H));
    append(cut, stadiumH('x', w0, w1, 0.0, CHG_Z, CHG_W, CHG_H));
    cut.push_back(cylAxis(gp_Pnt(-w1, 0, SWITCH_Z), gp_Dir(1, 0, 0), SWITCH_D / 2, w1 - w0));
    double vz = (VENT_Z0 + VENT_Z1) / 2, vh = VENT_Z1 - VENT_Z0;
    for (double u : {-12.0, -6.0, 0.0, 6.0, 12.0}) {
        append(cut, stadiumH('y', -w1, -w0, u, vz, VENT_W, vh));
        append(cut, stadiumH('y', w0, w1, u, vz, VENT_W, vh));
    }
    return clean(cutAll(s, cut));
}

// ---------- LID (origin: underside centre). Post removed; Ø6 hole for the encoder wires.
static TopoDS_Shape makeLid() {
    TopoDS_Shape s = roundSquare(ENC_R, ENC_HALF, 0, LID_T);
    double ro = ENC_R - WALL - LIP_CLEAR, ho = ENC_HALF - WALL - LIP_CLEAR;
    TopoDS_Shape lip = cutAll(roundSquare(ro, ho, -LIP_H, LIP_H + 0.5),
                              {roundSquare(ro - LIP_T, ho - LIP_T, -LIP_H - 1, LIP_H + 2)});
    std::vector<TopoDS_Shape> notch;
    for (int sx : {-1, 1}) for (int sy : {-1, 1})
        notch.push_back(cylZ(sx * BOSS_XY, sy * BOSS_XY, -LIP_H - 1, BOSS_R + 0.7, LIP_H + 1));
    lip = cutAll(lip, notch);
    s = fuseAll(s, {lip});
    std::vector<TopoDS_Shape> cut;
    cut.push_back(cylZ(0, 0, -1, CENTER_HOLE_D / 2, LID_T + 2));
    cut.push_back(cylZ(0, WIRE_HOLE_Y, -1, WIRE_HOLE_D / 2, LID_T + 2));
    double hp = HOLE_PITCH / 2;
    for (int sx : {-1, 1}) for (int sy : {-1, 1}) {
        append(cut, cskHole(sx * hp, sy * hp, 0, LID_T));
        append(cut, cskHole(sx * BOSS_XY, sy * BOSS_XY, 0, LID_T));
    }
    return clean(cutAll(s, cut));
}

// ---------- MOTOR (origin: flange face centre, shaft +z, shaft flat facing -x)
static TopoDS_Shape makeMotor() {
    TopoDS_Shape body = roundedBox(0, 0, -MOTOR_L, MOTOR_W, MOTOR_W, MOTOR_L, -MOTOR_CHAMF);
    TopoDS_Shape shaft = cylZ(0, 0, MOTOR_BOSS_H - 0.5, SHAFT_D / 2, SHAFT_L - MOTOR_BOSS_H + 0.5);
    shaft = cutAll(shaft, {box(-3, -3, SHAFT_L - 15, -(SHAFT_D / 2 - 0.5), 3, SHAFT_L + 1)});
    TopoDS_Shape s = fuseAll(body, {cylZ(0, 0, -0.5, MOTOR_BOSS_D / 2, MOTOR_BOSS_H + 0.5), shaft});
    std::vector<TopoDS_Shape> cut;
    double hp = HOLE_PITCH / 2;
    for (int sx : {-1, 1}) for (int sy : {-1, 1}) cut.push_back(cylZ(sx * hp, sy * hp, -4.5, 1.25, 5));
    return clean(cutAll(s, cut));
}

// ---------- ZDT Y42 V2.0 PCB (outline from DWG) + component envelopes, origin board centre on component side
static gp_Pnt P(double x, double y) { return gp_Pnt(x - 46.9, y - 46.9, 0); }
static gp_Pnt bulgeMid(gp_Pnt a, gp_Pnt b, double bulge) {
    double dx = b.X() - a.X(), dy = b.Y() - a.Y(), c = std::hypot(dx, dy), s = bulge * c / 2;
    double mx = (a.X() + b.X()) / 2, my = (a.Y() + b.Y()) / 2, nx = dy / c, ny = -dx / c;
    return gp_Pnt(mx + nx * s, my + ny * s, 0);
}
static TopoDS_Shape makeDriver() {
    struct Seg { double x0, y0, x1, y1, b; };
    std::vector<Seg> segs = {
        {31.695, 27.4, 62.105, 27.4, 0}, {62.105, 27.4, 64.273, 28.298, 0.199}, {64.273, 28.298, 65.502, 29.527, 0},
        {65.502, 29.527, 66.4, 31.695, 0.199}, {66.4, 31.695, 66.4, 62.105, 0}, {66.4, 62.105, 65.502, 64.273, 0.199},
        {65.502, 64.273, 64.273, 65.502, 0}, {64.273, 65.502, 62.105, 66.4, 0.199}, {62.105, 66.4, 51.936, 66.4, 0},
        {51.936, 66.4, 51.936, 64.8, 0}, {51.936, 64.8, 41.864, 64.8, 0}, {41.864, 64.8, 41.864, 66.4, 0},
        {41.864, 66.4, 31.695, 66.4, 0}, {31.695, 66.4, 29.527, 65.502, 0.199}, {29.527, 65.502, 28.298, 64.273, 0},
        {28.298, 64.273, 27.4, 62.105, 0.199}, {27.4, 62.105, 27.4, 31.695, 0}, {27.4, 31.695, 28.298, 29.527, 0.199},
        {28.298, 29.527, 29.527, 28.298, 0}, {29.527, 28.298, 31.695, 27.4, 0.199},
    };
    BRepBuilderAPI_MakeWire w;
    for (auto& g : segs) {
        gp_Pnt a = P(g.x0, g.y0), b = P(g.x1, g.y1);
        if (g.b == 0) w.Add(BRepBuilderAPI_MakeEdge(a, b).Edge());
        else w.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(a, bulgeMid(a, b, g.b), b).Value()).Edge());
    }
    TopoDS_Shape pcb = moved(BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(w.Wire()).Face(), gp_Vec(0, 0, PCB_T)).Shape(), 0, 0, -PCB_T);
    std::vector<TopoDS_Shape> cut; double hp = HOLE_PITCH / 2;
    for (int sx : {-1, 1}) for (int sy : {-1, 1}) cut.push_back(cylZ(sx * hp, sy * hp, -PCB_T - 1, 1.55, PCB_T + 2));
    pcb = cutAll(pcb, cut);
    double yc = 46.65 - 46.9;
    std::vector<TopoDS_Shape> parts = {
        box(13.6, yc - 11.2, 0, 19.6, yc + 11.2, 6.0),     // XH 8-pin right-angle (signals/power), +x edge
        box(-6.2, 13.9, 0, 6.2, 19.4, 7.0),                // XH 4-pin motor phases, +y edge
        box(-2.5, -2.0, 0, 2.5, 2.0, 1.6),                 // encoder IC
    };
    for (int sx : {-1, 1}) for (int sy : {-1, 1}) parts.push_back(cylZ(sx * hp, sy * hp, 0, 2.75, PCB_GAP)); // standoffs
    return clean(fuseAll(pcb, parts));
}
static TopoDS_Shape makeBattery() { return box(-BAT_X / 2, -BAT_Y / 2, 0, BAT_X / 2, BAT_Y / 2, BAT_Z); }

// ---------- ARM (origin: motor axis at the arm underside; beam along +x). Ends at X_SPLIT; the bearing
// lives in the separate housing (RZ42-005), screwed on with 2x M3 into inserts in the pocket roof.
static TopoDS_Shape makeArm() {
    BRepBuilderAPI_MakeWire w;
    gp_Pnt a(0, -ARM_R, 0), b(X_SPLIT, -ARM_R, 0), c(X_SPLIT, ARM_R, 0), d(0, ARM_R, 0);
    w.Add(BRepBuilderAPI_MakeEdge(a, b).Edge()); w.Add(BRepBuilderAPI_MakeEdge(b, c).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(c, d).Edge());
    w.Add(BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(d, gp_Pnt(-ARM_R, 0, 0), a).Value()).Edge());
    TopoDS_Shape s = BRepPrimAPI_MakePrism(BRepBuilderAPI_MakeFace(w.Wire()).Face(), gp_Vec(0, 0, ARM_H)).Shape();
    BRepFilletAPI_MakeFillet f(s);
    for (TopExp_Explorer ex(s, TopAbs_EDGE); ex.More(); ex.Next()) {
        TopoDS_Edge e = TopoDS::Edge(ex.Current()); TopoDS_Vertex v1, v2; TopExp::Vertices(e, v1, v2);
        if (!v1.IsNull() && !v2.IsNull() && std::fabs(BRep_Tool::Pnt(v1).X() - X_SPLIT) < 1e-6 && std::fabs(BRep_Tool::Pnt(v2).X() - X_SPLIT) < 1e-6) continue; // keep the mating face sharp
        f.Add(2.0, e);
    }
    f.Build(); if (f.IsDone()) s = f.Shape(); else fprintf(stderr, "arm fillet skipped\n");
    std::vector<TopoDS_Shape> cut;
    { BRepAlgoAPI_Common dc(cylZ(0, 0, -1, 2.55, BORE_DEPTH + 1), box(-D_FLAT, -3, -2, 3, 3, BORE_DEPTH + 1));
      cut.push_back(dc.Shape()); cut.push_back(cylZ(0, 0, -1, 2.55, 5.0)); }
    cut.push_back(cylAxis(gp_Pnt(-ARM_R - 1, 0, SET_Z), gp_Dir(1, 0, 0), INS_HOLE_D / 2, 1 + INS_HOLE_DEPTH));
    cut.push_back(cylAxis(gp_Pnt(-ARM_R + INS_HOLE_DEPTH - 0.5, 0, SET_Z), gp_Dir(1, 0, 0), INS_TIP_D / 2, ARM_R - INS_HOLE_DEPTH - 1.0));
    cut.push_back(box(26, -11, -1, WALL_X0, 11, ARM_H - 3));                                   // pocket, open underneath
    cut.push_back(box(WALL_X0 - AS_T - 0.2, -(AS_BOARD / 2 + 0.35), -1, WALL_X0, AS_BOARD / 2 + 0.35, PIV_Z + AS_BOARD / 2 + 0.5)); // AS5600 slot
    cut.push_back(cylAxis(gp_Pnt(WALL_X0 - 0.5, 0, PIV_Z), gp_Dir(1, 0, 0), ARM_WIN_D / 2, X_SPLIT - WALL_X0 + 1)); // IC window
    s = cutAll(s, cut);
    std::vector<TopoDS_Shape> boss;                                                             // insert bosses hanging from the roof
    for (int sy : {-1, 1}) boss.push_back(cylZ(TAB_SCREW_X, sy * TAB_SCREW_Y, ARM_H - 3 - 6, 4.5, 6.5));
    s = fuseAll(s, boss);
    std::vector<TopoDS_Shape> ins;
    for (int sy : {-1, 1}) {
        ins.push_back(cylZ(TAB_SCREW_X, sy * TAB_SCREW_Y, ARM_H - INS_HOLE_DEPTH, INS_HOLE_D / 2, INS_HOLE_DEPTH + 1));
        ins.push_back(cylZ(TAB_SCREW_X, sy * TAB_SCREW_Y, ARM_H - INS_HOLE_DEPTH - 4.5, INS_TIP_D / 2, 5.0));
    }
    return clean(cutAll(s, ins));
}
// ---------- BEARING HOUSING (same frame as the arm). End block x X_SPLIT..ARM_END + top tab over the arm.
// Print with the outer face (x = ARM_END) on the bed: the bearing seat is then vertical and round.
static TopoDS_Shape makeHousing() {
    TopoDS_Shape s = fuseAll(box(X_SPLIT, -ARM_R, 0, ARM_END, ARM_R, ARM_H + TAB_T),
                             {box(TAB_X0, -ARM_R, ARM_H, X_SPLIT + 0.5, ARM_R, ARM_H + TAB_T)});
    s = clean(s);
    BRepFilletAPI_MakeFillet f(s);
    for (TopExp_Explorer ex(s, TopAbs_EDGE); ex.More(); ex.Next()) {
        TopoDS_Edge e = TopoDS::Edge(ex.Current()); TopoDS_Vertex v1, v2; TopExp::Vertices(e, v1, v2);
        if (v1.IsNull() || v2.IsNull()) continue;
        gp_Pnt p = BRep_Tool::Pnt(v1), q = BRep_Tool::Pnt(v2);
        bool onMate = std::fabs(p.X() - X_SPLIT) < 1e-6 && std::fabs(q.X() - X_SPLIT) < 1e-6;
        bool onTabUnder = std::fabs(p.Z() - ARM_H) < 1e-6 && std::fabs(q.Z() - ARM_H) < 1e-6;
        bool concave = onMate && p.Z() >= ARM_H - 1e-6 && q.Z() >= ARM_H - 1e-6;
        if (onMate || onTabUnder || concave) continue;      // keep mating faces sharp
        f.Add(1.5, e);
    }
    f.Build(); if (f.IsDone()) s = f.Shape(); else fprintf(stderr, "housing fillet skipped\n");
    std::vector<TopoDS_Shape> cut;
    cut.push_back(cylAxis(gp_Pnt(SEAT_X0, 0, PIV_Z), gp_Dir(1, 0, 0), SEAT_D / 2, B608_W + 2));            // bearing seat
    cut.push_back(cylAxis(gp_Pnt(X_SPLIT - 1, 0, PIV_Z), gp_Dir(1, 0, 0), WIN_D / 2, SEAT_X0 - X_SPLIT + 2)); // floor window
    for (int sy : {-1, 1}) append(cut, cskHole(TAB_SCREW_X, sy * TAB_SCREW_Y, ARM_H, ARM_H + TAB_T));
    return clean(cutAll(s, cut));
}

// ---------- PENDULUM (origin: pivot centre on the plate mid-plane; journal along +Y; rod along +Z)
static TopoDS_Shape makePendulum() {
    auto disc = [&](double z, double r, double y0, double t) {
        return BRepPrimAPI_MakeCylinder(gp_Ax2(gp_Pnt(0, y0, z), gp_Dir(0, 1, 0)), r, t).Shape();
    };
    double h = PEN_T / 2;
    TopoDS_Shape s = fuseAll(disc(0, 9.0, -h, PEN_T), {
        box(-5, -h, 0, 5, h, COIN_Z),
        disc(COIN_Z, 16.0, -h, PEN_T),
        box(-6.5, -h, 0, 6.5, h, 8.5), box(-6.5, -h, 50, 6.5, h, 56),
        disc(0, SHOULDER_D / 2, h - 0.5, SHOULDER_W + 0.5),                                   // spacer shoulder -> inner ring
        disc(0, JOURNAL_D / 2, h + SHOULDER_W - 0.5, B608_W + 0.5 + 0.5),                     // Ø8.1 journal
    });
    double yEnd = h + SHOULDER_W + B608_W + 0.5;                                               // journal end face
    std::vector<TopoDS_Shape> pc = {disc(COIN_Z, COIN_D / 2, -COIN_T / 2, COIN_T)};          // sealed coin pocket
    if (MAG_IN_POCKET) pc.push_back(disc(0, (MAG_D + 0.1) / 2, yEnd - MAG_T - 0.1, MAG_T + 1)); // magnet pocket
    if (REINF_D > 0) pc.push_back(disc(0, REINF_D / 2, -h + 1.0, yEnd + h));                  // reinforcement pin hole (blind 1 mm from the outer face)
    s = cutAll(s, pc);
    return clean(s);
}

// ---------- purchased-part references (own frames)
static TopoDS_Shape makeBearing() {   // axis Z, centred
    return cutAll(cylZ(0, 0, -B608_W / 2, B608_OD / 2, B608_W), {cylZ(0, 0, -B608_W, B608_ID / 2, 2 * B608_W)});
}
static TopoDS_Shape makeMagnet() { return cylZ(0, 0, -MAG_T / 2, MAG_D / 2, MAG_T); }
static TopoDS_Shape makeInsert() {   // M3 x OD5 x L4, axis Z, top face at z=0
    return cutAll(cylZ(0, 0, -4, 2.5, 4), {cylZ(0, 0, -5, 1.25, 6)});
}
static TopoDS_Shape makeAS5600() {    // board in the YZ plane, IC on +X, origin at the IC centre on the board face
    return fuseAll(box(-AS_T, -AS_BOARD / 2, -AS_BOARD / 2, 0, AS_BOARD / 2, AS_BOARD / 2), {box(0, -2.5, -2.0, AS_IC_T, 2.5, 2.0)});
}

// ============================================================ placement (world = base frame)
static gp_Trsf rotZ(double a) { gp_Trsf t; t.SetRotation(gp::OZ(), a); return t; }
static gp_Trsf tr(double x, double y, double z) { gp_Trsf t; t.SetTranslation(gp_Vec(x, y, z)); return t; }
static gp_Trsf armPose(double theta) { return tr(0, 0, ARM_Z) * rotZ(-M_PI / 2 + theta); }  // URDF zero: arm along -y
static gp_Trsf penPose(double theta, double phi) {
    gp_Trsf sw; sw.SetRotation(gp::OX(), phi);
    gp_Trsf rz; rz.SetRotation(gp::OZ(), -M_PI / 2); gp_Trsf rx; rx.SetRotation(gp::OX(), M_PI);
    return armPose(theta) * tr(PIV_X, 0, PIV_Z) * sw * rz * rx;     // URDF visual rpy(pi,0,-pi/2)
}
static TopoDS_Shape place(const TopoDS_Shape& s, const gp_Trsf& t) { return BRepBuilderAPI_Transform(s, t, true).Shape(); }

// ============================================================ I/O + checks
static double volume(const TopoDS_Shape& s) { GProp_GProps g; BRepGProp::VolumeProperties(s, g); return g.Mass(); }
static void report(const char* name, const TopoDS_Shape& s) {
    Bnd_Box bb; BRepBndLib::AddOptimal(s, bb, false, false);
    double x0, y0, z0, x1, y1, z1; bb.Get(x0, y0, z0, x1, y1, z1);
    int nsol = 0; for (TopExp_Explorer e(s, TopAbs_SOLID); e.More(); e.Next()) nsol++;
    printf("%-26s valid=%d solids=%d vol=%9.1f mm3  size %.1f x %.1f x %.1f\n", name,
           (int)BRepCheck_Analyzer(s).IsValid(), nsol, volume(s), x1 - x0, y1 - y0, z1 - z0);
}
static double inter(const TopoDS_Shape& a, const TopoDS_Shape& b) {
    BRepAlgoAPI_Common c(a, b); return c.IsDone() ? volume(c.Shape()) : -1;
}
static void writeNamedStep(const TopoDS_Shape& s, const std::string& path, const char* name, Quantity_Color col) {
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc; app->NewDocument("MDTV-XCAF", doc);
    Handle(XCAFDoc_ShapeTool) st = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) ct = XCAFDoc_DocumentTool::ColorTool(doc->Main());
    TDF_Label l = st->AddShape(s, false); TDataStd_Name::Set(l, name); ct->SetColor(l, col, XCAFDoc_ColorSurf);
    STEPCAFControl_Writer w; Interface_Static::SetCVal("write.step.schema", "AP214IS");
    Interface_Static::SetCVal("write.step.unit", "MM");
    w.SetNameMode(true); w.SetColorMode(true); w.Transfer(doc, STEPControl_AsIs); w.Write(path.c_str());
    app->Close(doc);
}
static void writeStl(const TopoDS_Shape& s, const std::string& path, double defl = 0.05) {
    BRepMesh_IncrementalMesh m(s, defl, false, 0.3, true); StlAPI_Writer w; w.ASCIIMode() = false; w.Write(s, path.c_str());
}

int main(int argc, char** argv) {
    std::string out = argc > 1 ? argv[1] : "v2";
    setVariant(argc > 2 ? argv[2] : "4x16x6");
    printf("Bearing variant: %s  seat %.1f..%.1f  wall face %.2f  magnet face %.2f\n", BRG_NAME, SEAT_X0, ARM_END, WALL_X0, MAG_X0);
    TopoDS_Shape base = makeBase(), lid = makeLid(), motor = makeMotor(), drv = makeDriver(), bat = makeBattery();
    TopoDS_Shape arm = makeArm(), hsg = makeHousing(), pen = makePendulum(), brg = makeBearing(), mag = makeMagnet(), ins = makeInsert(), as = makeAS5600();
    printf("Heights: lid underside %.1f, lid top %.1f, arm underside %.1f, pivot axis %.1f, board top %.1f\n",
           LID_Z, LID_TOP, ARM_Z, ARM_Z + PIV_Z, FACE_Z - MOTOR_L - PCB_GAP);
    report("01 base", base); report("02 lid", lid); report("03 arm", arm); report("05 housing", hsg); report("04 pendulum", pen);
    report("motor", motor); report("driver", drv); report("battery", bat); report("bearing", brg);
    report("magnet", mag); report("insert", ins); report("as5600", as);

    // ---- static assembly (theta = 0, pendulum hanging)
    double boardTop = FACE_Z - MOTOR_L - PCB_GAP;
    TopoDS_Shape lidA = place(lid, tr(0, 0, LID_Z)), motorA = place(motor, tr(0, 0, FACE_Z));
    TopoDS_Shape drvA = place(drv, tr(0, 0, boardTop)), batA = place(bat, tr(0, 0, FLOOR));
    gp_Trsf rotY90; rotY90.SetRotation(gp::OY(), M_PI / 2);
    gp_Trsf brgL = tr(SEAT_X0 + B608_W / 2, 0, PIV_Z) * rotY90;                       // bearing in arm frame
    gp_Trsf magL = tr(MAG_X0 + MAG_T / 2, 0, PIV_Z) * rotY90;                          // magnet in arm frame
    gp_Trsf asL = tr(WALL_X0 - 0.0 - 0.0, 0, PIV_Z) * tr(0, 0, 0);                     // board face on the wall face
    gp_Trsf insL = tr(-ARM_R, 0, SET_Z) * rotY90 * rotY90 * rotY90;                    // insert axis +x, top at hub surface
    auto armT = [&](const gp_Trsf& l, double th) { return armPose(th) * l; };
    // AS5600: board occupies x in [-1.6,0] of its frame -> put board face at WALL_X0 - 0.1, IC into the window
    asL = tr(WALL_X0 - 0.1, 0, PIV_Z);

    printf("\nStatic interference (mm3):\n");
    struct C { const char* n; TopoDS_Shape a, b; const char* note; };
    std::vector<C> cs = {
        {"base/lid", base, lidA, ""}, {"base/motor", base, motorA, ""}, {"base/driver", base, drvA, ""},
        {"base/battery", base, batA, ""}, {"lid/motor", lidA, motorA, ""}, {"driver/motor", drvA, motorA, ""},
        {"driver/battery", drvA, batA, ""}, {"motor/battery", motorA, batA, ""},
        {"arm/motor(shaft)", place(arm, armPose(0)), place(motor, tr(0, 0, FACE_Z) * rotZ(-M_PI / 2)), "shaft flat aligned with bore flat"},
        {"housing/arm", place(hsg, armPose(0)), place(arm, armPose(0)), "faces touch"},
        {"housing/bearing", place(hsg, armPose(0)), place(brg, armT(brgL, 0)), "seat OD+0.1"},
        {"arm/bearing", place(arm, armPose(0)), place(brg, armT(brgL, 0)), ""},
        {"housing/pendulum", place(hsg, armPose(0)), place(pen, penPose(0, 0)), ""},
        {"housing/magnet", place(hsg, armPose(0)), place(mag, armT(magL, 0)), ""},
        {"housing/as5600", place(hsg, armPose(0)), place(as, armT(asL, 0)), ""},
        {"housing/lid", place(hsg, armPose(0)), lidA, ""},
        {"pendulum/bearing", place(pen, penPose(0, 0)), place(brg, armT(brgL, 0)), "journal press fit 0.05 (expected >0)"},
        {"pendulum/arm", place(pen, penPose(0, 0)), place(arm, armPose(0)), ""},
        {"magnet/pendulum", place(mag, armT(magL, 0)), place(pen, penPose(0, 0)), ""},
        {"as5600/arm", place(as, armT(asL, 0)), place(arm, armPose(0)), ""},
        {"as5600/magnet", place(as, armT(asL, 0)), place(mag, armT(magL, 0)), ""},
        {"as5600/pendulum", place(as, armT(asL, 0)), place(pen, penPose(0, 0)), ""},
        {"insert/arm", place(ins, armT(insL, 0)), place(arm, armPose(0)), "insert OD5 in Ø4.2 hole (expected >0, melt-in)"},
    };
    for (auto& c : cs) printf("  %-20s %9.3f   %s\n", c.n, inter(c.a, c.b), c.note);
    // magnet <-> sensor gap
    printf("  magnet face to AS5600 IC top: %.2f mm\n", MAG_X0 - (WALL_X0 - 0.1 + AS_IC_T));
    printf("  magnet/journal end contact: %s\n", MAG_IN_POCKET ? "in pocket" : "glued on the journal end face");

    // ---- kinematic sweep: arm 0..345 deg, pendulum swing -180..150 deg
    printf("\nKinematic sweep (arm theta every 5 deg, pendulum phi every 15 deg):\n");
    int hits = 0, n = 0;
    for (int td = 0; td < 360; td += 5) {
        double th = td * M_PI / 180;
        TopoDS_Shape armW = place(arm, armPose(th)), hW = place(hsg, armPose(th));
        double v1 = inter(armW, lidA) + inter(hW, lidA), v2 = inter(armW, base) + inter(hW, base);
        if (v1 > 1e-6 || v2 > 1e-6) { printf("  arm theta=%d hits lid/base (%.3f/%.3f)\n", td, v1, v2); hits++; }
        n += 2;
        for (int pd = -180; pd < 180; pd += 15) {
            TopoDS_Shape penW = place(pen, penPose(th, pd * M_PI / 180));
            double a = inter(penW, base), b = inter(penW, lidA), c = inter(penW, armW) + inter(penW, hW);
            n += 3;
            if (a > 1e-6 || b > 1e-6 || c > 1e-6) { printf("  theta=%d phi=%d: pen/base %.3f pen/lid %.3f pen/arm %.3f\n", td, pd, a, b, c); hits++; }
        }
    }
    printf("  %d pairwise checks, %d poses with interference\n", n, hits);

    // ---- pendulum mass properties (plastic only, solid) for comparison with the URDF
    { GProp_GProps g; BRepGProp::VolumeProperties(pen, g); gp_Pnt c = g.CentreOfMass();
      printf("\nPendulum (solid plastic) volume %.0f mm3, COM (%.2f, %.2f, %.2f) mm in part frame\n", g.Mass(), c.X(), c.Y(), c.Z()); }

    // ---- export individual parts
    std::string armN = std::string("RZ42-003_Arm_") + BRG_NAME, penN = std::string("RZ42-004_Pendulum_") + BRG_NAME,
                brgN = std::string("RZ42-104_Bearing_") + BRG_NAME, hsgN = std::string("RZ42-005_BearingHousing_") + BRG_NAME;
    struct E { TopoDS_Shape s; const char* file; const char* name; Quantity_Color col; };
    std::vector<E> ex = {
        {base, "RZ42-001_Base", "RZ42-001_Base", Quantity_Color(0.25, 0.25, 0.28, Quantity_TOC_RGB)},
        {lid, "RZ42-002_Lid", "RZ42-002_Lid", Quantity_Color(0.40, 0.40, 0.46, Quantity_TOC_RGB)},
        {arm, armN.c_str(), armN.c_str(), Quantity_Color(0.40, 0.60, 0.80, Quantity_TOC_RGB)},
        {pen, penN.c_str(), penN.c_str(), Quantity_Color(0.80, 0.40, 0.40, Quantity_TOC_RGB)},
        {motor, "RZ42-101_Motor_42x48", "RZ42-101_Motor_42x48", Quantity_Color(0.75, 0.75, 0.78, Quantity_TOC_RGB)},
        {drv, "RZ42-102_Driver_ZDT-Y42-V2.0", "RZ42-102_Driver_ZDT-Y42-V2.0", Quantity_Color(0.10, 0.50, 0.25, Quantity_TOC_RGB)},
        {bat, "RZ42-103_Battery_70x55x20", "RZ42-103_Battery_70x55x20", Quantity_Color(0.20, 0.35, 0.80, Quantity_TOC_RGB)},
        {brg, brgN.c_str(), brgN.c_str(), Quantity_Color(0.70, 0.70, 0.72, Quantity_TOC_RGB)},
        {hsg, hsgN.c_str(), hsgN.c_str(), Quantity_Color(0.30, 0.50, 0.75, Quantity_TOC_RGB)},
        {mag, "RZ42-105_Magnet_D6x2.5", "RZ42-105_Magnet_D6x2.5", Quantity_Color(0.55, 0.55, 0.60, Quantity_TOC_RGB)},
        {ins, "RZ42-106_HeatInsert_M3x5x4", "RZ42-106_HeatInsert_M3x5x4", Quantity_Color(0.85, 0.65, 0.25, Quantity_TOC_RGB)},
        {as, "RZ42-107_Encoder_AS5600_Module", "RZ42-107_Encoder_AS5600_Module", Quantity_Color(0.15, 0.30, 0.65, Quantity_TOC_RGB)},
    };
    for (auto& e : ex) writeNamedStep(e.s, out + "/" + e.file + ".STEP", e.name, e.col);
    // previews in assembly position (theta=30 deg so the arm is visible, pendulum hanging)
    double th = 0;
    writeStl(base, out + "/_asm_base.stl", 0.1); writeStl(lidA, out + "/_asm_lid.stl", 0.1);
    writeStl(motorA, out + "/_asm_motor.stl", 0.1); writeStl(drvA, out + "/_asm_driver.stl", 0.1);
    writeStl(batA, out + "/_asm_battery.stl", 0.1); writeStl(place(arm, armPose(th)), out + "/_asm_arm.stl", 0.05); writeStl(place(hsg, armPose(th)), out + "/_asm_housing.stl", 0.05);
    writeStl(place(pen, penPose(th, 0)), out + "/_asm_pendulum.stl", 0.05);
    writeStl(place(brg, armT(brgL, th)), out + "/_asm_bearing.stl", 0.05);
    writeStl(place(as, armT(asL, th)), out + "/_asm_as5600.stl", 0.05);
    writeStl(place(mag, armT(magL, th)), out + "/_asm_magnet.stl", 0.05);
    FILE* m = fopen((out + "/_meta.json").c_str(), "w");
    fprintf(m, "{\"arm_z\": %.3f, \"lid_top\": %.3f, \"lid_z\": %.3f, \"board_top\": %.3f, \"motor_rear\": %.3f, \"pivot_z\": %.3f}\n",
            ARM_Z, LID_TOP, LID_Z, boardTop, FACE_Z - MOTOR_L, ARM_Z + PIV_Z);
    fclose(m);
    printf("\nwritten to %s\n", out.c_str());
    return 0;
}
