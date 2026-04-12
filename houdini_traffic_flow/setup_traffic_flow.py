"""
Procedural Traffic Flow Intersection Curves - Houdini 21
=========================================================
Run in Houdini's Python Shell:
    exec(open(r"C:\\Users\\KABELO\\Downloads\\houdini_traffic_flow\\setup_traffic_flow.py").read())

Creates: /obj/traffic_flow_curves

Output matches the reference image:
  - Multi-lane parallel white road lines (straight through)
  - Red quarter-circle arcs at every intersection corner
  - Green original grid lines (construction reference)
  - Directional chevron indicators on arcs

Uses ConvertLine SOP (not Convert).
No U-turns. No Blast SOPs needed.
"""

import hou


# ============================================================
# VEX SNIPPETS
# ============================================================

CLASSIFY_VEX = r"""// classify_intersections - Run Over: Points
int nbs[] = neighbours(0, @ptnum);
i@neigh_count = len(nbs);

if(i@neigh_count == 4) {
    s@intersection_type = "4way";
    i@is_intersection = 1;
    setpointgroup(0, "type_4way", @ptnum, 1);
}
else if(i@neigh_count == 3) {
    s@intersection_type = "3way";
    i@is_intersection = 1;
    setpointgroup(0, "type_3way", @ptnum, 1);
}
else if(i@neigh_count == 2) {
    vector p0 = point(0, "P", nbs[0]);
    vector p1 = point(0, "P", nbs[1]);
    vector d0 = normalize(p0 - @P);
    vector d1 = normalize(p1 - @P);
    float ang = degrees(acos(clamp(dot(d0, d1), -1, 1)));
    if(ang < 170) {
        s@intersection_type = "corner";
        i@is_intersection = 1;
        setpointgroup(0, "type_corner", @ptnum, 1);
    } else {
        s@intersection_type = "straight";
        i@is_intersection = 0;
    }
}
else {
    s@intersection_type = "endpoint";
    i@is_intersection = 0;
}

int ncount = len(nbs);
for(int i = 0; i < ncount; i++) {
    vector nb_pos = point(0, "P", nbs[i]);
    vector dir = normalize(nb_pos - @P);
    setpointattrib(0, sprintf("dir_%d", i), @ptnum, dir, "set");
}
"""

GEN_LANES_VEX = r"""// gen_road_lanes - Run Over: Detail
float grid_size  = ch("grid_size");
int grid_div     = chi("grid_divisions");
int num_lanes    = chi("num_lanes");
float lane_space = ch("lane_spacing");

float cell = grid_size / float(grid_div);
float half = grid_size / 2.0;
int num_lines = grid_div + 1;

for(int row = 0; row < num_lines; row++) {
    float z_base = -half + float(row) * cell;
    for(int lane = 0; lane < num_lanes; lane++) {
        float off = (float(lane) - float(num_lanes - 1) / 2.0) * lane_space;
        int prim = addprim(0, "polyline");
        int p0 = addpoint(0, set(-half, 0, z_base + off));
        int p1 = addpoint(0, set( half, 0, z_base + off));
        setpointattrib(0, "curve_type", p0, "road", "set");
        setpointattrib(0, "curve_type", p1, "road", "set");
        addvertex(0, prim, p0);
        addvertex(0, prim, p1);
        setprimattrib(0, "is_lane", prim, 1, "set");
    }
}

for(int col = 0; col < num_lines; col++) {
    float x_base = -half + float(col) * cell;
    for(int lane = 0; lane < num_lanes; lane++) {
        float off = (float(lane) - float(num_lanes - 1) / 2.0) * lane_space;
        int prim = addprim(0, "polyline");
        int p0 = addpoint(0, set(x_base + off, 0, -half));
        int p1 = addpoint(0, set(x_base + off, 0,  half));
        setpointattrib(0, "curve_type", p0, "road", "set");
        setpointattrib(0, "curve_type", p1, "road", "set");
        addvertex(0, prim, p0);
        addvertex(0, prim, p1);
        setprimattrib(0, "is_lane", prim, 1, "set");
    }
}
"""

GEN_ARCS_VEX = r"""// gen_intersection_arcs - Run Over: Points
float road_hw     = ch("road_half_width");
int num_arcs      = chi("num_arcs");
float inner_r     = ch("inner_radius");
float arc_space   = ch("arc_spacing");
int arc_segs      = chi("arc_segments");
float chev_size   = ch("chevron_size");

if(i@is_intersection != 1) return;

int nbs[] = neighbours(0, @ptnum);
vector dirs[];
foreach(int nb; nbs) {
    append(dirs, normalize(point(0, "P", nb) - @P));
}

int ndirs = len(dirs);

for(int i = 0; i < ndirs; i++) {
    for(int j = i + 1; j < ndirs; j++) {
        vector d1 = dirs[i];
        vector d2 = dirs[j];
        float angle = degrees(acos(clamp(dot(d1, d2), -1, 1)));

        if(angle < 80 || angle > 100) continue;

        vector corner = @P + d1 * road_hw + d2 * road_hw;

        for(int a = 0; a < num_arcs; a++) {
            float r = inner_r + float(a) * arc_space;
            int prim = addprim(0, "polyline");

            for(int s = 0; s <= arc_segs; s++) {
                float theta = float(s) / float(arc_segs) * radians(90);
                vector pos = corner + r * (-d2 * cos(theta) - d1 * sin(theta));
                int pt = addpoint(0, pos);
                setpointattrib(0, "flow_t", pt, float(s) / float(arc_segs), "set");
                setpointattrib(0, "curve_type", pt, "turn", "set");
                addvertex(0, prim, pt);
            }

            setprimattrib(0, "is_arc", prim, 1, "set");
        }

        if(chev_size > 0 && num_arcs > 0) {
            int mid_idx   = num_arcs / 2;
            float mid_r   = inner_r + float(mid_idx) * arc_space;
            float mid_th  = radians(45);

            vector mid_pos = corner + mid_r * (-d2 * cos(mid_th) - d1 * sin(mid_th));
            vector tang = normalize(d2 * sin(mid_th) - d1 * cos(mid_th));
            vector perp = set(-tang.z, 0, tang.x);

            vector cb  = mid_pos - tang * chev_size;
            vector cl  = cb + perp * chev_size * 0.4;
            vector cr  = cb - perp * chev_size * 0.4;

            int cprim = addprim(0, "polyline");
            int cp0 = addpoint(0, cl);
            int cp1 = addpoint(0, mid_pos);
            int cp2 = addpoint(0, cr);

            setpointattrib(0, "curve_type", cp0, "turn", "set");
            setpointattrib(0, "curve_type", cp1, "turn", "set");
            setpointattrib(0, "curve_type", cp2, "turn", "set");

            addvertex(0, cprim, cp0);
            addvertex(0, cprim, cp1);
            addvertex(0, cprim, cp2);
            setprimattrib(0, "is_chevron", cprim, 1, "set");
        }
    }
}
"""

COLOR_VEX = r"""// color_visualization - Run Over: Points
if(s@curve_type == "road") {
    @Cd = {1, 1, 1};
}
else if(s@curve_type == "turn") {
    @Cd = {1.0, 0.23, 0.19};
}
else {
    @Cd = {0.13, 1.0, 0.4};
}
"""


# ============================================================
# HELPERS
# ============================================================

def add_lane_params(node, grid_size=348.0, grid_divisions=4,
                    num_lanes=5, lane_spacing=3.0):
    ptg = node.parmTemplateGroup()
    folder = hou.FolderParmTemplate("lane_params", "Lane Parameters")

    folder.addParmTemplate(hou.FloatParmTemplate(
        "grid_size", "Grid Size", 1,
        default_value=(grid_size,),
        min=10.0, max=1000.0,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.IntParmTemplate(
        "grid_divisions", "Grid Divisions", 1,
        default_value=(grid_divisions,),
        min=1, max=20,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.IntParmTemplate(
        "num_lanes", "Lanes per Road", 1,
        default_value=(num_lanes,),
        min=1, max=20,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "lane_spacing", "Lane Spacing", 1,
        default_value=(lane_spacing,),
        min=0.5, max=20.0,
        min_is_strict=False, max_is_strict=False
    ))

    ptg.append(folder)
    node.setParmTemplateGroup(ptg)


def add_arc_params(node, road_half_width=15.0, num_arcs=3,
                   inner_radius=6.0, arc_spacing=5.0,
                   arc_segments=12, chevron_size=4.0):
    ptg = node.parmTemplateGroup()
    folder = hou.FolderParmTemplate("arc_params", "Arc Parameters")

    folder.addParmTemplate(hou.FloatParmTemplate(
        "road_half_width", "Road Half Width", 1,
        default_value=(road_half_width,),
        min=5.0, max=50.0,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.IntParmTemplate(
        "num_arcs", "Arcs per Corner", 1,
        default_value=(num_arcs,),
        min=1, max=10,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "inner_radius", "Inner Radius", 1,
        default_value=(inner_radius,),
        min=1.0, max=40.0,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "arc_spacing", "Arc Spacing", 1,
        default_value=(arc_spacing,),
        min=1.0, max=20.0,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.IntParmTemplate(
        "arc_segments", "Arc Segments", 1,
        default_value=(arc_segments,),
        min=4, max=32,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "chevron_size", "Chevron Size", 1,
        default_value=(chevron_size,),
        min=0.0, max=20.0,
        min_is_strict=False, max_is_strict=False
    ))

    ptg.append(folder)
    node.setParmTemplateGroup(ptg)


# ============================================================
# MAIN: BUILD THE NODE NETWORK
# ============================================================

def create_traffic_flow():
    obj = hou.node("/obj")
    geo = obj.createNode("geo", "traffic_flow_curves")

    # Remove default File SOP
    for child in geo.children():
        child.destroy()

    # ----------------------------------------------------------
    # 1. Grid SOP
    # ----------------------------------------------------------
    grid = geo.createNode("grid", "base_grid")
    grid.parm("sizex").set(348)
    grid.parm("sizey").set(348)
    grid.parm("rows").set(5)       # 5 points = 4 divisions
    grid.parm("cols").set(5)
    grid.parm("orient").set(2)     # ZX Plane (Y up)

    # ----------------------------------------------------------
    # 2. ConvertLine SOP (NOT Convert!)
    # ----------------------------------------------------------
    convert = geo.createNode("convertline", "to_lines")
    convert.setInput(0, grid)

    # ----------------------------------------------------------
    # 3. Fuse SOP — snap shared vertices at intersections
    # ----------------------------------------------------------
    fuse = geo.createNode("fuse", "snap_shared_points")
    dist_parm = fuse.parm("dist") or fuse.parm("snapdist")
    if dist_parm:
        dist_parm.set(0.001)
    fuse.setInput(0, convert)

    # ==========================================================
    # BRANCH A: Multi-lane road lines
    # ==========================================================
    gen_lanes = geo.createNode("attribwrangle", "gen_road_lanes")
    gen_lanes.parm("snippet").set(GEN_LANES_VEX)
    gen_lanes.parm("class").set(0)  # Detail (runs once)
    gen_lanes.setInput(0, fuse)
    add_lane_params(gen_lanes)

    # Keep only the newly created lane primitives (remove original grid)
    blast_keep_lanes = geo.createNode("blast", "keep_only_lanes")
    blast_keep_lanes.parm("group").set("@is_lane==1")
    blast_keep_lanes.parm("negate").set(1)   # Keep selected, delete rest
    blast_keep_lanes.setInput(0, gen_lanes)

    # ==========================================================
    # BRANCH B: Intersection arcs
    # ==========================================================
    classify = geo.createNode("attribwrangle", "classify_intersections")
    classify.parm("snippet").set(CLASSIFY_VEX)
    classify.parm("class").set(2)  # Points
    classify.setInput(0, fuse)

    gen_arcs = geo.createNode("attribwrangle", "gen_intersection_arcs")
    gen_arcs.parm("snippet").set(GEN_ARCS_VEX)
    gen_arcs.parm("class").set(2)  # Points
    gen_arcs.setInput(0, classify)
    add_arc_params(gen_arcs)

    # Resample for smoother arcs
    resample = geo.createNode("resample", "smooth_arcs")
    resample.parm("length").set(2)
    last_vtx = resample.parm("maintainlast")
    if last_vtx:
        last_vtx.set(1)
    resample.setInput(0, gen_arcs)

    # ==========================================================
    # FINAL MERGE: lanes + arcs + original grid
    # ==========================================================
    final_merge = geo.createNode("merge", "final_output")
    final_merge.setInput(0, blast_keep_lanes)  # White road lanes
    final_merge.setInput(1, resample)          # Red intersection arcs
    final_merge.setInput(2, convert)           # Green original grid lines

    # Color everything
    color = geo.createNode("attribwrangle", "color_visualization")
    color.parm("snippet").set(COLOR_VEX)
    color.parm("class").set(2)  # Points
    color.setInput(0, final_merge)

    # Display
    color.setDisplayFlag(True)
    color.setRenderFlag(True)

    # Layout
    geo.layoutChildren()

    # Summary
    print("=" * 60)
    print("Traffic Flow Curves network created!")
    print("  Node: {}".format(geo.path()))
    print("")
    print("  BRANCH A — Road Lanes:")
    print("    gen_road_lanes > Lane Parameters folder")
    print("      Grid Size: 348 | Divisions: 4 | Lanes: 5 | Spacing: 3")
    print("")
    print("  BRANCH B — Intersection Arcs:")
    print("    gen_intersection_arcs > Arc Parameters folder")
    print("      Road Half Width: 15 | Arcs/Corner: 3")
    print("      Inner Radius: 6 | Spacing: 5 | Segments: 12")
    print("")
    print("  Colors:")
    print("    White = road lanes | Red = turning arcs | Green = grid")
    print("=" * 60)

    return geo


# ============================================================
# RUN
# ============================================================
create_traffic_flow()
