"""
Procedural Traffic Flow Intersection Curves - Houdini 21
=========================================================
Run in Houdini's Python Shell:
    exec(open(r"C:\\Users\\KABELO\\Downloads\\houdini_traffic_flow\\setup_traffic_flow.py").read())

Creates: /obj/traffic_flow_curves

Right-hand traffic, 2 lanes per direction (14m road):
  Inner lane = left turn only
  Outer lane = right turn only
  Direction arrows placed between lane lines
"""

import hou


# ============================================================
# VEX SNIPPETS
# ============================================================

CLASSIFY_VEX = r"""int nbs[] = neighbours(0, @ptnum);
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

GEN_LANES_VEX = r"""float grid_size   = ch("grid_size");
int grid_div      = chi("grid_divisions");
int num_lines     = chi("num_lane_lines");
float lane_space  = ch("lane_spacing");

float cell = grid_size / float(grid_div);
float half = grid_size / 2.0;
int grid_lines = grid_div + 1;

for(int row = 0; row < grid_lines; row++) {
    float z_base = -half + float(row) * cell;
    for(int i = 0; i < num_lines; i++) {
        float off = (float(i) - float(num_lines - 1) / 2.0) * lane_space;
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

for(int col = 0; col < grid_lines; col++) {
    float x_base = -half + float(col) * cell;
    for(int i = 0; i < num_lines; i++) {
        float off = (float(i) - float(num_lines - 1) / 2.0) * lane_space;
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

GEN_ARCS_VEX = r"""float lane_space  = ch("lane_spacing");
float entry_dist  = ch("entry_dist");
int arc_segs      = chi("arc_segments");
float chev_size   = ch("chevron_size");
float ind_dist    = ch("indicator_dist");

if(i@is_intersection != 1) return;

float inner_off = lane_space * 0.5;
float outer_off = lane_space * 1.5;

int nbs[] = neighbours(0, @ptnum);
vector dirs[];
foreach(int nb; nbs) {
    append(dirs, normalize(point(0, "P", nb) - @P));
}
int ndirs = len(dirs);

function vector bezier4(vector p0; vector p1; vector p2; vector p3; float t) {
    float u = 1.0 - t;
    return u*u*u*p0 + 3.0*u*u*t*p1 + 3.0*u*t*t*p2 + t*t*t*p3;
}

for(int ni = 0; ni < ndirs; ni++) {
    vector d = dirs[ni];
    vector travel = -d;
    vector right_d = set(-travel.z, 0, travel.x);
    vector left_d  = -right_d;

    int can_right = 0;
    int can_left = 0;
    int can_straight = 0;

    for(int k = 0; k < ndirs; k++) {
        if(dot(dirs[k], right_d) > 0.9)  can_right    = 1;
        if(dot(dirs[k], left_d)  > 0.9)  can_left     = 1;
        if(dot(dirs[k], travel)  > 0.9)  can_straight  = 1;
    }

    // RIGHT TURN (outer lane)
    if(can_right) {
        vector new_right = set(-right_d.z, 0, right_d.x);
        vector start  = @P + d * entry_dist + right_d * outer_off;
        vector end_pt = @P + right_d * entry_dist + new_right * outer_off;
        float right_radius = entry_dist - outer_off;
        float handle = right_radius * 0.5523;
        vector cp1 = start - d * handle;
        vector cp2 = end_pt - right_d * handle;

        int prim = addprim(0, "polyline");
        for(int s = 0; s <= arc_segs; s++) {
            float t = float(s) / float(arc_segs);
            vector pos = bezier4(start, cp1, cp2, end_pt, t);
            int pt = addpoint(0, pos);
            setpointattrib(0, "curve_type", pt, "turn", "set");
            setpointattrib(0, "turn_type", pt, "right", "set");
            setpointattrib(0, "flow_t", pt, t, "set");
            addvertex(0, prim, pt);
        }
    }

    // LEFT TURN (inner lane)
    if(can_left) {
        vector new_right_l = set(-left_d.z, 0, left_d.x);
        vector start  = @P + d * entry_dist + right_d * inner_off;
        vector end_pt = @P + left_d * entry_dist + new_right_l * inner_off;
        float left_radius = entry_dist + inner_off;
        float handle = left_radius * 0.5523;
        vector cp1 = start - d * handle;
        vector cp2 = end_pt - left_d * handle;

        int prim = addprim(0, "polyline");
        for(int s = 0; s <= arc_segs; s++) {
            float t = float(s) / float(arc_segs);
            vector pos = bezier4(start, cp1, cp2, end_pt, t);
            int pt = addpoint(0, pos);
            setpointattrib(0, "curve_type", pt, "turn", "set");
            setpointattrib(0, "turn_type", pt, "left", "set");
            setpointattrib(0, "flow_t", pt, t, "set");
            addvertex(0, prim, pt);
        }
    }

    // DIRECTION INDICATORS — inner lane
    {
        vector arrow_dir = can_left ? left_d : travel;
        vector pos = @P + d * ind_dist + right_d * inner_off;
        vector perp = set(-arrow_dir.z, 0, arrow_dir.x);
        vector tip    = pos + arrow_dir * chev_size;
        vector tail_l = pos - arrow_dir * chev_size * 0.3 + perp * chev_size * 0.5;
        vector tail_r = pos - arrow_dir * chev_size * 0.3 - perp * chev_size * 0.5;

        int iprim = addprim(0, "polyline");
        int ip0 = addpoint(0, tail_l); int ip1 = addpoint(0, tip); int ip2 = addpoint(0, tail_r);
        setpointattrib(0, "curve_type", ip0, "indicator", "set");
        setpointattrib(0, "curve_type", ip1, "indicator", "set");
        setpointattrib(0, "curve_type", ip2, "indicator", "set");
        addvertex(0, iprim, ip0); addvertex(0, iprim, ip1); addvertex(0, iprim, ip2);
        setprimattrib(0, "is_indicator", iprim, 1, "set");
    }

    // DIRECTION INDICATORS — outer lane
    {
        vector arrow_dir = can_right ? right_d : travel;
        vector pos = @P + d * ind_dist + right_d * outer_off;
        vector perp = set(-arrow_dir.z, 0, arrow_dir.x);
        vector tip    = pos + arrow_dir * chev_size;
        vector tail_l = pos - arrow_dir * chev_size * 0.3 + perp * chev_size * 0.5;
        vector tail_r = pos - arrow_dir * chev_size * 0.3 - perp * chev_size * 0.5;

        int oprim = addprim(0, "polyline");
        int op0 = addpoint(0, tail_l); int op1 = addpoint(0, tip); int op2 = addpoint(0, tail_r);
        setpointattrib(0, "curve_type", op0, "indicator", "set");
        setpointattrib(0, "curve_type", op1, "indicator", "set");
        setpointattrib(0, "curve_type", op2, "indicator", "set");
        addvertex(0, oprim, op0); addvertex(0, oprim, op1); addvertex(0, oprim, op2);
        setprimattrib(0, "is_indicator", oprim, 1, "set");
    }

    // STRAIGHT ARROW (extra, in outer lane if right turn + straight both exist)
    if(can_straight && can_right) {
        vector pos = @P + d * (ind_dist + chev_size * 3) + right_d * outer_off;
        vector perp = set(-travel.z, 0, travel.x);
        vector tip    = pos + travel * chev_size;
        vector tail_l = pos - travel * chev_size * 0.3 + perp * chev_size * 0.5;
        vector tail_r = pos - travel * chev_size * 0.3 - perp * chev_size * 0.5;

        int sprim = addprim(0, "polyline");
        int sp0 = addpoint(0, tail_l); int sp1 = addpoint(0, tip); int sp2 = addpoint(0, tail_r);
        setpointattrib(0, "curve_type", sp0, "indicator", "set");
        setpointattrib(0, "curve_type", sp1, "indicator", "set");
        setpointattrib(0, "curve_type", sp2, "indicator", "set");
        addvertex(0, sprim, sp0); addvertex(0, sprim, sp1); addvertex(0, sprim, sp2);
        setprimattrib(0, "is_indicator", sprim, 1, "set");
    }

    // STRAIGHT ARROW (extra, in inner lane if left turn + straight both exist)
    if(can_straight && can_left) {
        vector pos = @P + d * (ind_dist + chev_size * 3) + right_d * inner_off;
        vector perp = set(-travel.z, 0, travel.x);
        vector tip    = pos + travel * chev_size;
        vector tail_l = pos - travel * chev_size * 0.3 + perp * chev_size * 0.5;
        vector tail_r = pos - travel * chev_size * 0.3 - perp * chev_size * 0.5;

        int sprim = addprim(0, "polyline");
        int sp0 = addpoint(0, tail_l); int sp1 = addpoint(0, tip); int sp2 = addpoint(0, tail_r);
        setpointattrib(0, "curve_type", sp0, "indicator", "set");
        setpointattrib(0, "curve_type", sp1, "indicator", "set");
        setpointattrib(0, "curve_type", sp2, "indicator", "set");
        addvertex(0, sprim, sp0); addvertex(0, sprim, sp1); addvertex(0, sprim, sp2);
        setprimattrib(0, "is_indicator", sprim, 1, "set");
    }
}
"""

COLOR_VEX = r"""if(s@curve_type == "road") {
    @Cd = {1, 1, 1};
}
else if(s@curve_type == "turn") {
    @Cd = {1.0, 0.23, 0.19};
}
else if(s@curve_type == "indicator") {
    @Cd = {1.0, 0.85, 0.0};
}
else {
    @Cd = {0.13, 1.0, 0.4};
}
"""


# ============================================================
# HELPERS
# ============================================================

def add_lane_params(node, grid_size=348.0, grid_divisions=4,
                    num_lane_lines=5, lane_spacing=3.5):
    ptg = node.parmTemplateGroup()
    folder = hou.FolderParmTemplate("lane_params", "Lane Parameters")

    folder.addParmTemplate(hou.FloatParmTemplate(
        "grid_size", "Grid Size", 1,
        default_value=(grid_size,), min=10.0, max=1000.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.IntParmTemplate(
        "grid_divisions", "Grid Divisions", 1,
        default_value=(grid_divisions,), min=1, max=20,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.IntParmTemplate(
        "num_lane_lines", "Lane Lines (5 = 4 lanes)", 1,
        default_value=(num_lane_lines,), min=2, max=20,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "lane_spacing", "Lane Spacing (m)", 1,
        default_value=(lane_spacing,), min=0.5, max=20.0,
        min_is_strict=False, max_is_strict=False))

    ptg.append(folder)
    node.setParmTemplateGroup(ptg)


def add_arc_params(node, lane_spacing=3.5, entry_dist=20.0,
                   arc_segments=16, chevron_size=3.0, indicator_dist=32.0):
    ptg = node.parmTemplateGroup()
    folder = hou.FolderParmTemplate("arc_params", "Arc & Indicator Parameters")

    folder.addParmTemplate(hou.FloatParmTemplate(
        "lane_spacing", "Lane Spacing (m)", 1,
        default_value=(lane_spacing,), min=0.5, max=20.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "entry_dist", "Arc Entry Distance", 1,
        default_value=(entry_dist,), min=3.0, max=50.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.IntParmTemplate(
        "arc_segments", "Arc Segments", 1,
        default_value=(arc_segments,), min=4, max=32,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "chevron_size", "Arrow Size", 1,
        default_value=(chevron_size,), min=0.5, max=10.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "indicator_dist", "Arrow Distance from Centre", 1,
        default_value=(indicator_dist,), min=5.0, max=60.0,
        min_is_strict=False, max_is_strict=False))

    ptg.append(folder)
    node.setParmTemplateGroup(ptg)


# ============================================================
# MAIN
# ============================================================

def create_traffic_flow():
    obj = hou.node("/obj")
    geo = obj.createNode("geo", "traffic_flow_curves")

    for child in geo.children():
        child.destroy()

    # 1. Grid
    grid = geo.createNode("grid", "base_grid")
    grid.parm("sizex").set(348)
    grid.parm("sizey").set(348)
    grid.parm("rows").set(5)
    grid.parm("cols").set(5)
    grid.parm("orient").set(2)

    # 2. ConvertLine
    convert = geo.createNode("convertline", "to_lines")
    convert.setInput(0, grid)

    # 3. Fuse
    fuse = geo.createNode("fuse", "snap_shared_points")
    dist_parm = fuse.parm("dist") or fuse.parm("snapdist")
    if dist_parm:
        dist_parm.set(0.001)
    fuse.setInput(0, convert)

    # === BRANCH A: Road Lanes ===
    gen_lanes = geo.createNode("attribwrangle", "gen_road_lanes")
    gen_lanes.parm("snippet").set(GEN_LANES_VEX)
    gen_lanes.parm("class").set(0)  # Detail
    gen_lanes.setInput(0, fuse)
    add_lane_params(gen_lanes)

    blast_keep_lanes = geo.createNode("blast", "keep_only_lanes")
    blast_keep_lanes.parm("group").set("@is_lane==1")
    blast_keep_lanes.parm("negate").set(1)
    blast_keep_lanes.setInput(0, gen_lanes)

    # === BRANCH B: Intersection Arcs + Indicators ===
    classify = geo.createNode("attribwrangle", "classify_intersections")
    classify.parm("snippet").set(CLASSIFY_VEX)
    classify.parm("class").set(2)  # Points
    classify.setInput(0, fuse)

    gen_arcs = geo.createNode("attribwrangle", "gen_intersection_arcs")
    gen_arcs.parm("snippet").set(GEN_ARCS_VEX)
    gen_arcs.parm("class").set(2)  # Points
    gen_arcs.setInput(0, classify)
    add_arc_params(gen_arcs)

    resample = geo.createNode("resample", "smooth_arcs")
    resample.parm("length").set(2)
    last_vtx = resample.parm("maintainlast")
    if last_vtx:
        last_vtx.set(1)
    resample.setInput(0, gen_arcs)

    # === FINAL MERGE ===
    final_merge = geo.createNode("merge", "final_output")
    final_merge.setInput(0, blast_keep_lanes)
    final_merge.setInput(1, resample)
    final_merge.setInput(2, convert)

    color = geo.createNode("attribwrangle", "color_visualization")
    color.parm("snippet").set(COLOR_VEX)
    color.parm("class").set(2)
    color.setInput(0, final_merge)

    color.setDisplayFlag(True)
    color.setRenderFlag(True)

    geo.layoutChildren()

    print("=" * 60)
    print("Traffic Flow Curves — Right-Hand Traffic")
    print("  Node: {}".format(geo.path()))
    print("")
    print("  Road: 14m wide (5 lines x 3.5m spacing)")
    print("  Inner lane: LEFT TURN ONLY")
    print("  Outer lane: RIGHT TURN ONLY")
    print("")
    print("  Colors:")
    print("    White  = road lanes")
    print("    Red    = turn arcs (bus-compatible radius)")
    print("    Amber  = direction arrows")
    print("    Green  = grid reference")
    print("")
    print("  Arc entry dist: 20 (turn radius ~14.75m for right, ~21.75m for left)")
    print("=" * 60)

    return geo

create_traffic_flow()
