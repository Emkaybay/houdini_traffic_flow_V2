"""
Procedural Traffic Flow Intersection Curves - Houdini 21
=========================================================
Run this script in Houdini's Python Shell (Windows > Python Shell)
or the Python Source Editor to auto-build the entire node network.

Creates: /obj/traffic_flow_curves

Features:
  - 4x4 grid (348x348, 5x5 points, 87-unit spacing)
  - Intersection classification (4-way, 3-way, corner)
  - Cubic Bezier traffic flow curves (straight-through + turns)
  - Directional chevron indicators at curve midpoints
  - Color coding (green = straight, red = turn)
  - Exposed spare parameters on each curve wrangle

No U-turns generated.
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

# Shared curve generation body (parameterised by intersection type string)
_CURVE_GEN_TEMPLATE = r"""// {label} - Run Over: Points
float road_width  = ch("road_width");
float turn_radius = ch("turn_radius");
int arc_segments  = chi("arc_segments");
float chevron_size = ch("chevron_size");

if(s@intersection_type != "{type_str}") return;

int nbs[] = neighbours(0, @ptnum);
vector dirs[];
foreach(int nb; nbs) {{
    vector nb_pos = point(0, "P", nb);
    append(dirs, normalize(nb_pos - @P));
}}

float sort_angles[];
foreach(vector d; dirs) {{
    append(sort_angles, atan2(d.z, d.x));
}}
for(int i = 0; i < len(sort_angles); i++) {{
    for(int j = i+1; j < len(sort_angles); j++) {{
        if(sort_angles[j] < sort_angles[i]) {{
            float tmp_a = sort_angles[i]; sort_angles[i] = sort_angles[j]; sort_angles[j] = tmp_a;
            vector tmp_d = dirs[i]; dirs[i] = dirs[j]; dirs[j] = tmp_d;
        }}
    }}
}}

function vector bezier4(vector p0; vector p1; vector p2; vector p3; float t) {{
    float u = 1.0 - t;
    return u*u*u*p0 + 3.0*u*u*t*p1 + 3.0*u*t*t*p2 + t*t*t*p3;
}}

int ndirs = len(dirs);
for(int i = 0; i < ndirs; i++) {{
    for(int j = i+1; j < ndirs; j++) {{
        vector d_in  = dirs[i];
        vector d_out = dirs[j];
        float angle = degrees(acos(clamp(dot(d_in, d_out), -1, 1)));

        if(angle < 60) continue;

        vector start  = @P + d_in  * road_width;
        vector end_pt = @P + d_out * road_width;

        float handle_len;
        if(angle > 160) {{
            handle_len = road_width * 0.4;
        }} else {{
            handle_len = turn_radius * 0.55;
        }}

        vector cp1 = start  - d_in  * handle_len;
        vector cp2 = end_pt - d_out * handle_len;

        int prim = addprim(0, "polyline");
        for(int s = 0; s <= arc_segments; s++) {{
            float t = float(s) / float(arc_segments);
            vector pos = bezier4(start, cp1, cp2, end_pt, t);
            int pt = addpoint(0, pos);
            setpointattrib(0, "flow_t", pt, t, "set");
            setpointattrib(0, "curve_type", pt,
                angle > 160 ? "straight" : "turn", "set");
            addvertex(0, prim, pt);
        }}

        float mid_t = 0.5;
        vector mid_pos = bezier4(start, cp1, cp2, end_pt, mid_t);
        vector tangent = normalize(
            bezier4(start, cp1, cp2, end_pt, mid_t + 0.01) -
            bezier4(start, cp1, cp2, end_pt, mid_t - 0.01)
        );
        vector perp = set(-tangent.z, 0, tangent.x);

        vector chev_back  = mid_pos - tangent * chevron_size;
        vector chev_left  = chev_back + perp * chevron_size * 0.4;
        vector chev_right = chev_back - perp * chevron_size * 0.4;

        int chev_prim = addprim(0, "polyline");
        int p0 = addpoint(0, chev_left);
        int p1 = addpoint(0, mid_pos);
        int p2 = addpoint(0, chev_right);
        addvertex(0, chev_prim, p0);
        addvertex(0, chev_prim, p1);
        addvertex(0, chev_prim, p2);
        setprimattrib(0, "is_chevron", chev_prim, 1, "set");
    }}
}}
"""

GEN_4WAY_VEX = _CURVE_GEN_TEMPLATE.format(
    label="gen_4way_curves", type_str="4way"
)
GEN_3WAY_VEX = _CURVE_GEN_TEMPLATE.format(
    label="gen_3way_curves", type_str="3way"
)
GEN_CORNER_VEX = _CURVE_GEN_TEMPLATE.format(
    label="gen_corner_curves", type_str="corner"
)

COLOR_VEX = r"""// color_visualization - Run Over: Points
if(s@curve_type == "straight") {
    @Cd = {0.13, 1.0, 0.4};
}
else if(s@curve_type == "turn") {
    @Cd = {1.0, 0.23, 0.19};
}
else {
    @Cd = {0.5, 0.5, 1.0};
}
"""


# ============================================================
# HELPER: ADD SPARE PARAMETERS TO WRANGLE NODES
# ============================================================

def add_curve_params(wrangle_node,
                     road_width=35.0,
                     turn_radius=30.0,
                     arc_segments=16,
                     chevron_size=8.0):
    """
    Adds spare parameters (Road Width, Turn Radius, Arc Segments,
    Chevron Size) to an Attribute Wrangle so the VEX ch() calls resolve.
    """
    ptg = wrangle_node.parmTemplateGroup()

    folder = hou.FolderParmTemplate(
        "curve_params_folder", "Curve Parameters"
    )

    folder.addParmTemplate(hou.FloatParmTemplate(
        "road_width", "Road Width", 1,
        default_value=(road_width,),
        min=10.0, max=43.0,
        min_is_strict=False, max_is_strict=False
    ))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "turn_radius", "Turn Radius", 1,
        default_value=(turn_radius,),
        min=10.0, max=43.0,
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
        min=2.0, max=20.0,
        min_is_strict=False, max_is_strict=False
    ))

    ptg.append(folder)
    wrangle_node.setParmTemplateGroup(ptg)


# ============================================================
# MAIN: BUILD THE NODE NETWORK
# ============================================================

def create_traffic_flow():
    """
    Creates the complete traffic flow curves network
    under /obj/traffic_flow_curves.
    """
    obj = hou.node("/obj")
    geo = obj.createNode("geo", "traffic_flow_curves")

    # Remove the default File SOP that Houdini creates inside new geo nodes
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
    # 2. Convert to polylines
    #    Option A: Convert SOP (converts polygon faces to lines)
    #    Option B: Change Grid surftype to "Rows & Columns"
    #    Using Convert SOP for clarity.
    # ----------------------------------------------------------
    convert = geo.createNode("convert", "convert_to_lines")
    convert.setInput(0, grid)

    # ----------------------------------------------------------
    # 3. Fuse SOP — snap shared vertices at intersections
    # ----------------------------------------------------------
    fuse = geo.createNode("fuse", "snap_shared_points")
    # Try both old and new parameter names for compatibility
    dist_parm = fuse.parm("dist") or fuse.parm("snapdist")
    if dist_parm:
        dist_parm.set(0.001)
    fuse.setInput(0, convert)

    # ----------------------------------------------------------
    # 4. Classify intersections (Attribute Wrangle)
    # ----------------------------------------------------------
    classify = geo.createNode("attribwrangle", "classify_intersections")
    classify.parm("snippet").set(CLASSIFY_VEX)
    classify.parm("class").set(2)  # Run Over: Points
    classify.setInput(0, fuse)

    # ----------------------------------------------------------
    # 5. Blast SOPs — isolate each intersection type
    #    negate=1 means "delete everything NOT in the group" (keep group)
    # ----------------------------------------------------------
    blast_4way = geo.createNode("blast", "isolate_4way")
    blast_4way.parm("group").set("type_4way")
    blast_4way.parm("negate").set(1)
    blast_4way.setInput(0, classify)

    blast_3way = geo.createNode("blast", "isolate_3way")
    blast_3way.parm("group").set("type_3way")
    blast_3way.parm("negate").set(1)
    blast_3way.setInput(0, classify)

    blast_corner = geo.createNode("blast", "isolate_corners")
    blast_corner.parm("group").set("type_corner")
    blast_corner.parm("negate").set(1)
    blast_corner.setInput(0, classify)

    # ----------------------------------------------------------
    # 6. Curve generation wrangles
    # ----------------------------------------------------------
    gen_4way = geo.createNode("attribwrangle", "gen_4way_curves")
    gen_4way.parm("snippet").set(GEN_4WAY_VEX)
    gen_4way.parm("class").set(2)  # Points
    gen_4way.setInput(0, blast_4way)
    add_curve_params(gen_4way)

    gen_3way = geo.createNode("attribwrangle", "gen_3way_curves")
    gen_3way.parm("snippet").set(GEN_3WAY_VEX)
    gen_3way.parm("class").set(2)
    gen_3way.setInput(0, blast_3way)
    add_curve_params(gen_3way)

    gen_corner = geo.createNode("attribwrangle", "gen_corner_curves")
    gen_corner.parm("snippet").set(GEN_CORNER_VEX)
    gen_corner.parm("class").set(2)
    gen_corner.setInput(0, blast_corner)
    add_curve_params(gen_corner)

    # ----------------------------------------------------------
    # 7. Merge all curve branches
    # ----------------------------------------------------------
    merge_curves = geo.createNode("merge", "merge_all_curves")
    merge_curves.setInput(0, gen_4way)
    merge_curves.setInput(1, gen_3way)
    merge_curves.setInput(2, gen_corner)

    # ----------------------------------------------------------
    # 8. Resample — smooth out polylines
    # ----------------------------------------------------------
    resample = geo.createNode("resample", "smooth_curves")
    resample.parm("length").set(2)
    # Enable "Maintain Last Vertex"
    last_vtx_parm = resample.parm("maintainlast")
    if last_vtx_parm:
        last_vtx_parm.set(1)
    resample.setInput(0, merge_curves)

    # ----------------------------------------------------------
    # 9. Color by curve type (Attribute Wrangle)
    # ----------------------------------------------------------
    color_wrangle = geo.createNode("attribwrangle", "color_by_type")
    color_wrangle.parm("snippet").set(COLOR_VEX)
    color_wrangle.parm("class").set(2)  # Points
    color_wrangle.setInput(0, resample)

    # ----------------------------------------------------------
    # 10. Final merge: original grid lines + colored curves
    # ----------------------------------------------------------
    final_merge = geo.createNode("merge", "final_output")
    final_merge.setInput(0, convert)       # Original grid wireframe
    final_merge.setInput(1, color_wrangle) # Generated + colored curves

    # ----------------------------------------------------------
    # Set display / render flags
    # ----------------------------------------------------------
    final_merge.setDisplayFlag(True)
    final_merge.setRenderFlag(True)

    # ----------------------------------------------------------
    # Auto-layout the network
    # ----------------------------------------------------------
    geo.layoutChildren()

    # ----------------------------------------------------------
    # Done
    # ----------------------------------------------------------
    print("=" * 60)
    print("Traffic Flow Curves network created successfully!")
    print("  Node: {}".format(geo.path()))
    print("")
    print("  Intersections:")
    print("    4-way (interior):  9   -> 54 curves")
    print("    3-way (edge):     12   -> 36 curves")
    print("    Corner:            4   ->  4 curves")
    print("    Total:            25   -> 94 curves")
    print("")
    print("  Adjust parameters on each gen_*_curves wrangle:")
    print("    Road Width   (default 35)")
    print("    Turn Radius  (default 30)")
    print("    Arc Segments (default 16)")
    print("    Chevron Size (default  8)")
    print("=" * 60)

    return geo


# ============================================================
# RUN
# ============================================================
# Paste this entire script into Houdini's Python Shell and
# it will auto-execute. Or source it from the Script Editor.

create_traffic_flow()
