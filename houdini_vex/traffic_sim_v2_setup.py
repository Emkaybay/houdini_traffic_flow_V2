"""
Traffic Sim V2 - FIXED (Route Switching + Anti-Collision)
=========================================================
Run in Houdini 21 Python Shell:
   exec(open(r"C:\path\to\traffic_sim_v2_setup.py").read())

FIXES:
  - Solver HDA unlocked before editing
  - Route switching at endpoints (vehicles turn at intersections!)
  - Vehicles never stop (always find a new route)
  - init_vehicles runs over Detail (no duplication bug)
  - Proper anti-collision with actual speed control
"""

import hou

# ============================================================
# VEX: Route Generation (unchanged from your working version)
# ============================================================
VEX_GEN_ROUTES = r'''
float grid_size  = ch("grid_size");
int   grid_div   = chi("grid_divisions");
float lane_space = ch("lane_spacing");
float entry_dist = ch("entry_dist");
int   arc_segs   = chi("arc_segments");

float cell = grid_size / float(grid_div);
float half = grid_size / 2.0;
int   num_roads = grid_div + 1;
float inner_off = lane_space * 0.5;
float outer_off = lane_space * 1.5;

function vector bezier4(vector p0; vector p1; vector p2; vector p3; float t) {
    float u = 1.0 - t;
    return u*u*u*p0 + 3.0*u*u*t*p1 + 3.0*u*t*t*p2 + t*t*t*p3;
}

for(int row = 0; row < num_roads; row++) {
    float zb = -half + float(row) * cell;
    int pr;
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set(-half, 0, zb + inner_off)));
    addvertex(0, pr, addpoint(0, set( half, 0, zb + inner_off)));
    setprimattrib(0, "route_type", pr, "straight", "set");
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set(-half, 0, zb + outer_off)));
    addvertex(0, pr, addpoint(0, set( half, 0, zb + outer_off)));
    setprimattrib(0, "route_type", pr, "straight", "set");
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set( half, 0, zb - inner_off)));
    addvertex(0, pr, addpoint(0, set(-half, 0, zb - inner_off)));
    setprimattrib(0, "route_type", pr, "straight", "set");
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set( half, 0, zb - outer_off)));
    addvertex(0, pr, addpoint(0, set(-half, 0, zb - outer_off)));
    setprimattrib(0, "route_type", pr, "straight", "set");
}
for(int col = 0; col < num_roads; col++) {
    float xb = -half + float(col) * cell;
    int pr;
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set(xb - inner_off, 0, -half)));
    addvertex(0, pr, addpoint(0, set(xb - inner_off, 0,  half)));
    setprimattrib(0, "route_type", pr, "straight", "set");
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set(xb - outer_off, 0, -half)));
    addvertex(0, pr, addpoint(0, set(xb - outer_off, 0,  half)));
    setprimattrib(0, "route_type", pr, "straight", "set");
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set(xb + inner_off, 0,  half)));
    addvertex(0, pr, addpoint(0, set(xb + inner_off, 0, -half)));
    setprimattrib(0, "route_type", pr, "straight", "set");
    pr = addprim(0, "polyline");
    addvertex(0, pr, addpoint(0, set(xb + outer_off, 0,  half)));
    addvertex(0, pr, addpoint(0, set(xb + outer_off, 0, -half)));
    setprimattrib(0, "route_type", pr, "straight", "set");
}
vector card_dirs[];
append(card_dirs, set( 1, 0, 0));
append(card_dirs, set(-1, 0, 0));
append(card_dirs, set( 0, 0, 1));
append(card_dirs, set( 0, 0,-1));
for(int row = 0; row < num_roads; row++) {
    float iz = -half + float(row) * cell;
    for(int col = 0; col < num_roads; col++) {
        float ix = -half + float(col) * cell;
        vector center = set(ix, 0, iz);
        for(int ai = 0; ai < 4; ai++) {
            vector d = card_dirs[ai];
            vector travel = -d;
            vector right_d = set(-travel.z, 0, travel.x);
            vector left_d  = -right_d;
            int has_app = 1;
            if(d.x >  0.5 && col >= grid_div) has_app = 0;
            if(d.x < -0.5 && col <= 0)        has_app = 0;
            if(d.z >  0.5 && row >= grid_div) has_app = 0;
            if(d.z < -0.5 && row <= 0)        has_app = 0;
            if(!has_app) continue;
            int can_right = 0, can_left = 0;
            if(right_d.x >  0.5 && col < grid_div) can_right = 1;
            if(right_d.x < -0.5 && col > 0)        can_right = 1;
            if(right_d.z >  0.5 && row < grid_div) can_right = 1;
            if(right_d.z < -0.5 && row > 0)        can_right = 1;
            if(left_d.x  >  0.5 && col < grid_div) can_left  = 1;
            if(left_d.x  < -0.5 && col > 0)        can_left  = 1;
            if(left_d.z  >  0.5 && row < grid_div) can_left  = 1;
            if(left_d.z  < -0.5 && row > 0)        can_left  = 1;
            if(can_right) {
                vector new_right = set(-right_d.z, 0, right_d.x);
                vector arc_s = center + d * entry_dist + right_d * outer_off;
                vector arc_e = center + right_d * entry_dist + new_right * outer_off;
                float h = entry_dist * 0.45;
                vector cp1 = arc_s - d * h;
                vector cp2 = arc_e - right_d * h;
                vector app_edge, ext_edge;
                if(abs(d.x) > 0.5) app_edge = set(d.x > 0 ? half : -half, 0, arc_s.z);
                else                app_edge = set(arc_s.x, 0, d.z > 0 ? half : -half);
                if(abs(right_d.x) > 0.5) ext_edge = set(right_d.x > 0 ? half : -half, 0, arc_e.z);
                else                      ext_edge = set(arc_e.x, 0, right_d.z > 0 ? half : -half);
                int prim = addprim(0, "polyline");
                addvertex(0, prim, addpoint(0, app_edge));
                for(int s = 0; s <= arc_segs; s++) {
                    float t = float(s) / float(arc_segs);
                    addvertex(0, prim, addpoint(0, bezier4(arc_s, cp1, cp2, arc_e, t)));
                }
                addvertex(0, prim, addpoint(0, ext_edge));
                setprimattrib(0, "route_type", prim, "right_turn", "set");
            }
            if(can_left) {
                vector new_right_l = set(-left_d.z, 0, left_d.x);
                vector arc_s = center + d * entry_dist + right_d * inner_off;
                vector arc_e = center + left_d * entry_dist + new_right_l * inner_off;
                float h = entry_dist * 0.72;
                vector cp1 = arc_s - d * h;
                vector cp2 = arc_e - left_d * h;
                vector app_edge, ext_edge;
                if(abs(d.x) > 0.5) app_edge = set(d.x > 0 ? half : -half, 0, arc_s.z);
                else                app_edge = set(arc_s.x, 0, d.z > 0 ? half : -half);
                if(abs(left_d.x) > 0.5) ext_edge = set(left_d.x > 0 ? half : -half, 0, arc_e.z);
                else                     ext_edge = set(arc_e.x, 0, left_d.z > 0 ? half : -half);
                int prim = addprim(0, "polyline");
                addvertex(0, prim, addpoint(0, app_edge));
                for(int s = 0; s <= arc_segs; s++) {
                    float t = float(s) / float(arc_segs);
                    addvertex(0, prim, addpoint(0, bezier4(arc_s, cp1, cp2, arc_e, t)));
                }
                addvertex(0, prim, addpoint(0, ext_edge));
                setprimattrib(0, "route_type", prim, "left_turn", "set");
            }
        }
    }
}
'''

# ============================================================
# VEX: Init Vehicles (runs over DETAIL — no duplication)
# ============================================================
VEX_INIT = r'''
float max_speed = ch("vehicle_speed");
float spacing   = ch("vehicle_spacing");
float y_off     = ch("vehicle_offset");
int   seed      = chi("random_seed");
float str_dens  = ch("straight_density");
float turn_dens = ch("turn_density");

int num_prims = nprimitives(0);

for(int p = 0; p < num_prims; p++) {
    float prim_len = primintrinsic(0, "measuredperimeter", p);
    if(prim_len < spacing * 0.5) continue;

    string rtype = prim(0, "route_type", p);
    float density = (rtype == "straight") ? str_dens : turn_dens;

    float expected = prim_len / spacing * density;
    int num_v = (expected >= 1.0) ? int(floor(expected)) : ((random(p * 0.731 + seed * 0.137) < expected) ? 1 : 0);
    if(num_v < 1) continue;

    for(int v = 0; v < num_v; v++) {
        float u = random(p * 7.13 + v * 3.91 + seed * 0.53);
        u = fit01(u, 0.05, 0.95);

        vector pos = primuv(0, "P", p, set(u, 0, 0));
        pos.y += y_off;

        float eps = 0.005;
        vector p_fwd = primuv(0, "P", p, set(min(u + eps, 0.999), 0, 0));
        vector p_bck = primuv(0, "P", p, set(max(u - eps, 0.001), 0, 0));
        vector tang = normalize(p_fwd - p_bck);

        int vid = p * 1000 + v;
        int pt = addpoint(0, pos);
        setpointattrib(0, "N",            pt, tang,       "set");
        setpointattrib(0, "up",           pt, {0, 1, 0},  "set");
        setpointattrib(0, "u_param",      pt, u,          "set");
        setpointattrib(0, "speed",        pt, max_speed,   "set");
        setpointattrib(0, "target_speed", pt, max_speed,   "set");
        setpointattrib(0, "brake",        pt, 0.0,         "set");
        setpointattrib(0, "route_id",     pt, p,           "set");
        setpointattrib(0, "vehicle_id",   pt, vid,         "set");
        setpointattrib(0, "prim_length",  pt, prim_len,    "set");
        setpointattrib(0, "curve_type",   pt, "vehicle",   "set");
    }
}

// Remove all source prims (vehicle points are standalone, they survive)
for(int p = num_prims - 1; p >= 0; p--) {
    removeprim(0, p, 1);
}
'''

# ============================================================
# VEX: Solver Step (WITH route switching + anti-collision)
# ============================================================
VEX_SOLVER = r'''
if(s@curve_type != "vehicle") return;

float max_speed  = ch("max_speed");
float accel      = ch("acceleration");
float decel      = ch("deceleration");
float look_dist  = ch("look_ahead_dist");
float cross_dist = ch("cross_detect_dist");
float safe_dist  = ch("min_safe_dist");
float cross_time = ch("cross_safe_time");
int   max_search = chi("search_count");
float y_off      = ch("vehicle_offset");
float match_dist = ch("route_match_dist");

float dt = @TimeInc;
if(dt <= 0) dt = 1.0 / 24.0;

float u         = f@u_param;
float cur_speed = f@speed;
vector my_pos   = @P;
vector my_dir   = @N;
int   my_vid    = i@vehicle_id;
int   my_route  = i@route_id;
float prim_len  = f@prim_length;

int num_routes = nprimitives(1);
if(num_routes == 0 || my_route < 0 || my_route >= num_routes) {
    f@brake = 1.0;
    return;
}

float target = max_speed;

// =======================================================
// PHASE 1: Same-lane following
// =======================================================
vector fwd_center = my_pos + my_dir * (look_dist * 0.4);
int fwd_nearby[] = pcfind(0, "P", fwd_center, look_dist, max_search);
float closest_follow = 1e9;

foreach(int nb; fwd_nearby) {
    if(point(0, "curve_type", nb) != "vehicle") continue;
    if(point(0, "vehicle_id", nb) == my_vid) continue;
    vector nb_pos = point(0, "P", nb);
    vector nb_dir = point(0, "N", nb);
    if(dot(my_dir, nb_dir) < 0.4) continue;
    vector to_nb = nb_pos - my_pos;
    float fwd = dot(to_nb, my_dir);
    if(fwd < 0.3) continue;
    float lat = length(to_nb - my_dir * fwd);
    if(lat > safe_dist * 0.6) continue;
    if(fwd < closest_follow) closest_follow = fwd;
}
if(closest_follow < 1e8) {
    float room = closest_follow - safe_dist;
    if(room <= 0.5) target = 0.0;
    else if(room < look_dist * 0.7) {
        target = min(target, max_speed * smooth(0, 1, clamp(room / (look_dist * 0.7), 0, 1)));
    }
}

// =======================================================
// PHASE 2: Cross-traffic
// =======================================================
int cross_nearby[] = pcfind(0, "P", my_pos, cross_dist, max_search);
foreach(int nb; cross_nearby) {
    if(point(0, "curve_type", nb) != "vehicle") continue;
    if(point(0, "vehicle_id", nb) == my_vid) continue;
    int nb_route = point(0, "route_id", nb);
    if(nb_route == my_route) continue;
    vector nb_pos = point(0, "P", nb);
    vector nb_dir = point(0, "N", nb);
    if(dot(my_dir, nb_dir) < -0.7) continue;
    int xor_val = my_route ^ nb_route;
    int i_yield = (xor_val & 1) ? (my_route > nb_route) : (my_route < nb_route);
    if(!i_yield) continue;
    vector to_nb = nb_pos - my_pos;
    float dist = length(to_nb);
    float my_proj = dot(to_nb, my_dir);
    float nb_proj = dot(-to_nb, nb_dir);
    if(my_proj < -safe_dist || nb_proj < -safe_dist * 2) continue;
    float my_t = max(my_proj, 0) / max(cur_speed, 0.01);
    float nb_t = max(nb_proj, 0) / max(point(0, "speed", nb), 0.01);
    vector my_cross = my_pos + my_dir * max(my_proj, 0);
    vector nb_cross = nb_pos + nb_dir * max(nb_proj, 0);
    float gap = length(my_cross - nb_cross);
    int is_threat = 0;
    if(dist < safe_dist * 1.5 && dot(normalize(to_nb), my_dir) > -0.3) is_threat = 1;
    else if(gap < safe_dist * 2.5 && abs(my_t - nb_t) < cross_time) is_threat = 1;
    else if(gap < safe_dist * 2 && my_proj > 0 && nb_proj > 0 && abs(my_t - nb_t) < cross_time * 1.5) is_threat = 1;
    if(!is_threat) continue;
    float stop_room = max(my_proj - safe_dist, 0);
    if(stop_room <= 0.5) target = 0;
    else target = min(target, max_speed * smooth(0, 1, clamp(stop_room / cross_dist, 0, 1)));
}

// =======================================================
// PHASE 3: Speed adjustment
// =======================================================
if(cur_speed < target) cur_speed = min(cur_speed + accel * dt, target);
else if(cur_speed > target) cur_speed = max(cur_speed - decel * dt, target);
cur_speed = clamp(cur_speed, 0, max_speed);

// =======================================================
// PHASE 4: Advance
// =======================================================
if(prim_len > 0.01 && cur_speed > 0.001) {
    u += cur_speed * dt / prim_len;
}

// =======================================================
// PHASE 5: ROUTE SWITCHING (the key fix!)
// When vehicle reaches the end of its route, find a new
// route that starts near the same point. This is how
// vehicles turn at intersections.
// =======================================================
if(u >= 0.97) {
    // Get endpoint of current route
    vector end_pos = primuv(1, "P", my_route, set(0.999, 0, 0));

    // Find routes whose START is near our END
    int candidates[];
    for(int r = 0; r < num_routes; r++) {
        if(r == my_route) continue;
        vector r_start = primuv(1, "P", r, set(0.001, 0, 0));
        if(length(r_start - end_pos) < match_dist) {
            append(candidates, r);
        }
    }

    if(len(candidates) > 0) {
        // Pick a random route from candidates
        int idx = int(floor(random(my_vid * 0.37 + @Frame * 0.13) * float(len(candidates))));
        idx = clamp(idx, 0, len(candidates) - 1);
        int new_route = candidates[idx];

        // Switch to the new route
        i@route_id = new_route;
        f@u_param = 0.02;
        f@prim_length = primintrinsic(1, "measuredperimeter", new_route);

        // Update position on new route
        vector new_pos = primuv(1, "P", new_route, set(0.02, 0, 0));
        new_pos.y += y_off;
        @P = new_pos;

        // Update direction
        vector nf = primuv(1, "P", new_route, set(0.025, 0, 0));
        vector nb = primuv(1, "P", new_route, set(0.015, 0, 0));
        @N = normalize(nf - nb);

        f@speed = cur_speed;
        f@brake = 1.0 - clamp(cur_speed / max(max_speed, 0.01), 0, 1);
        return;
    } else {
        // No matching route — loop on same route
        u = 0.02;
    }
}

// Clamp u
u = clamp(u, 0.001, 0.999);

// =======================================================
// PHASE 6: Update position from route curve (input 1)
// =======================================================
vector new_pos = primuv(1, "P", my_route, set(u, 0, 0));
new_pos.y += y_off;
float eps = 0.005;
vector p_fwd = primuv(1, "P", my_route, set(min(u + eps, 0.999), 0, 0));
vector p_bck = primuv(1, "P", my_route, set(max(u - eps, 0.001), 0, 0));
vector tang = normalize(p_fwd - p_bck);

@P = new_pos;
@N = tang;
f@u_param = u;
f@speed = cur_speed;
f@target_speed = target;
f@brake = 1.0 - clamp(cur_speed / max(max_speed, 0.01), 0, 1);
'''

# ============================================================
# VEX: Color
# ============================================================
VEX_COLOR = r'''
float b = f@brake;
if(b < 0.01) @Cd = {0.15, 0.85, 0.25};
else if(b < 0.5) { float t = b / 0.5; @Cd = lerp({0.15, 0.85, 0.25}, {1.0, 0.85, 0.1}, t); }
else { float t = (b - 0.5) / 0.5; @Cd = lerp({1.0, 0.85, 0.1}, {0.95, 0.12, 0.1}, t); }
'''


# ============================================================
# HELPER
# ============================================================
def add_parms(node, parms):
    ptg = node.parmTemplateGroup()
    for name, label, ptype, default in parms:
        if ptype == 'f':
            ptg.append(hou.FloatParmTemplate(name, label, 1, [default]))
        elif ptype == 'i':
            ptg.append(hou.IntParmTemplate(name, label, 1, [default]))
    node.setParmTemplateGroup(ptg)


# ============================================================
# MAIN
# ============================================================
def create_traffic_sim():
    obj = hou.node("/obj")
    old = obj.node("traffic_sim_v2")
    if old:
        old.destroy()

    geo = obj.createNode("geo", "traffic_sim_v2")
    for child in geo.children():
        child.destroy()

    # --- 1. Routes ---
    gen_routes = geo.createNode("attribwrangle", "gen_vehicle_routes")
    gen_routes.parm("class").set(0)
    gen_routes.parm("snippet").set(VEX_GEN_ROUTES)
    add_parms(gen_routes, [
        ("grid_size", "Grid Size", 'f', 348.0),
        ("grid_divisions", "Grid Divisions", 'i', 4),
        ("lane_spacing", "Lane Spacing", 'f', 3.5),
        ("entry_dist", "Entry Distance", 'f', 20.0),
        ("arc_segments", "Arc Segments", 'i', 16),
    ])

    resample = geo.createNode("resample", "resample_routes")
    resample.setInput(0, gen_routes)
    resample.parm("dolength").set(1)
    resample.parm("length").set(2.0)

    route_null = geo.createNode("null", "route_curves")
    route_null.setInput(0, resample)

    # --- 2. Init Vehicles (runs over DETAIL) ---
    init = geo.createNode("attribwrangle", "init_vehicles")
    init.setInput(0, route_null)
    init.parm("class").set(0)  # DETAIL — runs once, loops internally
    init.parm("snippet").set(VEX_INIT)
    add_parms(init, [
        ("vehicle_speed", "Vehicle Speed", 'f', 15.0),
        ("vehicle_spacing", "Vehicle Spacing", 'f', 30.0),
        ("straight_density", "Straight Density", 'f', 0.35),
        ("turn_density", "Turn Density", 'f', 0.0),
        ("vehicle_offset", "Vehicle Offset", 'f', 0.5),
        ("random_seed", "Random Seed", 'i', 42),
    ])

    # --- 3. Solver ---
    solver = geo.createNode("solver", "traffic_solver")
    solver.setInput(0, init)

    # UNLOCK the solver HDA
    solver.allowEditingOfContents()

    # Find internal nodes
    prev = None
    output_node = None
    for child in solver.children():
        ctype = child.type().name()
        cname = child.name().lower()
        if ctype == "output":
            output_node = child
        elif prev is None and ctype != "output":
            prev = child

    print("Solver internals:")
    for child in solver.children():
        print("  {} ({})".format(child.name(), child.type().name()))

    # Object Merge for route curves (FULL path, no relative)
    obj_merge = solver.createNode("object_merge", "route_curves_ref")
    obj_merge.parm("numobj").set(1)
    obj_merge.parm("objpath1").set(route_null.path())
    obj_merge.parm("xformtype").set(1)

    # Solver step wrangle
    step = solver.createNode("attribwrangle", "solver_step")
    if prev:
        step.setInput(0, prev)
    step.setInput(1, obj_merge)
    step.parm("class").set(2)  # Points
    step.parm("snippet").set(VEX_SOLVER)
    add_parms(step, [
        ("max_speed", "Max Speed", 'f', 15.0),
        ("acceleration", "Acceleration", 'f', 8.0),
        ("deceleration", "Deceleration", 'f', 25.0),
        ("look_ahead_dist", "Look Ahead Dist", 'f', 30.0),
        ("cross_detect_dist", "Cross Detect Dist", 'f', 35.0),
        ("min_safe_dist", "Min Safe Dist", 'f', 10.0),
        ("cross_safe_time", "Cross Safe Time", 'f', 1.5),
        ("search_count", "Search Count", 'i', 150),
        ("vehicle_offset", "Vehicle Offset", 'f', 0.5),
        ("route_match_dist", "Route Match Dist", 'f', 12.0),
    ])

    # Wire output
    if output_node:
        output_node.setInput(0, step)
    else:
        step.setDisplayFlag(True)
        step.setRenderFlag(True)

    solver.layoutChildren()

    # --- 4. Color ---
    color = geo.createNode("attribwrangle", "color_vehicles")
    color.setInput(0, solver)
    color.parm("class").set(2)
    color.parm("snippet").set(VEX_COLOR)

    # --- 5. Box ---
    box = geo.createNode("box", "car_box")
    box.parmTuple("size").set((4.0, 1.5, 2.0))
    box.parmTuple("t").set((0.0, 0.75, 0.0))

    # --- 6. Copy to Points ---
    copy = geo.createNode("copytopoints::2.0", "copy_cars")
    copy.setInput(0, box)
    copy.setInput(1, color)
    copy.parm("useimplicitn").set(1)

    copy.setDisplayFlag(True)
    copy.setRenderFlag(True)
    geo.layoutChildren()

    print("\n" + "=" * 50)
    print("Traffic Sim V2 created: " + geo.path())
    print("Object Merge path: " + route_null.path())
    print("")
    print("HOW IT WORKS:")
    print("  - Vehicles follow routes using persistent speed in solver")
    print("  - At route endpoints, vehicles SWITCH to a new route")
    print("  - This is how they turn at intersections!")
    print("  - route_match_dist controls how routes connect (default 3.0)")
    print("")
    print("TO RUN: Go to frame 1, press Play")
    print("AFTER CHANGING init_vehicles params: go to frame 1 first!")
    print("=" * 50)

    return geo

create_traffic_sim()
