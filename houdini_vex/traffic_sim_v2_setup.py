"""
Traffic Sim V2 - Solver-Based (RBD-Ready)
==========================================
Run in Houdini 21 Python Shell:
   exec(open("/path/to/traffic_sim_v2_setup.py").read())

Creates /obj/traffic_sim_v2 with full node network.
"""

import hou

# ============================================================
# VEX SNIPPETS
# ============================================================

VEX_GEN_ROUTES = r"""
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
"""


VEX_INIT_VEHICLES = r"""
float max_speed = ch("vehicle_speed");
float spacing   = ch("vehicle_spacing");
float y_off     = ch("vehicle_offset");
int   seed      = chi("random_seed");
float str_dens  = ch("straight_density");
float turn_dens = ch("turn_density");

float prim_len = primintrinsic(0, "measuredperimeter", @primnum);
if(prim_len < spacing * 0.5) return;

// Use different density for straight vs turn routes
string rtype = s@route_type;
float density;
if(rtype == "straight") {
    density = str_dens;
} else {
    density = turn_dens;
}

float expected = prim_len / spacing * density;
int num_v;
if(expected >= 1.0) {
    num_v = int(floor(expected));
} else {
    num_v = (random(@primnum * 0.731 + seed * 0.137) < expected) ? 1 : 0;
}
if(num_v < 1) { removeprim(0, @primnum, 1); return; }

// Generate random u values with minimum gap
float uvals[];
for(int v = 0; v < num_v; v++) {
    float u = random(@primnum * 7.13 + v * 3.91 + seed * 0.53 + v * seed * 0.17);
    u = fit01(u, 0.03, 0.97);
    append(uvals, u);
}
// Sort to enforce minimum spacing
for(int i = 0; i < len(uvals) - 1; i++) {
    for(int j = i + 1; j < len(uvals); j++) {
        if(uvals[j] < uvals[i]) {
            float tmp = uvals[i];
            uvals[i] = uvals[j];
            uvals[j] = tmp;
        }
    }
}
float min_u_gap = spacing / max(prim_len, 1.0);
int valid[];
append(valid, 1);
for(int i = 1; i < len(uvals); i++) {
    if(uvals[i] - uvals[i-1] >= min_u_gap) {
        append(valid, 1);
    } else {
        append(valid, 0);
    }
}

for(int v = 0; v < len(uvals); v++) {
    if(!valid[v]) continue;
    float u = uvals[v];

    vector pos = primuv(0, "P", @primnum, set(u, 0, 0));
    pos.y += y_off;

    float eps = 0.005;
    vector p_fwd = primuv(0, "P", @primnum, set(min(u + eps, 0.999), 0, 0));
    vector p_bck = primuv(0, "P", @primnum, set(max(u - eps, 0.001), 0, 0));
    vector tang = normalize(p_fwd - p_bck);

    int vid = @primnum * 1000 + v;

    int pt = addpoint(0, pos);
    setpointattrib(0, "N",            pt, tang,       "set");
    setpointattrib(0, "up",           pt, {0, 1, 0},  "set");
    setpointattrib(0, "u_param",      pt, u,          "set");
    setpointattrib(0, "speed",        pt, max_speed,   "set");
    setpointattrib(0, "target_speed", pt, max_speed,   "set");
    setpointattrib(0, "brake",        pt, 0.0,         "set");
    setpointattrib(0, "route_id",     pt, @primnum,    "set");
    setpointattrib(0, "vehicle_id",   pt, vid,         "set");
    setpointattrib(0, "prim_length",  pt, prim_len,    "set");
    setpointattrib(0, "curve_type",   pt, "vehicle",   "set");
}

removeprim(0, @primnum, 1);
"""


VEX_SOLVER_STEP = r"""
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

float dt = @TimeInc;
if(dt <= 0) dt = 1.0 / 24.0;

float u         = f@u_param;
float cur_speed = f@speed;
vector my_pos   = @P;
vector my_dir   = @N;
int   my_vid    = i@vehicle_id;
int   my_route  = i@route_id;
float prim_len  = f@prim_length;

int num_route_prims = nprimitives(1);
if(num_route_prims == 0 || my_route < 0 || my_route >= num_route_prims) {
    f@brake = 1.0;
    return;
}

float target = max_speed;

// --- PHASE 1: Same-lane following ---
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
    if(room <= 0.5) {
        target = 0.0;
    } else if(room < look_dist * 0.7) {
        float ratio = clamp(room / (look_dist * 0.7), 0.0, 1.0);
        target = min(target, max_speed * smooth(0, 1, ratio));
    }
}

// --- PHASE 2: Cross-traffic ---
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
    int bit = xor_val & 1;
    int i_yield = bit ? (my_route > nb_route) : (my_route < nb_route);
    if(!i_yield) continue;
    vector to_nb = nb_pos - my_pos;
    float dist = length(to_nb);
    float my_proj = dot(to_nb, my_dir);
    float nb_proj = dot(-to_nb, nb_dir);
    if(my_proj < -safe_dist) continue;
    if(nb_proj < -safe_dist * 2.0) continue;
    float my_t = max(my_proj, 0.0) / max(cur_speed, 0.01);
    float nb_spd = point(0, "speed", nb);
    float nb_t = max(nb_proj, 0.0) / max(nb_spd, 0.01);
    vector my_cross = my_pos + my_dir * max(my_proj, 0.0);
    vector nb_cross = nb_pos + nb_dir * max(nb_proj, 0.0);
    float gap = length(my_cross - nb_cross);
    int is_threat = 0;
    if(dist < safe_dist * 1.5 && dot(normalize(to_nb), my_dir) > -0.3) is_threat = 1;
    else if(gap < safe_dist * 2.5 && abs(my_t - nb_t) < cross_time) is_threat = 1;
    else if(gap < safe_dist * 2.0 && my_proj > 0 && nb_proj > 0 && abs(my_t - nb_t) < cross_time * 1.5) is_threat = 1;
    if(!is_threat) continue;
    float stop_room = max(my_proj - safe_dist, 0.0);
    if(stop_room <= 0.5) target = 0.0;
    else target = min(target, max_speed * smooth(0, 1, clamp(stop_room / cross_dist, 0.0, 1.0)));
}

// --- PHASE 3: Speed adjustment ---
if(cur_speed < target) cur_speed = min(cur_speed + accel * dt, target);
else if(cur_speed > target) cur_speed = max(cur_speed - decel * dt, target);
cur_speed = clamp(cur_speed, 0.0, max_speed);

// --- PHASE 4: Advance ---
if(prim_len > 0.01 && cur_speed > 0.001) {
    float du = cur_speed * dt / prim_len;
    u += du;
    if(u > 1.0) u -= 1.0;
    if(u < 0.0) u += 1.0;
    u = clamp(u, 0.001, 0.999);
}

// --- PHASE 5: Position from route curve (input 1) ---
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
f@brake = 1.0 - clamp(cur_speed / max(max_speed, 0.01), 0.0, 1.0);
"""


VEX_COLOR = r"""
float b = f@brake;
if(b < 0.01) @Cd = {0.15, 0.85, 0.25};
else if(b < 0.5) { float t = b / 0.5; @Cd = lerp({0.15, 0.85, 0.25}, {1.0, 0.85, 0.1}, t); }
else { float t = (b - 0.5) / 0.5; @Cd = lerp({1.0, 0.85, 0.1}, {0.95, 0.12, 0.1}, t); }
"""


# ============================================================
# HELPER: Add spare parameters to a wrangle node
# ============================================================
def add_parms(node, parms):
    """parms = list of (name, label, type, default)
       type: 'f' = float, 'i' = int
    """
    ptg = node.parmTemplateGroup()
    for name, label, ptype, default in parms:
        if ptype == 'f':
            ptg.append(hou.FloatParmTemplate(name, label, 1, [default]))
        elif ptype == 'i':
            ptg.append(hou.IntParmTemplate(name, label, 1, [default]))
    node.setParmTemplateGroup(ptg)


# ============================================================
# MAIN SETUP
# ============================================================
def create_traffic_sim():
    obj = hou.node("/obj")

    # Remove existing
    old = obj.node("traffic_sim_v2")
    if old:
        old.destroy()

    # Create geo container
    geo = obj.createNode("geo", "traffic_sim_v2")
    # Remove default file node
    for child in geo.children():
        child.destroy()

    # -------------------------------------------------------
    # 1. ROUTE GENERATION
    # -------------------------------------------------------
    gen_routes = geo.createNode("attribwrangle", "gen_vehicle_routes")
    gen_routes.parm("class").set(0)  # Detail
    gen_routes.parm("snippet").set(VEX_GEN_ROUTES)
    add_parms(gen_routes, [
        ("grid_size",      "Grid Size",      'f', 348.0),
        ("grid_divisions", "Grid Divisions", 'i', 4),
        ("lane_spacing",   "Lane Spacing",   'f', 3.5),
        ("entry_dist",     "Entry Distance", 'f', 20.0),
        ("arc_segments",   "Arc Segments",   'i', 16),
    ])

    # Resample for smoother primuv interpolation
    resample = geo.createNode("resample", "resample_routes")
    resample.setInput(0, gen_routes)
    resample.parm("dolength").set(1)
    resample.parm("length").set(2.0)

    # Null for clean reference
    route_null = geo.createNode("null", "route_curves")
    route_null.setInput(0, resample)

    # -------------------------------------------------------
    # 2. INIT VEHICLES
    # -------------------------------------------------------
    init = geo.createNode("attribwrangle", "init_vehicles")
    init.setInput(0, route_null)
    init.parm("class").set(1)  # Primitives
    init.parm("snippet").set(VEX_INIT_VEHICLES)
    add_parms(init, [
        ("vehicle_speed",    "Vehicle Speed",     'f', 15.0),
        ("vehicle_spacing",  "Vehicle Spacing",   'f', 30.0),
        ("straight_density", "Straight Density",  'f', 0.4),
        ("turn_density",     "Turn Density",      'f', 0.08),
        ("vehicle_offset",   "Vehicle Offset",    'f', 0.5),
        ("random_seed",      "Random Seed",       'i', 42),
    ])

    # -------------------------------------------------------
    # 3. SOLVER
    # -------------------------------------------------------
    solver = geo.createNode("solver", "traffic_solver")
    solver.setInput(0, init)

    # --- Inside the Solver ---
    # Find the prev_frame node (different names in different H versions)
    prev = None
    for child in solver.children():
        cname = child.name().lower()
        if "prev" in cname or cname == "d" or "input" in cname:
            if child.type().name() != "output":
                prev = child
                break
    if prev is None:
        # Fallback: use the first non-output child
        for child in solver.children():
            if child.type().name() != "output":
                prev = child
                break

    # Create Object Merge to bring in route curves
    obj_merge = solver.createNode("object_merge", "route_curves_ref")
    obj_merge.parm("numobj").set(1)
    obj_merge.parm("objpath1").set(route_null.path())
    obj_merge.parm("xformtype").set(1)  # Into this object

    # Create solver step wrangle
    step = solver.createNode("attribwrangle", "solver_step")
    if prev:
        step.setInput(0, prev)
    step.setInput(1, obj_merge)
    step.parm("class").set(2)  # Points
    step.parm("snippet").set(VEX_SOLVER_STEP)
    add_parms(step, [
        ("max_speed",         "Max Speed",         'f', 15.0),
        ("acceleration",      "Acceleration",      'f', 8.0),
        ("deceleration",      "Deceleration",      'f', 25.0),
        ("look_ahead_dist",   "Look Ahead Dist",   'f', 30.0),
        ("cross_detect_dist", "Cross Detect Dist", 'f', 35.0),
        ("min_safe_dist",     "Min Safe Dist",     'f', 10.0),
        ("cross_safe_time",   "Cross Safe Time",   'f', 1.5),
        ("search_count",      "Search Count",      'i', 150),
        ("vehicle_offset",    "Vehicle Offset",    'f', 0.5),
    ])

    # Wire solver output
    # Find output node inside solver
    output_node = None
    for child in solver.children():
        if child.type().name() == "output":
            output_node = child
            break
    if output_node:
        output_node.setInput(0, step)
    else:
        # No output node — set display/render flags
        step.setDisplayFlag(True)
        step.setRenderFlag(True)

    solver.layoutChildren()

    # -------------------------------------------------------
    # 4. COLOR
    # -------------------------------------------------------
    color = geo.createNode("attribwrangle", "color_vehicles")
    color.setInput(0, solver)
    color.parm("class").set(2)  # Points
    color.parm("snippet").set(VEX_COLOR)

    # -------------------------------------------------------
    # 5. BOX (simple car)
    # -------------------------------------------------------
    box = geo.createNode("box", "car_box")
    box.parmTuple("size").set((4.0, 1.5, 2.0))
    box.parmTuple("t").set((0.0, 0.75, 0.0))

    # -------------------------------------------------------
    # 6. COPY TO POINTS
    # -------------------------------------------------------
    copy = geo.createNode("copytopoints::2.0", "copy_cars")
    copy.setInput(0, box)
    copy.setInput(1, color)
    copy.parm("useimplicitn").set(1)

    # Set display
    copy.setDisplayFlag(True)
    copy.setRenderFlag(True)

    # Layout
    geo.layoutChildren()

    # -------------------------------------------------------
    # VERIFICATION
    # -------------------------------------------------------
    print("=" * 50)
    print("Traffic Sim V2 created at: " + geo.path())
    print("=" * 50)
    print("")
    print("Node network:")
    print("  gen_vehicle_routes -> resample_routes -> route_curves (Null)")
    print("  route_curves -> init_vehicles -> traffic_solver")
    print("  traffic_solver -> color_vehicles -> copy_cars (DISPLAY)")
    print("")
    print("Inside solver:")
    if prev:
        print("  " + prev.name() + " -> solver_step (input 0)")
    print("  route_curves_ref (Object Merge) -> solver_step (input 1)")
    print("  Object Merge path: " + route_null.path())
    print("")
    print("Route prims: check route_curves node")
    print("")
    print("HOW TO RUN:")
    print("  1. Go to frame 1")
    print("  2. Press Play")
    print("  3. Vehicles should move and stop at intersections")
    print("")
    print("IF VEHICLES DON'T MOVE:")
    print("  - Dive into traffic_solver")
    print("  - Check route_curves_ref (Object Merge) resolves")
    print("  - Verify solver_step has 2 inputs connected")
    print("  - Check the Object Merge path: " + route_null.path())
    print("")
    print("CHANGING PARAMS:")
    print("  - After changing init_vehicles params, go to frame 1 first!")
    print("  - The solver resets from initial state at frame 1")
    print("=" * 50)

    return geo

# RUN
create_traffic_sim()
