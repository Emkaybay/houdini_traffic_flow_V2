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

v6: 90% car reduction + vehicle awareness (same-route + cross-route)
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

# === v6c: Vehicle generation with 24-frame prediction polylines ===
GEN_VEHICLES_VEX = r"""float speed   = ch("vehicle_speed");
float spacing = ch("vehicle_spacing");
float y_off   = ch("vehicle_offset");
float density = ch("vehicle_density");
int pred_frames = chi("pred_frames");

float prim_len = primintrinsic(0, "measuredperimeter", @primnum);
if(prim_len < spacing * 0.5) return;

float expected = prim_len / spacing * density;
int num_v;
if(expected >= 1.0) {
    num_v = int(floor(expected));
} else {
    num_v = (random(@primnum * 0.731) < expected) ? 1 : 0;
}
if(num_v < 1) return;

float phase = random(@primnum);
float fps = 24.0;
float dt = 1.0 / fps;
float du_per_frame = speed * dt / prim_len;

for(int v = 0; v < num_v; v++) {
    float base_u = (float(v) + 0.5) / float(num_v);
    float animated_u = base_u + phase + @Time * speed / prim_len;
    animated_u = animated_u - floor(animated_u);

    vector pos = primuv(0, "P", @primnum, set(animated_u, 0, 0));
    pos.y += y_off;

    float du = 0.005;
    float u_fwd = min(animated_u + du, 0.999);
    float u_bck = max(animated_u - du, 0.001);
    vector p_fwd = primuv(0, "P", @primnum, set(u_fwd, 0, 0));
    vector p_bck = primuv(0, "P", @primnum, set(u_bck, 0, 0));
    vector tang = normalize(p_fwd - p_bck);

    int vid = @primnum * 1000 + v;

    int pt = addpoint(0, pos);
    setpointattrib(0, "N", pt, tang, "set");
    setpointattrib(0, "up", pt, set(0, 1, 0), "set");
    setpointattrib(0, "curve_type", pt, "vehicle", "set");
    setpointattrib(0, "route_id", pt, @primnum, "set");
    setpointattrib(0, "vehicle_id", pt, vid, "set");
    setpointgroup(0, "vehicles", pt, 1);

    // Prediction polyline (debug viz + collision data — follows actual curve)
    int pred_prim = addprim(0, "polyline");
    setprimattrib(0, "is_prediction", pred_prim, 1, "set");
    for(int f = 0; f <= pred_frames; f++) {
        float future_u = animated_u + float(f) * du_per_frame;
        future_u = future_u - floor(future_u);
        future_u = clamp(future_u, 0.001, 0.999);
        vector fpos = primuv(0, "P", @primnum, set(future_u, 0, 0));
        fpos.y += y_off + 0.1;
        int fpt = addpoint(0, fpos);
        setpointattrib(0, "curve_type", fpt, "prediction", "set");
        setpointattrib(0, "route_id", fpt, @primnum, "set");
        setpointattrib(0, "vehicle_id", fpt, vid, "set");
        setpointattrib(0, "frame_offset", fpt, f, "set");
        setpointgroup(0, "predictions", fpt, 1);
        addvertex(0, pred_prim, fpt);
    }
}
"""

# === v6c: Prediction-based collision avoidance ===
VEHICLE_AWARENESS_VEX = r"""float stop_dist    = ch("stop_distance");
float coll_thresh  = ch("collision_threshold");
float v_speed      = ch("vehicle_speed");

if(s@curve_type != "vehicle") return;

vector my_pos   = @P;
vector my_dir   = @N;
int    my_route = i@route_id;
int    my_vid   = i@vehicle_id;

int    pred_frames = 24;
float  fps = 24.0;
float  dt  = 1.0 / fps;

float search_radius = v_speed * 1.5;
int nearby[] = pcfind(0, "P", my_pos, search_radius, 500);

// Collect my predictions and others' predictions
vector my_preds[];
int    my_preds_ok[];
resize(my_preds, pred_frames + 1);
resize(my_preds_ok, pred_frames + 1);
for(int i = 0; i <= pred_frames; i++) my_preds_ok[i] = 0;

int    o_frame[];
vector o_pos[];
int    o_route[];

foreach(int nb; nearby) {
    string t = point(0, "curve_type", nb);
    if(t != "prediction") continue;
    int    vid   = point(0, "vehicle_id", nb);
    int    frame = point(0, "frame_offset", nb);
    vector pos   = point(0, "P", nb);
    int    route = point(0, "route_id", nb);
    if(vid == my_vid) {
        if(frame >= 0 && frame <= pred_frames) {
            my_preds[frame] = pos;
            my_preds_ok[frame] = 1;
        }
    } else {
        append(o_frame, frame);
        append(o_pos, pos);
        append(o_route, route);
    }
}

// Find earliest predicted collision (frame-by-frame path comparison)
float earliest_ttc = float(pred_frames + 1);
vector coll_point  = my_pos;

int num_others = len(o_frame);
for(int i = 0; i < num_others; i++) {
    int f = o_frame[i];
    if(f < 0 || f > pred_frames) continue;
    if(!my_preds_ok[f]) continue;
    float dist = length(o_pos[i] - my_preds[f]);
    if(dist >= coll_thresh) continue;
    vector to_coll = o_pos[i] - my_pos;
    if(dot(normalize(to_coll), my_dir) < 0.0) continue;
    int nb_route = o_route[i];
    if(nb_route != my_route) {
        int xor_val = my_route ^ nb_route;
        int higher_yields = xor_val & 1;
        int i_yield = higher_yields ? (my_route > nb_route) : (my_route < nb_route);
        if(!i_yield) continue;
    }
    if(float(f) < earliest_ttc) {
        earliest_ttc = float(f);
        coll_point = o_pos[i];
    }
}

// Decelerate based on distance-to-collision
if(earliest_ttc <= float(pred_frames)) {
    float dist_along = dot(coll_point - my_pos, my_dir);
    dist_along = max(dist_along, 0.0);
    float room = max(dist_along - stop_dist, 0.0);
    float frame_travel = v_speed * dt;
    float frames_room  = room / max(frame_travel, 0.001);
    float brake_horizon = 18.0;
    float brake = 0.0;
    if(frames_room < brake_horizon) {
        brake = 1.0 - frames_room / brake_horizon;
        brake = clamp(brake, 0.0, 1.0);
        brake = smooth(0, 1, brake);
    }
    if(brake > 0.01) {
        float pullback = brake * frame_travel;
        @P -= @N * pullback;
    }
}
"""

GEN_ROUTES_VEX = r"""float grid_size  = ch("grid_size");
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

// Straight-through routes (edge to edge at lane centres)
for(int row = 0; row < num_roads; row++) {
    float zb = -half + float(row) * cell;
    int pr = addprim(0, "polyline");
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
    int pr = addprim(0, "polyline");
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

// Turning routes (edge -> arc -> edge)
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

            int can_right = 0;
            int can_left  = 0;
            if(right_d.x >  0.5 && col < grid_div) can_right = 1;
            if(right_d.x < -0.5 && col > 0)        can_right = 1;
            if(right_d.z >  0.5 && row < grid_div) can_right = 1;
            if(right_d.z < -0.5 && row > 0)        can_right = 1;

            if(left_d.x >  0.5 && col < grid_div) can_left = 1;
            if(left_d.x < -0.5 && col > 0)        can_left = 1;
            if(left_d.z >  0.5 && row < grid_div) can_left = 1;
            if(left_d.z < -0.5 && row > 0)        can_left = 1;

            if(can_right) {
                vector new_right = set(-right_d.z, 0, right_d.x);
                vector arc_s = center + d * entry_dist + right_d * outer_off;
                vector arc_e = center + right_d * entry_dist + new_right * outer_off;
                float r_rad = entry_dist - outer_off;
                float h = r_rad * 0.5523;
                vector cp1 = arc_s - d * h;
                vector cp2 = arc_e - right_d * h;

                vector app_edge;
                if(abs(d.x) > 0.5) app_edge = set(d.x > 0 ? half : -half, 0, arc_s.z);
                else                app_edge = set(arc_s.x, 0, d.z > 0 ? half : -half);
                vector ext_edge;
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
                float l_rad = entry_dist + inner_off;
                float h = l_rad * 0.5523;
                vector cp1 = arc_s - d * h;
                vector cp2 = arc_e - left_d * h;

                vector app_edge;
                if(abs(d.x) > 0.5) app_edge = set(d.x > 0 ? half : -half, 0, arc_s.z);
                else                app_edge = set(arc_s.x, 0, d.z > 0 ? half : -half);
                vector ext_edge;
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


def add_vehicle_params(node, vehicle_speed=15.0, vehicle_spacing=30.0,
                       vehicle_offset=0.75, vehicle_density=0.1, pred_frames=24):
    """v6c: Added pred_frames for prediction lookahead."""
    ptg = node.parmTemplateGroup()
    folder = hou.FolderParmTemplate("vehicle_params", "Vehicle Parameters")

    folder.addParmTemplate(hou.FloatParmTemplate(
        "vehicle_speed", "Vehicle Speed (m/s)", 1,
        default_value=(vehicle_speed,), min=1.0, max=50.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "vehicle_spacing", "Vehicle Spacing (m)", 1,
        default_value=(vehicle_spacing,), min=5.0, max=100.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "vehicle_offset", "Vehicle Y Offset", 1,
        default_value=(vehicle_offset,), min=0.0, max=5.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "vehicle_density", "Vehicle Density (0.1 = 10%)", 1,
        default_value=(vehicle_density,), min=0.01, max=1.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.IntParmTemplate(
        "pred_frames", "Prediction Frames", 1,
        default_value=(pred_frames,), min=1, max=48,
        min_is_strict=False, max_is_strict=False))

    ptg.append(folder)
    node.setParmTemplateGroup(ptg)


def add_awareness_params(node, stop_distance=8.0, collision_threshold=5.0,
                         vehicle_speed=15.0):
    """v6c: Prediction-based awareness — stop_dist, collision threshold, speed."""
    ptg = node.parmTemplateGroup()
    folder = hou.FolderParmTemplate("awareness_params", "Awareness Parameters")

    folder.addParmTemplate(hou.FloatParmTemplate(
        "stop_distance", "Stop Distance (m)", 1,
        default_value=(stop_distance,), min=1.0, max=20.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "collision_threshold", "Collision Threshold (m)", 1,
        default_value=(collision_threshold,), min=1.0, max=15.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.FloatParmTemplate(
        "vehicle_speed", "Vehicle Speed (m/s)", 1,
        default_value=(vehicle_speed,), min=1.0, max=50.0,
        min_is_strict=False, max_is_strict=False))

    ptg.append(folder)
    node.setParmTemplateGroup(ptg)


def add_route_params(node, grid_size=348.0, grid_divisions=4,
                     lane_spacing=3.5, entry_dist=20.0, arc_segments=16):
    ptg = node.parmTemplateGroup()
    folder = hou.FolderParmTemplate("route_params", "Route Parameters")

    folder.addParmTemplate(hou.FloatParmTemplate(
        "grid_size", "Grid Size", 1,
        default_value=(grid_size,), min=10.0, max=1000.0,
        min_is_strict=False, max_is_strict=False))
    folder.addParmTemplate(hou.IntParmTemplate(
        "grid_divisions", "Grid Divisions", 1,
        default_value=(grid_divisions,), min=1, max=20,
        min_is_strict=False, max_is_strict=False))
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

    # === VEHICLE FLOW (v6: density reduction + awareness) ===
    route_null = geo.createNode("null", "route_null")

    gen_routes = geo.createNode("attribwrangle", "gen_vehicle_routes")
    gen_routes.parm("snippet").set(GEN_ROUTES_VEX)
    gen_routes.parm("class").set(0)  # Detail
    gen_routes.setInput(0, route_null)
    add_route_params(gen_routes)

    # Resample for smooth primuv interpolation
    resample_routes = geo.createNode("resample", "resample_routes")
    resample_routes.parm("length").set(2.0)
    resample_routes.setInput(0, gen_routes)

    # Generate animated vehicle points + prediction polylines (v6c)
    gen_vehicles = geo.createNode("attribwrangle", "gen_vehicle_points")
    gen_vehicles.parm("snippet").set(GEN_VEHICLES_VEX)
    gen_vehicles.parm("class").set(1)  # Run over: Primitives
    gen_vehicles.setInput(0, resample_routes)
    add_vehicle_params(gen_vehicles)

    # === v6c: Awareness BEFORE blast (needs prediction points for collision detection) ===
    vehicle_awareness = geo.createNode("attribwrangle", "vehicle_awareness")
    vehicle_awareness.parm("snippet").set(VEHICLE_AWARENESS_VEX)
    vehicle_awareness.parm("class").set(2)  # Run over: Points
    vehicle_awareness.setInput(0, gen_vehicles)
    add_awareness_params(vehicle_awareness)

    # After awareness: separate vehicles from predictions
    blast_keep_veh = geo.createNode("blast", "keep_vehicles_only")
    blast_keep_veh.parm("group").set("vehicles")
    blast_keep_veh.parm("negate").set(1)    # delete everything EXCEPT vehicles
    blast_keep_veh.parm("grouptype").set(3)  # operate on points
    blast_keep_veh.setInput(0, vehicle_awareness)

    blast_keep_pred = geo.createNode("blast", "keep_predictions_only")
    blast_keep_pred.parm("group").set("predictions")
    blast_keep_pred.parm("negate").set(1)
    blast_keep_pred.parm("grouptype").set(3)
    blast_keep_pred.setInput(0, vehicle_awareness)

    # Car box geometry (width 2m, height 1.5m, length 4m)
    car_box = geo.createNode("box", "car_box")
    car_box.parm("sizex").set(2.0)
    car_box.parm("sizey").set(1.5)
    car_box.parm("sizez").set(4.0)

    # Copy box to each vehicle point (oriented by N + up)
    copy_cars = geo.createNode("copytopoints", "copy_cars")
    copy_cars.setInput(0, car_box)
    copy_cars.setInput(1, blast_keep_veh)

    # Color vehicles blue
    color_veh = geo.createNode("color", "color_vehicles")
    color_veh.parm("colorr").set(0.2)
    color_veh.parm("colorg").set(0.5)
    color_veh.parm("colorb").set(1.0)
    color_veh.setInput(0, copy_cars)

    # Color prediction debug lines (cyan)
    color_pred = geo.createNode("color", "color_predictions")
    color_pred.parm("colorr").set(0.0)
    color_pred.parm("colorg").set(0.85)
    color_pred.parm("colorb").set(1.0)
    color_pred.setInput(0, blast_keep_pred)

    # === ROAD VISUALIZATION (unchanged) ===
    final_merge = geo.createNode("merge", "final_output")
    final_merge.setInput(0, blast_keep_lanes)
    final_merge.setInput(1, resample)
    final_merge.setInput(2, convert)

    color = geo.createNode("attribwrangle", "color_visualization")
    color.parm("snippet").set(COLOR_VEX)
    color.parm("class").set(2)
    color.setInput(0, final_merge)

    # === DISPLAY: road viz + vehicles + prediction debug lines ===
    display_merge = geo.createNode("merge", "display_output")
    display_merge.setInput(0, color)
    display_merge.setInput(1, color_veh)
    display_merge.setInput(2, color_pred)

    display_merge.setDisplayFlag(True)
    display_merge.setRenderFlag(True)

    geo.layoutChildren()

    print("=" * 60)
    print("Traffic Flow Curves + Predictive Awareness (v6c)")
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
    print("    Blue   = vehicles (animated boxes)")
    print("    Cyan   = prediction debug lines (24-frame lookahead)")
    print("")
    print("  v6c — Predictive Vehicle Awareness:")
    print("    24-frame prediction paths per vehicle (curves on turns)")
    print("    Frame-by-frame collision detection via pcfind")
    print("    Distance-based braking: 18 frames ramp, Hermite S-curve")
    print("    Priority hash: deterministic per route-pair")
    print("    Pullback = brake * one_frame_travel (max ~0.6m, no jumps)")
    print("")
    print("  Adjustable nodes:")
    print("    gen_vehicle_points  — density, speed, spacing, pred_frames")
    print("    vehicle_awareness   — stop_distance, collision_threshold, speed")
    print("")
    print("  Tip: Set frame range to 1-240 (10 sec at 24fps)")
    print("=" * 60)

    return geo

create_traffic_flow()
