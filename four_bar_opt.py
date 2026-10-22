"""
Dual four-bar (fb1, fb2) synthesis: both rocker points move approximately along y on x = X_LINE,
in opposite directions:
    forward motor stroke: fb1 end moves -y (>= STROKE1), fb2 end lifts +y (>= STROKE2)
    backward motor stroke: fb1 end lifts, fb2 end moves -y
    fb2 end starts START_OFFSET below fb1 end

Coordinate system (units: mm in optimization, m in visualization):
    O1 = (0, 0)       shared crank pivot (motor flange), origin
    O2_k = (0, f_k)   rocker pivot of fb_k, on +y axis
    A  = crank tip,   B = coupler/rocker joint
    P  = working point rigidly attached to the rocker:
         |O2P| = p, angle(O2B -> O2P) = beta

Both cranks are fixed on the same flange:
    theta_fb1 = theta,  theta_fb2 = theta + delta
The motor turns theta from theta0 by motor_range (direction dirn = +-1) and back.

Per-linkage variables [f, a, b, c, p, beta, sigma]
    f     frame            70 ~ 100
    a     crank            10 ~ 30
    b     connecting rod   10 ~ 80
    c     rocker           10 ~ 50
    p     |O2P|            10 ~ 50
    beta  offset angle     -pi ~ pi
    sigma assembly branch  {-1, +1}
Shared variables [theta0, delta, dirn, motor_range]

Note: P is on the rocker, so it moves on an arc centered at O2 (on the y axis);
the straight line x = X_LINE is approximated by an arc segment near x = -p.
"""
import numpy as np
from scipy.optimize import differential_evolution

# ---------------------------------------------------------------------------
# problem setup
# ---------------------------------------------------------------------------
FB_BOUNDS = [(70., 100.),  # f frame
             (10., 30.),  # a crank
             (10., 80.),  # b connecting rod
             (10., 50.),  # c rocker
             (10., 50.),  # p distance O2->P
             (-np.pi, np.pi),  # beta
             (-1., 1.)]  # sigma (sign taken)
SHARED_BOUNDS = [(-np.pi, np.pi),  # theta0 motor start angle
                 (-np.pi, np.pi),  # delta, fb2 crank angle relative to fb1
                 (-1., 1.),  # dirn, motor turning direction (sign taken)
                 (np.radians(45.), np.pi)]  # motor working stroke (angle)
BOUNDS = FB_BOUNDS + FB_BOUNDS + SHARED_BOUNDS
X_LINE = -30.  # required x of the end points
# forward motor stroke: fb1 end moves -y, fb2 end moves +y (lifts); reversed on the way back
STROKE1 = 25.  # required minimum -y travel of fb1 end
STROKE2 = 25.  # required minimum +y travel of fb2 end
START_OFFSET = -25.  # required y(fb2 start) - y(fb1 start)
N_MOTOR = 91  # samples over the motor stroke
MU_MIN = np.radians(40.)  # minimum allowed transmission angle
W_MU = 50.  # weight of transmission-angle penalty (mm / rad)
W_MONO = 20.  # weight of non-monotonic (+y) motion penalty
W_STROKE = 5.  # weight of stroke shortage penalty
W_CRANK_DIR = 20.  # weight of crank pointing to -x penalty (cranks must stay at x >= 0)
W_OFFSET = 5.  # weight of start offset error
JOINT_X_RANGE = (-15., 20.)  # allowed x of the revolute joints (A, B; O1, O2 are on the y axis) during motion
W_JOINT_X = 20.  # weight of joint x out-of-range penalty
JOINT_Y_MARGIN = 0.  # moving joints (A, B) must stay at y <= O2_y - JOINT_Y_MARGIN during motion
W_JOINT_Y = 20.  # weight of joint-above-O2 penalty


def split(x):
    """x -> (fb1 params, fb2 params, theta0, delta, dirn, motor_range)"""
    x = np.asarray(x)
    n = len(FB_BOUNDS)
    return (x[:n], x[n:2 * n], x[2 * n], x[2 * n + 1],
            (1. if x[2 * n + 2] >= 0 else -1.), x[2 * n + 3])


def motor_angles(theta0, dirn, motor_range, n=N_MOTOR):
    return theta0 + dirn * np.linspace(0, motor_range, n)


# ---------------------------------------------------------------------------
# kinematics
# ---------------------------------------------------------------------------
def solve_fourbar(x, thetas):
    """
    x: [f, a, b, c, p, beta, sigma]
    returns dict with A, B, P (N,2), transmission angle mu (N,), valid flag
    valid=False if the linkage cannot be assembled for some crank angle
    """
    f, a, b, c, p, beta, sigma = x
    sigma = 1. if sigma >= 0 else -1.
    o2 = np.array([0., f])
    A = np.stack([a * np.cos(thetas), a * np.sin(thetas)], axis=1)
    v = o2 - A  # A -> O2
    d = np.linalg.norm(v, axis=1)
    # circle(A, b) ∩ circle(O2, c)
    l = (b ** 2 - c ** 2 + d ** 2) / (2 * d)
    h2 = b ** 2 - l ** 2
    valid = bool(np.all(h2 >= 0))
    h = np.sqrt(np.clip(h2, 0, None))
    u = v / d[:, None]
    n = np.stack([-u[:, 1], u[:, 0]], axis=1)
    B = A + l[:, None] * u + sigma * h[:, None] * n
    phi = np.arctan2(B[:, 1] - o2[1], B[:, 0] - o2[0])
    P = o2 + p * np.stack([np.cos(phi + beta), np.sin(phi + beta)], axis=1)
    cos_mu = np.clip((b ** 2 + c ** 2 - d ** 2) / (2 * b * c), -1, 1)
    mu = np.arccos(cos_mu)
    return dict(O1=np.zeros(2), O2=o2, A=A, B=B, P=P, mu=mu, phi=phi, valid=valid)


def solve_dual(x, n=N_MOTOR):
    x1, x2, theta0, delta, dirn, motor_range = split(x)
    thetas = motor_angles(theta0, dirn, motor_range, n)
    return solve_fourbar(x1, thetas), solve_fourbar(x2, thetas + delta)


# ---------------------------------------------------------------------------
# objective
# ---------------------------------------------------------------------------
def fb_cost(sol, ydir, stroke):
    """ydir = -1: end point must move -y over the forward motor stroke, +1: +y"""
    if not sol['valid']:
        return 1e6
    P = sol['P']
    # straightness: max deviation from x = X_LINE
    e_x = np.max(np.abs(P[:, 0] - X_LINE))
    # monotonic motion along ydir: penalize any step against it
    dy = ydir * np.diff(P[:, 1])
    e_mono = np.sum(np.maximum(-dy, 0))
    # stroke shortage
    e_stroke = max(stroke - ydir * (P[-1, 1] - P[0, 1]), 0)
    # transmission angle within [MU_MIN, pi - MU_MIN]
    mu = sol['mu']
    mu_dev = np.maximum(MU_MIN - mu, 0) + np.maximum(mu - (np.pi - MU_MIN), 0)
    # crank points to +x over the whole motor stroke
    e_dir = np.max(np.maximum(-sol['A'][:, 0], 0))
    # revolute joints stay within JOINT_X_RANGE
    jx = np.concatenate([sol['A'][:, 0], sol['B'][:, 0]])
    e_jx = np.max(np.maximum(JOINT_X_RANGE[0] - jx, 0) + np.maximum(jx - JOINT_X_RANGE[1], 0))
    # no moving joint above the rocker pivot O2
    jy = np.concatenate([sol['A'][:, 1], sol['B'][:, 1]])
    e_jy = np.max(np.maximum(jy - (sol['O2'][1] - JOINT_Y_MARGIN), 0))
    return (e_x + W_MONO * e_mono + W_STROKE * e_stroke + W_MU * np.max(mu_dev) + W_CRANK_DIR * e_dir
            + W_JOINT_X * e_jx + W_JOINT_Y * e_jy)


def cost(x):
    sol1, sol2 = solve_dual(x)
    if not (sol1['valid'] and sol2['valid']):
        return 2e6
    e_offset = abs(sol2['P'][0, 1] - sol1['P'][0, 1] - START_OFFSET)
    return fb_cost(sol1, -1, STROKE1) + fb_cost(sol2, +1, STROKE2) + W_OFFSET * e_offset


def optimize(seed=0, maxiter=1000, popsize=30):
    res = differential_evolution(cost, BOUNDS, seed=seed, maxiter=maxiter, popsize=popsize,
                                 tol=0, atol=1e-6, mutation=(0.5, 1.0), recombination=0.9,
                                 init='sobol', polish=True, updating='deferred', workers=-1)
    return res


def report(x):
    x1, x2, theta0, delta, dirn, motor_range = split(x)
    sols = solve_dual(x)
    print("=" * 56)
    print(f"motor: theta0 = {np.degrees(theta0):.2f} deg, range = {np.degrees(motor_range):.1f} deg, "
          f"dir = {'CCW' if dirn > 0 else 'CW'}")
    print(f"fb2 crank offset delta = {np.degrees(delta):.2f} deg")
    print(f"start y offset (fb2 - fb1) = {sols[1]['P'][0, 1] - sols[0]['P'][0, 1]:.2f} mm "
          f"(required {START_OFFSET})")
    for name, xk, sol, ydir in zip(('fb1', 'fb2'), (x1, x2), sols, (-1, +1)):
        f, a, b, c, p, beta, sigma = xk
        P = sol['P']
        print(f"--- {name} ---")
        print(f"  frame f = {f:6.2f}  crank a = {a:6.2f}  coupler b = {b:6.2f}  rocker c = {c:6.2f} mm")
        print(f"  |O2P| p = {p:6.2f} mm, beta = {np.degrees(beta):.2f} deg, branch = {'+1' if sigma >= 0 else '-1'}")
        print(f"  P start = ({P[0, 0]:.2f}, {P[0, 1]:.2f}), end = ({P[-1, 0]:.2f}, {P[-1, 1]:.2f})")
        print(f"  {'-' if ydir < 0 else '+'}y stroke = {ydir * (P[-1, 1] - P[0, 1]):.2f} mm, "
              f"max |x - ({X_LINE})| = {np.max(np.abs(P[:, 0] - X_LINE)):.3f} mm, "
              f"max reverse step = {max((-ydir * np.diff(P[:, 1])).max(), 0):.4f} mm")
        print(f"  mu range = [{np.degrees(sol['mu'].min()):.1f}, {np.degrees(sol['mu'].max()):.1f}] deg")
        crank_ang = np.degrees(np.arctan2(sol['A'][:, 1], sol['A'][:, 0]))
        print(f"  crank angle {crank_ang[0]:.1f} -> {crank_ang[-1]:.1f} deg (min x of A = {sol['A'][:, 0].min():.2f} mm)")
        print(f"  joint x: A in [{sol['A'][:, 0].min():.2f}, {sol['A'][:, 0].max():.2f}], "
              f"B in [{sol['B'][:, 0].min():.2f}, {sol['B'][:, 0].max():.2f}] mm (allowed {JOINT_X_RANGE})")
        print(f"  highest joint y = {max(sol['A'][:, 1].max(), sol['B'][:, 1].max()):.2f} mm "
              f"(O2 y = {sol['O2'][1]:.2f}, margin {JOINT_Y_MARGIN})")
    print("=" * 56)


# ---------------------------------------------------------------------------
# visualization with `one`
# ---------------------------------------------------------------------------
def visualize(x):
    from one import ovw, ossop, oum, ouc
    from imgui_bundle import imgui
    import one.viewer.imgui_overlay as ovio

    MM = 1e-3
    Z = np.array([0., 0., 1.])
    sols = solve_dual(x)
    x1, x2, *_ = split(x)
    # fb2 drawn on a parallel layer behind fb1
    layers = (0., -6.)
    # avoid pure R/G/B, which are used by the coordinate frame axes
    colors = ((ouc.ExtendedColor.GOLD, ouc.ExtendedColor.VIOLET, ouc.ExtendedColor.CHOCOLATE),
              (ouc.ExtendedColor.CARROT_ORANGE, ouc.ExtendedColor.DEEP_PINK, ouc.ExtendedColor.SLATE_GRAY))

    def p3(v2, z=0.):
        return np.array([v2[0] * MM, v2[1] * MM, z * MM], dtype=np.float32)

    base = ovw.World(cam_pos=(-.015, .024, .25), cam_lookat_pos=(-.015, .025, 0))

    # bounding sphere of all links over the motor stroke, for the view buttons
    pts = np.vstack([np.vstack([p3(q, z) for q in np.vstack([s['A'], s['B'], s['P'], [s['O1'], s['O2']]])])
                     for s, z in zip(sols, layers)])
    view_center = (pts.min(axis=0) + pts.max(axis=0)) / 2
    view_dist = 1.2 * np.linalg.norm(pts.max(axis=0) - pts.min(axis=0)) / 2 / np.tan(
        np.radians(base.camera.fov) / 2)

    def front_view():
        """look along -z at the xy plane, y up"""
        base.camera.set_to(pos=view_center + np.array([0, 0, view_dist]), look_at=view_center, up=(0, 1, 0))

    def center_view():
        """keep the current view direction, center and fit the mechanism"""
        d = oum.unit_vec(base.camera.pos - base.camera.look_at, return_length=False)
        base.camera.set_to(pos=view_center + d * view_dist, look_at=view_center)

    def panel():
        imgui.set_next_window_pos((10, 10), imgui.Cond_.first_use_ever)
        imgui.begin('view', flags=imgui.WindowFlags_.always_auto_resize)
        if imgui.button('Front view'):
            front_view()
        imgui.same_line()
        if imgui.button('Center view'):
            center_view()
        imgui.end()

    front_view()
    ovio.ImGuiOverlay(base, panel)
    # arrow head size follows radius_scale, not length_scale
    ossop.frame(length_scale=.15, radius_scale=.1).attach_to(base.scene)

    r_link = 1.2 * MM
    # target line x = X_LINE
    y_lo, y_hi = pts[:, 1].min() / MM - 10., pts[:, 1].max() / MM + 10.
    ossop.cylinder(spos=p3((X_LINE, y_lo)), epos=p3((X_LINE, y_hi)), radius=.3 * MM,
                   rgb=ouc.BasicColor.ORANGE).attach_to(base.scene)
    # allowed x band of the revolute joints
    for jx in JOINT_X_RANGE:
        ossop.cylinder(spos=p3((jx, y_lo)), epos=p3((jx, y_hi)), radius=.2 * MM,
                       rgb=ouc.ExtendedColor.SILVER_GRAY).attach_to(base.scene)

    def make_link(length, rgb, radius=r_link):
        """built along +x with fixed length, re-oriented each frame"""
        o = ossop.cylinder(spos=(0, 0, 0), epos=(length * MM, 0, 0), radius=radius, rgb=rgb)
        o.attach_to(base.scene)
        return o

    def place(link, s2, e2, z):
        s, e = p3(s2, z), p3(e2, z)
        link.set_pos_rotmat(pos=s, rotmat=oum.rotmat_between_vecs(Z, oum.unit_vec(e - s, return_length=False)))

    fbs = []
    for xk, sol, z, (c_crank, c_coupler, c_rocker) in zip((x1, x2), sols, layers, colors):
        a, b, c, p = xk[1:5]
        B, P = sol['B'], sol['P']
        ossop.cylinder(spos=p3(sol['O1'], z), epos=p3(sol['O2'], z), radius=r_link * .8,
                       rgb=ouc.BasicColor.BLACK).attach_to(base.scene)
        ossop.sphere(pos=p3(sol['O2'], z), radius=2.5 * MM, rgb=ouc.BasicColor.BLACK).attach_to(base.scene)
        segs = np.stack([np.array([p3(P[i], z), p3(P[i + 1], z)]) for i in range(len(P) - 1)])
        ossop.linsegs(segs, radius=.4 * MM, srgbs=ouc.BasicColor.MAGENTA).attach_to(base.scene)
        links = dict(crank=make_link(a, c_crank),
                     coupler=make_link(b, c_coupler),
                     rocker_ob=make_link(c, c_rocker),
                     rocker_op=make_link(p, c_rocker),
                     rocker_bp=make_link(np.linalg.norm(B[0] - P[0]), c_rocker, radius=r_link * .6))
        joints = {k: ossop.sphere(radius=2 * MM, rgb=ouc.BasicColor.BLACK) for k in ('A', 'B')}
        joints['P'] = ossop.sphere(radius=2.5 * MM, rgb=ouc.BasicColor.MAGENTA)
        for o in joints.values():
            o.attach_to(base.scene)
        fbs.append((sol, z, links, joints))
    ossop.sphere(pos=p3((0, 0)), radius=3 * MM, rgb=ouc.BasicColor.BLACK).attach_to(base.scene)

    # motor sweeps forward and back over its working range
    sweep = np.r_[np.arange(N_MOTOR), np.arange(N_MOTOR - 2, 0, -1)]
    state = {'k': 0}

    def tick(dt):
        i = sweep[state['k']]
        for sol, z, links, joints in fbs:
            O1, O2, A, B, P = sol['O1'], sol['O2'], sol['A'][i], sol['B'][i], sol['P'][i]
            place(links['crank'], O1, A, z)
            place(links['coupler'], A, B, z)
            place(links['rocker_ob'], O2, B, z)
            place(links['rocker_op'], O2, P, z)
            place(links['rocker_bp'], B, P, z)
            joints['A'].pos = p3(A, z)
            joints['B'].pos = p3(B, z)
            joints['P'].pos = p3(P, z)
        state['k'] = (state['k'] + 1) % len(sweep)

    tick(0)
    base.schedule_interval(tick, interval=1 / 30.)
    base.run()


if __name__ == '__main__':
    res = optimize()
    print(res.message, "cost =", res.fun)
    report(res.x)
    np.save("four_bar_result.npy", res.x)
    visualize(res.x)
