import numpy as np
import matplotlib.pyplot as plt

# ============================================================
# UNITY vs QUEST: STAR VECTOR AND QUATERNION ERROR
#
# Quaternion format: scalar-first Hamilton [w, x, y, z]
# Both quaternions represent ECI -> Body.
#
# Plot 1: Both actual rotated star vectors overlaid, plus a
#         magnified endpoint-difference arrow for visibility.
# Plot 2: Relative quaternion error axis and exaggerated
#         rotation arc. Numerical errors remain unscaled.
# ============================================================

# ------------------------- INPUTS -----------------------------

UNITY_Q = np.array([
    0.704812, -0.056926, 0.056926, 0.704812
], dtype=float)

QUEST_Q = np.array([
    0.704879, -0.0570565, 0.056931, 0.704733
], dtype=float)

# Orion star; edit these values to test another star.
RA_DEG = 83.001666
DEC_DEG = -0.299093

# Visual-only multiplier. Does not change reported errors.
VISUAL_ERROR_SCALE = 1000.0


# ---------------------- MATH FUNCTIONS ------------------------

def normalize(v):
    v = np.asarray(v, dtype=float)
    n = np.linalg.norm(v)
    if n < 1e-15:
        raise ValueError("Cannot normalize a zero-length vector.")
    return v / n


def radec_to_eci(ra_deg, dec_deg):
    """Convert catalogue RA/Dec (degrees) to an ECI unit vector."""
    ra = np.radians(ra_deg)
    dec = np.radians(dec_deg)
    return normalize(np.array([
        np.cos(dec) * np.cos(ra),
        np.cos(dec) * np.sin(ra),
        np.sin(dec)
    ]))


def quat_to_dcm(q):
    """Scalar-first Hamilton quaternion [w,x,y,z] -> active DCM."""
    w, x, y, z = normalize(q)
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w),     2*(x*z + y*w)],
        [2*(x*y + z*w),     1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w),     2*(y*z + x*w),     1 - 2*(x*x + y*y)]
    ])


def quat_multiply(q1, q2):
    """Hamilton product q1 ⊗ q2; scalar-first."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


def quat_conjugate(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def angle_between(v1, v2):
    v1, v2 = normalize(v1), normalize(v2)
    dot = np.clip(np.dot(v1, v2), -1.0, 1.0)
    return np.degrees(np.arccos(dot))


def rotate_about_axis(v, axis, angle):
    """Rodrigues rotation formula."""
    axis = normalize(axis)
    return (
        v * np.cos(angle)
        + np.cross(axis, v) * np.sin(angle)
        + axis * np.dot(axis, v) * (1.0 - np.cos(angle))
    )


# ---------------------- PLOT HELPERS --------------------------

def draw_body_axes(ax, length=1.0):
    """C++ Body convention: +X Right, +Y Down, +Z Forward."""
    style = dict(color="0.30", alpha=0.65, linewidth=1.5,
                 arrow_length_ratio=0.08)
    ax.quiver(0, 0, 0, length, 0, 0, **style)
    ax.quiver(0, 0, 0, 0, length, 0, **style)
    ax.quiver(0, 0, 0, 0, 0, length, **style)

    ax.text(length * 1.06, 0, 0, "+X Right", fontsize=9)
    ax.text(0, length * 1.06, 0, "+Y Down", fontsize=9)
    ax.text(0, 0, length * 1.06, "+Z Forward", fontsize=9)

    ax.set_xlabel("Body X")
    ax.set_ylabel("Body Y")
    ax.set_zlabel("Body Z")
    ax.grid(True, alpha=0.25)


def draw_unit_sphere(ax):
    u = np.linspace(0, 2*np.pi, 36)
    v = np.linspace(0, np.pi, 20)
    x = np.outer(np.cos(u), np.sin(v))
    y = np.outer(np.sin(u), np.sin(v))
    z = np.outer(np.ones_like(u), np.cos(v))
    ax.plot_wireframe(
        x, y, z, color="gray", alpha=0.12, linewidth=0.5
    )


def equal_3d_limits(ax, points, padding=0.22):
    points = np.asarray(points)
    limit = max(float(np.max(np.abs(points))), 1.0) * (1 + padding)
    ax.set_xlim(-limit, limit)
    ax.set_ylim(-limit, limit)
    ax.set_zlim(-limit, limit)
    ax.set_box_aspect((1, 1, 1))


# ------------------- CALCULATE STAR VECTORS -------------------

r_eci = radec_to_eci(RA_DEG, DEC_DEG)

R_unity = quat_to_dcm(UNITY_Q)
R_quest = quat_to_dcm(QUEST_Q)

# Both quaternions map ECI vectors into the C++ Body frame.
b_unity = normalize(R_unity @ r_eci)
b_quest = normalize(R_quest @ r_eci)

vector_error_deg = angle_between(b_unity, b_quest)
delta_vector = b_quest - b_unity
endpoint_difference = np.linalg.norm(delta_vector)


# ----------------- RELATIVE QUATERNION ERROR ------------------

q_unity = normalize(UNITY_Q)
q_quest = normalize(QUEST_Q)

# This relative rotation maps Unity's body-frame result to QUEST's:
# q_error = q_QUEST ⊗ inverse(q_UNITY)
q_error = normalize(
    quat_multiply(q_quest, quat_conjugate(q_unity))
)

# q and -q encode the same rotation. Choose the shorter rotation.
if q_error[0] < 0:
    q_error = -q_error

q_error[0] = np.clip(q_error[0], -1.0, 1.0)
actual_error_rad = 2.0 * np.arccos(q_error[0])
actual_error_deg = np.degrees(actual_error_rad)

sin_half = np.linalg.norm(q_error[1:])
if sin_half > 1e-12:
    error_axis = q_error[1:] / sin_half
else:
    error_axis = np.array([0.0, 0.0, 0.0])

# Only the visualized arc is amplified. Cap it to avoid an unreadable plot.
visual_error_rad = min(
    actual_error_rad * VISUAL_ERROR_SCALE,
    np.radians(120.0)
)


# ------------------------- TEXT OUTPUT ------------------------

print("\\n============== INPUT STAR ==============")
print(f"RA                  : {RA_DEG:.8f} deg")
print(f"Dec                 : {DEC_DEG:.8f} deg")
print("ECI unit vector     :", np.array2string(r_eci, precision=10))

print("\\n========== ROTATED BODY VECTORS ==========")
print("Unity body vector   :", np.array2string(b_unity, precision=10))
print("QUEST body vector   :", np.array2string(b_quest, precision=10))

print("\\n============== VECTOR ERROR ==============")
print(f"Angular separation  : {vector_error_deg:.12f} deg")
print(f"Endpoint difference : {endpoint_difference:.12e} (unit-sphere units)")

print("\\n=========== QUATERNION ERROR =============")
print("Relative q [w,x,y,z]:", np.array2string(q_error, precision=12))
print("Error rotation axis :", np.array2string(error_axis, precision=10))
print(f"Actual angle        : {actual_error_deg:.12f} deg")
print(f"Visual scale        : {VISUAL_ERROR_SCALE:.1f}x")
print(f"Displayed arc angle : {np.degrees(visual_error_rad):.8f} deg")


# ============================================================
# PLOT 1: SUPERIMPOSED STAR VECTORS
# ============================================================

fig1 = plt.figure(figsize=(11, 9))
ax1 = fig1.add_subplot(111, projection="3d")

draw_unit_sphere(ax1)
draw_body_axes(ax1)

# These are the two real, unit-length body-frame star vectors.
ax1.quiver(
    0, 0, 0, *b_unity,
    color="royalblue", linewidth=3,
    arrow_length_ratio=0.10,
    label="Unity rotated vector (actual)"
)
ax1.quiver(
    0, 0, 0, *b_quest,
    color="crimson", linewidth=3,
    arrow_length_ratio=0.10,
    label="QUEST rotated vector (actual)"
)

ax1.scatter(*b_unity, color="royalblue", s=55, depthshade=False)
ax1.scatter(*b_quest, color="crimson", s=55, depthshade=False)

# Magnify only the difference arrow, starting at Unity's endpoint.
# This orange arrow is a visual error indicator, not a physical ray.
visual_delta = VISUAL_ERROR_SCALE * delta_vector
visual_delta_endpoint = b_unity + visual_delta

ax1.quiver(
    *b_unity, *visual_delta,
    color="darkorange", linewidth=3,
    arrow_length_ratio=0.16,
    label=f"Endpoint difference × {VISUAL_ERROR_SCALE:g} (visual only)"
)
ax1.scatter(
    *visual_delta_endpoint,
    color="darkorange", marker="X", s=75,
    depthshade=False,
    label="Magnified difference endpoint"
)

ax1.set_title(
    "UNITY vs QUEST — ROTATED STAR VECTOR\\n"
    f"RA = {RA_DEG:.6f}°, Dec = {DEC_DEG:.6f}°\\n"
    f"Actual angular separation = {vector_error_deg:.10f}°",
    fontsize=13, pad=22
)
ax1.legend(loc="upper left", fontsize=9)
ax1.view_init(elev=24, azim=38)

points1 = np.vstack([
    np.zeros(3), b_unity, b_quest, visual_delta_endpoint
])
equal_3d_limits(ax1, points1)

fig1.text(
    0.5, 0.025,
    "Blue and red are the actual unit vectors. Orange shows the endpoint "
    "difference multiplied for visibility; it is not an actual star vector.",
    ha="center", fontsize=9
)
fig1.tight_layout(rect=(0, 0.055, 1, 1))


# ============================================================
# PLOT 2: QUATERNION ERROR AXIS AND EXAGGERATED ROTATION ARC
# ============================================================

fig2 = plt.figure(figsize=(11, 9))
ax2 = fig2.add_subplot(111, projection="3d")

draw_unit_sphere(ax2)
draw_body_axes(ax2)

# Actual Unity and QUEST vectors for reference.
ax2.quiver(
    0, 0, 0, *b_unity,
    color="royalblue", linewidth=3,
    arrow_length_ratio=0.10,
    label="Unity vector (actual)"
)
ax2.quiver(
    0, 0, 0, *b_quest,
    color="crimson", linewidth=3,
    arrow_length_ratio=0.10,
    label="QUEST vector (actual)"
)

# Actual, very small endpoint separation.
ax2.plot(
    [b_unity[0], b_quest[0]],
    [b_unity[1], b_quest[1]],
    [b_unity[2], b_quest[2]],
    color="black", linestyle=":", linewidth=2,
    label="Actual endpoint difference"
)

plot2_points = [np.zeros(3), b_unity, b_quest]

if np.linalg.norm(error_axis) > 1e-12:
    # Relative quaternion's axis, expressed in the Body frame.
    ax2.quiver(
        0, 0, 0, *error_axis,
        color="purple", linewidth=2.5,
        arrow_length_ratio=0.10,
        label="Relative quaternion error axis"
    )
    plot2_points.append(error_axis)

    # Rotate Unity's vector around the relative error axis.
    arc_angles = np.linspace(0.0, visual_error_rad, 100)
    arc_points = np.array([
        rotate_about_axis(b_unity, error_axis, angle)
        for angle in arc_angles
    ])

    ax2.plot(
        arc_points[:, 0],
        arc_points[:, 1],
        arc_points[:, 2],
        color="darkorange", linewidth=3,
        label=f"Exaggerated rotation arc ({VISUAL_ERROR_SCALE:g}×)"
    )

    exaggerated_endpoint = arc_points[-1]
    ax2.scatter(
        *exaggerated_endpoint,
        color="darkorange", marker="X", s=75,
        depthshade=False,
        label="Exaggerated endpoint (visual only)"
    )
    plot2_points.append(exaggerated_endpoint)

ax2.set_title(
    "RELATIVE QUATERNION ERROR — ROTATION AXIS AND ARC\\n"
    f"Actual error = {actual_error_deg:.10f}° | "
    f"Displayed arc = {np.degrees(visual_error_rad):.6f}°",
    fontsize=13, pad=22
)
ax2.legend(loc="upper left", fontsize=9)
ax2.view_init(elev=24, azim=38)
equal_3d_limits(ax2, np.array(plot2_points))

fig2.text(
    0.5, 0.025,
    "Orange arc is an exaggerated rotation about the relative quaternion "
    "error axis. The printed quaternion error remains the actual value.",
    ha="center", fontsize=9
)
fig2.tight_layout(rect=(0, 0.055, 1, 1))

plt.show()
