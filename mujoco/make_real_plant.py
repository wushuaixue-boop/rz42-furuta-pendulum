"""Build the hidden "real" plant rz42_real_plant.xml from the nominal CAD model rz42_pendulum.xml.

The real printed rig never matches the CAD mass model exactly. This script makes a
MuJoCo model whose masses, centres of mass, inertias and joint friction differ from the
CAD values in physically plausible ways (more filament than the slicer estimate, the coin
glued a few mm lower and off-centre, a greased 608ZZ with more drag, a stepper with some
detent drag).

It also writes two variants with a small known mass taped to the pendulum (the
"added-mass" experiment, done on the assembled rig): rz42_real_plant_mass_d1.xml and
rz42_real_plant_mass_d2.xml.

The identification (../identification/02_parameter_identification.ipynb) never reads these
files. It only sees the encoder logs made from them and the bench measurements in
../dataset/bench.json. The true values go to ../dataset/ground_truth.json and are read only
by ../identification/03_evaluate_against_truth.ipynb.

usage: python make_real_plant.py
"""
import json
import re
from pathlib import Path

HERE = Path(__file__).resolve().parent

TRUE = {
    # arm body (frame at the motor axis, x along the arm)
    "arm_mass": 0.03784,                      # kg  (+11 % vs CAD: denser infill, glue, wires)
    "arm_com": [0.03620, 0.0, 0.01560],       # m
    "arm_I": [5.55e-06, 2.160e-05, 2.115e-05, 0.0, -7.06e-07, 0.0],   # kg m^2 about COM: xx yy zz xy xz yz
    "arm_armature": 7.5e-06,                  # kg m^2, motor rotor
    "arm_damping": 1.5e-04,                   # N m s/rad
    "arm_frictionloss": 8.0e-04,              # N m
    # pendulum body (frame at the 608 centre, x along the hinge, hanging along -z)
    "pen_mass": 0.01812,                      # kg
    "pen_com": [-0.00090, 0.0, -0.04580],     # m   (coin glued ~4 mm lower than CAD)
    "pen_I": [1.680e-05, 1.630e-05, 9.60e-07, 0.0, 1.050e-06, 0.0],   # kg m^2 about COM
    "pen_damping": 2.60e-05,                  # N m s/rad  (608ZZ, greased)
    "pen_frictionloss": 5.20e-05,             # N m
}

# Added mass for the absolute-scale experiment: about 10 g (e.g. two M6 nuts), taped on the pendulum
# centre line. True values here; ../dataset/bench.json has what a scale and calipers would read.
ADDED = {
    "mass": 0.01003,                          # kg
    "radius": 0.0065, "length": 0.0050,       # m, modelled as a small solid cylinder (own inertia included)
    "d1": 0.0253,                             # m below the pivot (identification)
    "d2": 0.0452,                             # m below the pivot (validation)
}


def fmt(v):
    return " ".join(f"{x:.9g}" for x in v)


def main():
    xml = (HERE / "rz42_pendulum.xml").read_text()
    t = TRUE
    # The stepper driver's closed position loop is computed by the experiment runner, outside MuJoCo,
    # and applied as a torque motor, so it acts on the true arm dynamics. (MuJoCo's own <position kv=...>
    # servo with an implicit integrator folds h*kv into the arm inertia, about 5x the real J0.)
    # The torque is never logged: a stepper driver cannot measure it.
    xml = xml.replace('<option timestep="0.001" integrator="implicitfast"', '<option timestep="0.0001" integrator="implicitfast"')
    xml = re.sub(r'<position name="motor"[^>]*/>',
                 '<motor name="motor" joint="arm_joint" gear="1" ctrlrange="-0.45 0.45" ctrllimited="true"/>', xml, flags=re.S)
    xml = xml.replace('</actuator>', '</actuator>\n  <custom>\n    <!-- stepper driver position loop, used by run_experiments.py / mj_runner -->\n'
                      '    <numeric name="servo_kp" data="1000"/>\n    <numeric name="servo_kv" data="0.765516"/>\n'
                      '    <numeric name="servo_tmax" data="0.45"/>\n  </custom>')
    assert '<motor name="motor"' in xml and 'servo_kp' in xml
    xml = xml.replace('<mujoco model="RZ42_v5_rotary_inverted_pendulum">',
                      '<mujoco model="RZ42_v5_real_plant">\n  <!-- HIDDEN "REAL" PLANT for system-identification tests. '
                      'Made by make_real_plant.py; do not use these numbers as an initial guess. -->')
    # arm joint + inertial
    xml = re.sub(r'(<joint name="arm_joint"[^>]*?)armature="[^"]*" damping="[^"]*"',
                 rf'\1armature="{t["arm_armature"]:.9g}" damping="{t["arm_damping"]:.9g}" frictionloss="{t["arm_frictionloss"]:.9g}"',
                 xml, flags=re.S)
    arm_inertial = f'<inertial pos="{fmt(t["arm_com"])}" mass="{t["arm_mass"]:.9g}" fullinertia="{fmt(t["arm_I"])}"/>'
    pen_inertial = f'<inertial pos="{fmt(t["pen_com"])}" mass="{t["pen_mass"]:.9g}" fullinertia="{fmt(t["pen_I"])}"/>'
    inertials = re.findall(r'<inertial [^>]*/>', xml)
    assert len(inertials) == 2, inertials
    xml = xml.replace(inertials[0], arm_inertial).replace(inertials[1], pen_inertial)
    xml = re.sub(r'(<joint name="pendulum_joint"[^>]*?)damping="[^"]*" frictionloss="[^"]*"',
                 rf'\1damping="{t["pen_damping"]:.9g}" frictionloss="{t["pen_frictionloss"]:.9g}"', xml)
    (HERE / "rz42_real_plant.xml").write_text(xml)

    # added-mass variants: a child body welded to the pendulum at (0, 0, -d)
    a = ADDED
    ixx = a["mass"] * a["radius"]**2 / 2                                # cylinder axis along the hinge (x)
    iyy = a["mass"] * (3 * a["radius"]**2 + a["length"]**2) / 12
    for tag in ("d1", "d2"):
        body = (f'<body name="added_mass" pos="0 0 {-a[tag]:.6f}">\n'
                f'          <inertial pos="0 0 0" mass="{a["mass"]:.6g}" diaginertia="{ixx:.6g} {iyy:.6g} {iyy:.6g}"/>\n'
                f'          <geom type="cylinder" size="{a["radius"]} {a["length"]/2}" euler="0 1.5707963 0" rgba="0.7 0.7 0.2 1"/>\n'
                f'        </body>\n        <site name="pendulum_tip"')
        x2 = xml.replace('<site name="pendulum_tip"', body, 1)
        assert 'added_mass' in x2
        (HERE / f"rz42_real_plant_mass_{tag}.xml").write_text(x2)

    # ground truth in the notation of the Kane model (identification/01_kane_model.ipynb)
    r, g = 0.062, 9.81
    m_a, x_a = t["arm_mass"], t["arm_com"][0]
    J_a = t["arm_I"][2] + t["arm_armature"]
    m_p, c_p, l_p = t["pen_mass"], t["pen_com"][0], -t["pen_com"][2]
    I_px, I_py, I_pz, I_pxz = t["pen_I"][0], t["pen_I"][1], t["pen_I"][2], t["pen_I"][4]
    phys = dict(m_a=m_a, x_a=x_a, J_a=J_a, m_p=m_p, l_p=l_p, c_p=c_p,
                I_px=I_px, I_py=I_py, I_pz=I_pz, I_pxz=I_pxz,
                b_a=t["arm_damping"], F_a=t["arm_frictionloss"],
                b_p=t["pen_damping"], F_p=t["pen_frictionloss"])
    lumped = dict(
        J0=J_a + m_a * x_a**2 + m_p * (r + c_p)**2 + I_pz,
        J1=I_py - I_pz + m_p * l_p**2,
        J2=I_px + m_p * l_p**2,
        K=I_pxz + m_p * l_p * (r + c_p),
        G=g * m_p * l_p,
        b_a=phys["b_a"], F_a=phys["F_a"], b_p=phys["b_p"], F_p=phys["F_p"],
    )
    J2, G = lumped["J2"], lumped["G"]
    ratios = {"kappa": lumped["K"] / J2, "rho": lumped["J1"] / J2, "omega2": G / J2,
              "beta": lumped["b_p"] / J2, "phi": lumped["F_p"] / J2}
    out = {"note": "Hidden truth of rz42_real_plant*.xml. Read ONLY by 03_evaluate_against_truth.ipynb.",
           "physical": phys, "lumped": lumped, "ratios": ratios,
           "added_mass": ADDED, "mujoco_values": TRUE}
    (HERE.parent / "dataset").mkdir(exist_ok=True)
    (HERE.parent / "dataset" / "ground_truth.json").write_text(json.dumps(out, indent=2))
    print("wrote rz42_real_plant.xml, rz42_real_plant_mass_d1/d2.xml and ../dataset/ground_truth.json")
    for k, v in lumped.items():
        print(f"  {k:4s} = {v:.6e}")


if __name__ == "__main__":
    main()
