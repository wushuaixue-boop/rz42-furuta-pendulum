# Equations of motion and parameter identification

| File | Content |
|---|---|
| `01_kane_model.ipynb` | Equations of motion of the RZ42 Furuta pendulum with Kane's method (`sympy.physics.mechanics`), then: the standard form M q'' + C q' + G + F = τ, natural frequencies, simulation, comparison with MuJoCo, linearisation, and the link to the identification. |
| `EXPERIMENT_PROTOCOL.md` | **Lab procedure for the physical rig.** Firmware logging, bench values, and every experiment, with the rig fully assembled and no torque measurement. |
| `02_parameter_identification.ipynb` | **The identification.** Encoder logs → AS5600 zero → derivatives → linear LS → output-error fit of κ, ρ, ω², β, φ → repeatability → absolute scale from the added mass → validation on independent runs → controller model. It never reads the hidden plant or the ground truth. |
| `03_evaluate_against_truth.ipynb` | **Simulation study only.** Compares `identified_parameters.json` with `../dataset/ground_truth.json`. Not used on real hardware. |
| `identified_parameters.json` | Output of 02: ratios, repeatability, absolute values, pendulum COM, what is not identifiable, upright model (A, B) for the controller, validation scores. |
| `figures/` | `validation.png` (from 02) and `parameter_errors.png` (from 03). |

Run from this folder with `python -m notebook`:
- notebook 01 takes about a minute;
- notebook 02 takes about a minute;
- notebook 03 takes a second.
