# tools

These are Linux helpers. They are **not needed on Windows**, where everything runs through the `mujoco` Python package.

| File | Purpose | Build |
|---|---|---|
| `mj_runner.c` | Headless runner used to generate the identification dataset (open-loop arm commands). | `gcc -O2 -o mj_runner mj_runner.c -I<mujoco>/include -L<mujoco>/lib -lmujoco -lm -Wl,-rpath,<mujoco>/lib`, then `cd ../mujoco && python run_experiments.py --runner ../tools/mj_runner` |
| `mjshim.c` | Tiny C shim over the MuJoCo C API. `control/plant.py` falls back to it when the `mujoco` Python package is not installed; the closed loop was tested with it here. | `gcc -O2 -shared -fPIC -o libmjshim.so mjshim.c -I<mujoco>/include -L<mujoco>/lib -lmujoco -Wl,-rpath,<mujoco>/lib` |
| `render_replay.c` | Renders a logged closed-loop run (`results/*.csv`) to MP4 with MuJoCo's offscreen renderer. Replay only: no physics. | `gcc -O2 -o render_replay render_replay.c -I<mujoco>/include -L<mujoco>/lib -lmujoco -lEGL -lm -Wl,-rpath,<mujoco>/lib`, then `./render_replay ../mujoco/rz42_real_plant.xml ../results/demo_log.csv ../results/demo.mp4` |
