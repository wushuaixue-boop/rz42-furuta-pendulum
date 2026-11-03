# Notice and acknowledgements

This project is based on and inspired by **rotary-inverted-pendulum** by **Henrique Ferrolho**:
<https://github.com/ferrolho/rotary-inverted-pendulum>
(documentation: <https://ferrolho.github.io/rotary-inverted-pendulum/>).

Used from that project:
- the rotary-inverted-pendulum rig concept and the joint/frame conventions of its URDF;
- the 608 pendulum-bearing friction values (viscous 1.74e-5 N m s, Coulomb 3.46e-5 N m) from
  `policy/sysid_params_rig2.json`, used here as the nominal/initial-guess pendulum friction;
- the measured pendulum inertia and natural frequency of its rigs, used to sanity-check the mass model.

Its license notice, as required by the MIT License:

```
MIT License

Copyright (c) 2024 Henrique Ferrolho

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

Other software used: MuJoCo (Apache-2.0, Google DeepMind), OpenCASCADE Technology (LGPL-2.1
with exception) for the CAD generator, SymPy, NumPy, SciPy, Matplotlib.
