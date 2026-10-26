# Validation

What the models were checked against, the numbers, and where they disagree. Differences are recorded here and
turned into config changes, not code hacks. Reproduce: `pytest` (host), `scripts/uw.sh test` (GPU),
`scripts/uw.sh py tools/plume_preview.py` (writes `output/plume_preview/<preset>/stats.json`).

## M1: water, leak, bubbles, plume (2026-10-26, profile `linus`, default config)

### Water and gas (`uw/water.py`, `tests/test_water.py`)

| Quantity (15 °C, 35 psu, surface) | Model | Reference | Difference |
|---|---|---|---|
| Density | 1026.05 kg/m³ | 1025.97 kg/m³, UNESCO EOS-80 (Millero & Poisson 1981) | +0.08 kg/m³ |
| Sound speed (Mackenzie 1981) | 1506.69 m/s | 1506.7 m/s | < 0.1 m/s |
| Absorption at 750 kHz (Ainslie & McColm 1998) | 0.228 dB/m | 0.214 dB/m, Francois & Garrison 1982 | +6.6 % |
| Absorption at 1.2 MHz (HF mode) | 0.476 dB/m | 0.443 dB/m, Francois & Garrison 1982 | +7.5 % |

Both absorption formulas are used near or beyond the top of their fitted range at these frequencies; the 7 %
spread is the uncertainty to carry into M2 (0.03 dB/m over a 3 m two-way path is 0.2 dB).

**Default leak** (methane, 3 bar over the water pressure at the hole, 1 mm hole, Cd 0.62, gas at 15 °C): choked;
mass flow 0.3596 g/s; 25.47 L/min at the hole (2.50 m deep, 127 kPa), 31.8 L/min at 1 atm; exit velocity 412 m/s;
jet momentum flux 0.199 N. `leak.pipe_pressure_bar` is read as the pressure difference across the pipe wall.
Tested: the subsonic and choked branches meet at the critical pressure ratio.

### Bubble slip (Fan & Tsuchiya 1990, contaminated: n = 0.8, c = 1.4, K_b0 = 14.7)

Reference: the Grace correlation for contaminated systems (Grace, Wairegi & Nguyen 1976, as given in Clift, Grace
& Weber 1978, ch. 7): H = 4/3 Eo M^-0.149 (μ/0.0009)^-0.14; J = 0.94 H^0.757 (2 < H ≤ 59.3), 3.42 H^0.441
(H > 59.3); U = μ/(ρd) M^-0.149 (J − 0.857).

| d [mm] | FT, water 20 °C | Grace, water 20 °C | FT, sea 15 °C | Grace, sea 15 °C |
|---|---|---|---|---|
| 1 | 0.142 m/s | 0.118 (FT +20 %) | 0.129 | 0.111 (FT +16 %) |
| 2 | 0.220 | 0.199 (+11 %) | 0.210 | 0.191 (+10 %) |
| 3 | 0.233 | 0.253 (−8 %) | 0.227 | 0.245 (−8 %) |
| 5 | 0.234 | 0.242 (−3 %) | 0.231 | 0.244 (−6 %) |

Reference for a contaminated 1 mm bubble: **0.12 m/s** (Grace correlation, 20 °C water). Small contaminated
bubbles behave like rigid spheres, and the standard drag curve (Schiller & Naumann) gives 0.112 m/s, an
independent check. The earlier plan value of 0.14 m/s was Fan & Tsuchiya's own output at those conditions, not the
reference (`docs/PLAN.md` corrected). For the plume the 1 mm gap hardly matters: most of the gas is in 3–7 mm
bubbles, where the two agree within 8 %.

### Plume regime

| | exposed | buried |
|---|---|---|
| Jet length l_M = M0^(3/4) / B^(1/2) | 2.6 cm | 0 (no momentum) |
| Bubble length scale D = g Q0 / (4π α0² u_s³), α0 = 0.083 (Bombardelli et al. 2007) | 3.75 m | 3.51 m |
| Height of the water column above the source, as z/D | ≤ 0.67 | ≤ 0.75 |

The jet dominates only the first few centimetres. The tank plume is a buoyant plume with slip
(z ≫ l_M, z < D), not the slip-dominated far field where bubble-plume correlations are measured (z/D > 2). The plan's
remark that at 3 m depth "the plume is dominated by the near-source jet" does not hold for this leak. The `basin`
preset (3.6 m column) does not reach the far field either.

### Plume against the literature (`tools/plume_preview.py`, `profiles.png`)

References, as plotted:
1. **MTT pure plume** (Morton, Taylor & Turner 1956), Gaussian form, B = g Q_gas at the source, no slip:
   b = 1.2 α z, u_c = 2 (25 B / (96 π α²))^(1/3) z^(−1/3). `tests/test_plume.py` checks that the integral model
   reproduces it with slip, momentum and expansion off (exponent −1/3 ± 0.005, slope 1.2 α ± 1 %, u_c ± 2 %).
2. **Lai & Socolofsky (2019)**, J. Fluid Mech. 865, 993–1041, bubble plume in unstratified, stationary water:
   U_c / u_s = 1.6 (z/D)^(−1/3), D as above. Fitted for 2 < z/D < 11.
3. **Bubble-plume width**: b_g = 0.114 z (Gaussian 1/e radius), the line through the bubble-plume data including
   Lai & Socolofsky (2019). Bubble core ratio λ = b_bubbles / b_g = 0.8–0.9 (Milgram 1983, J. Fluid Mech. 133,
   345–376).

Items 2 and 3 are taken as given by Bordoloi, Lai, Clark, Veliz & Variano (2019), *Turbulence statistics in a
negatively buoyant multiphase plume*, arXiv:1907.08315, sec. 3.1 and fig. 7. I could not access the original papers.
These values are the next thing to verify against the originals.

Exposed preset; the buried one differs by at most 4 percentage points (`output/plume_preview/buried/stats.json`):

| Height above source | 0.5 m | 1.0 m | 2.0 m |
|---|---|---|---|
| z/D | 0.13 | 0.27 | 0.53 |
| Model liquid u_c [m/s] | 0.799 | 0.635 | 0.504 |
| MTT u_c | 0.952 (model −16 %) | 0.756 (−16 %) | 0.600 (−16 %) |
| Lai & Socolofsky, extrapolated | 0.733 (model +9 %) | 0.582 (+9 %) | 0.462 (+9 %) |
| Model b [m] | 0.055 | 0.104 | 0.200 |
| MTT b | 0.048 (model +14 %) | 0.096 (+8 %) | 0.192 (+4 %) |
| Bubble-plume data, 0.114 z | 0.057 (model −4 %) | 0.114 (−9 %) | 0.228 (−12 %) |
| Bubble cloud RMS radius [m] (after calibration) | 0.050 | 0.087 | 0.161 |
| λ = bubble / liquid width | 0.92 | 0.84 | 0.80 (calibrated to 0.80–0.85, see below) |

The liquid rows do not depend on the calibration below; the bubble rows do.

Where we disagree, plainly:
- **Centreline speed: 16 % below MTT.** Expected, because slip lowers the buoyancy per unit length (g Q / (u + u_s)).
  Gas expansion (+25 % flux at the surface) is included and only partly offsets it. The speed is 9 % above the
  Lai & Socolofsky fit, but that comparison is outside the fit's range (z/D 0.13–0.53 vs 2–11), so it indicates
  rather than validates.
- **Liquid plume width: 9–14 % narrower** than the bubble-plume data line from 1 m up (12 % at 2 m). b = 1.2 α z
  would match 0.114 z for α = 0.095. **Accepted as within the literature scatter** of bubble-plume spreading rates;
  `plume.entrainment_coeff` stays 0.08 (Gaussian). Project decision, 2026-10-26.
- Below 0.5 m the bubble statistics describe the release region (bubbles formed along the first 2 cm of the jet,
  or across the pockmark floor), so they are not compared.

### Calibration (not a validation): `plume.turbulence` = 0.425

What it sets: the lateral dispersion of the bubbles. The integral model fixes the liquid flow; how far the bubbles
spread across it comes from the Ornstein-Uhlenbeck velocity fluctuations (σ = `plume.turbulence` × local u_c,
Lagrangian time 0.3 b/u_c). This one knob lumps every lateral dispersion mechanism (turbulence, wake-induced lift,
path instability), so it is not a measured turbulence intensity.

Target: bubble core ratio λ = b_bubbles / b_liquid = **0.80–0.85 at 1–2 m** above the source, from Milgram (1983),
J. Fluid Mech. 133, 345–376 (0.8–0.9), as quoted by Bordoloi et al. (2019, arXiv:1907.08315, sec. 3.1). Measured as
the gas-volume-weighted RMS horizontal distance of the bubbles from their centroid, pooled over 60 snapshots in a
15 s window after the 12 s warm-up (`tools/plume_preview.py`; for a Gaussian void fraction exp(−r²/b²) the RMS
distance is b).

Sweep (λ at 1 / 1.5 / 2 m; run-to-run noise about ±0.01):

| `plume.turbulence` | exposed | buried | mean rise time |
|---|---|---|---|
| 0.25 (draft) | 0.56 / 0.52 / 0.50 | 0.57 / 0.53 / 0.52 | 4.3 s |
| 0.40 | 0.81 / 0.78 / 0.77 | 0.81 / 0.78 / 0.77 | 5.1 s |
| 0.42 | 0.84 / 0.80 / 0.79 | 0.84 / 0.80 / 0.80 | 5.2 s |
| **0.425** | **0.84 / 0.82 / 0.80** | **0.84 / 0.81 / 0.81** | 5.3 s |
| 0.44 | 0.86 / 0.84 / 0.83 | 0.87 / 0.84 / 0.83 | 5.4 s |
| 0.46 | 0.89 / 0.87 / 0.86 | 0.90 / 0.86 / 0.86 | 5.5 s |

Rows 0.40, 0.44 and 0.46 were measured before the bubble-pool fix below; it only affects bubbles older than 12.7 s,
and at 0.425 the runs before and after it differ by at most 0.016.
λ falls with height and the target band is barely wider than that fall, so 0.425 is the value that keeps both
presets inside it from 1 m to 2 m. Consequences: the mean bubble rise time to the surface grows from 4.3 s to 5.3 s,
and about 62 k (exposed) / 24 k (buried) bubbles are alive, of a 150 k pool. To revisit against a real recording
(M2): the plume's acoustic footprint in the sonar image is set mostly by this width.

### Drafts fixed during M1 (one line each)

- Bubble sizes were clipped to `d_range_mm`, which piles bubbles at 10 mm. They are now drawn from the truncated
  log-normal that the mean volume and slip already assumed.
- `plume.entrainment_coeff` is the Gaussian α that papers quote (and that the config comment's ranges are in).
  The top-hat integral now uses √2 α; before, it entrained 30 % too little.
- Buried leak: the vents, a few cm apart, feed one merged plume with the total gas flux. The draft gave each vent a
  full-flux plume and summed the velocities, about 3× too fast where they overlap.
- The integral model's buoyancy now includes gas expansion as the pressure falls (+25 % volume flux at the surface
  for the default leak).
- The liquid's radial velocity is now derived from continuity of the Gaussian plume: slightly outward in a
  decelerating core, entrainment inflow outside. The draft's inflow-only term made the core converge and squeezed
  the bubbles to λ ≈ 0.42.
- Bubble pool: the draft's ring buffer overwrote any bubble older than pool size / emission rate (12.7 s), even
  with most slots free. Slow small bubbles that leave the plume live longer than that (found when the calibration
  raised the dispersion). New bubbles now go into free slots, and a bubble is dropped only when the pool is full,
  which is counted.

### Bubble model checks (GPU, `tests/gpu/test_bubbles.py`)

Both pipe presets, 12 s warm-up + 20 s:
- No NaNs.
- Gas amount (p·V) balance: emitted = alive + burst + dropped within 10⁻⁵. The alive part is computed from each
  bubble's current radius and depth, so the expansion law is checked too. Nothing is dropped: about 62 k alive
  (exposed) and 24 k (buried) of a 150 k pool. A separate run with a 3000-bubble pool overflows on purpose: bubbles
  are dropped and the balance still closes.
- In steady state, the gas bursting at the surface balances the gas released within 10 %.
- The plume reaches the surface, and no bubble is above it.
- `plume.flow_field: lbm` raises NotImplementedError naming M6.

### Acoustic scatterers (`uw/geometry.py`, `tests/test_geometry.py`)

Adaptive spacing: max(2 mm, √(R Δθ Δr / 9)), i.e. about 3 × 3 scatterers per resolution cell once that is coarser
than 2 mm (from about 0.5 m at 1° and 4 mm); walls at least 4 mm. Each point carries the area it stands for, so
areas and expected backscatter are kept:

| | tank fixed | tank adaptive | basin adaptive |
|---|---|---|---|
| Points | 4.71 M | 2.53 M | 4.75 M |
| Build time (host CPU) | 8.8 s | 4.7 s | 9.6 s |
| Area vs fixed | | −0.00 % sand, −0.11 % pipe | |
| Visible Lambert energy vs fixed | | −0.13 % | |

Fixed 2 mm spacing in the basin would need about 16.5 M points.

## M2: bubble scattering and the sonar (2026-10-26, profile `linus`, LF mode)

### Bubble scattering (`uw/scattering.py`, `tests/test_scattering.py`)

Reference: Anderson (1950) modal series for a fluid sphere, gas in water (methane at 1.27 bar, 15 °C:
g = ρ_gas/ρ_w = 8.3·10⁻⁴, h = c_gas/c_w = 0.294). At 750 kHz the bubbles have ka = 1.6–16.

Runtime closed forms, tabulated against ka for the GPU:
- **Backscatter: Stanton (1989)** high-pass model for a gas-filled sphere, σ_bs = a² (ka)⁴ α² G / (1 + 4 (ka)⁴ α² /
  (R² F)), with α = (1 − g h²)/(3 g h²) + (1 − g)/(1 + 2g), R = (gh − 1)/(gh + 1), F = 1 + 0.5 (ka)^−0.75 and
  G = 1 + 85 exp(−5·10⁵ (ka − 0.0135)²). Constants as implemented in hydrac. Thuraisingham (1997) and Ainslie &
  Leighton (2011) correct the resonance region (ka ≲ 1) and do not reach the geometric regime where these bubbles are.
- **Extinction:** a two-parameter fit to the modal series, Q = σ_ext/(πa²) = 2 + 2/(1 + (ka/2.24)^0.95). I found no
  published closed form covering ka 0.1–20.

| | mean abs. error | bias | 95 % within | max |
|---|---|---|---|---|
| σ_bs, Stanton vs modal, ka 0.1–20 | 0.68 dB | −0.21 dB | 1.66 dB | 11.9 dB at ka = 2.14 (a narrow interior resonance of the ideal sphere) |
| σ_ext, fit vs modal, ka 0.1–20 | 0.10 dB | | | 3.8 dB (same kind of spike) |

Limits:
- Modal σ_bs/a², averaged over ka 60–100, is 0.25 ± 5 %; Stanton at ka = 1000 gives 0.250.
- Modal σ_ext/(πa²) at ka = 1000 is 2.02 (extinction paradox: 2).

The drafts had no TS or σ_ext formulas to remove; the plan's note referred to an earlier draft.

### Sonar model (`uw/sonar.py`, `tests/gpu/test_sonar.py`)

| Check | Measured | Expected |
|---|---|---|
| Point target, 2 m, TS −30 dB, beam 300 | range bin 500, beam 300; level 162.98 dB re 1 µPa | bin 500, beam 300; SL − 40 log R − 2αR + TS + fan gain = 162.98 dB |
| Beam pattern, 30 m, unfocused (4× the Fraunhofer distance) | −3 dB width 0.982°, first sidelobe −27.6 dB | 1°, −28 dB (Taylor, n̄ = 4) |
| Near field, 1 m, focused | 0.980° | ≈ 1° |
| Near field, 1 m, unfocused | 3.825° | 3.833° (exact spherical wavefront at the 127 elements) |
| Sand speckle, flat sand, contrast std/mean of I / local mean | fixed 0.984, adaptive 0.983 (tank, 2.15–2.8 m); adaptive 0.989 (basin, 4–6 m) | 1 (Rayleigh); ⟨I²⟩/⟨I⟩² 1.97–1.98 (Rayleigh: 2) |
| Surface ghosts, sonar 0.3 m below a calm surface, point 2 m ahead | bins 511 and 522; +1.22 dB and −9.61 dB re direct | bins 511 and 522; +1.22 dB, −9.59 dB (image geometry, vertical sinc) |
| Blind zone | rows below 0.1 m all zero / NaN, with noise and ring-down on | nothing returned |

- **Array:** 127 half-wavelength elements, L = 0.127 m, design width 0.993°, Fraunhofer distance L²/λ = 8.0 m. The
  whole tank is in the near field.
- **Basin enabled.** Adaptive spacing passes the speckle test where it is coarsest (4–6 m, about 6 mm, 9 scatterers
  per resolution cell).
- **Speckle test region:** |bearing| < 25°. Beyond about 32° the side walls are in the window and the contrast
  rises to 1.2 (bright lines, not under-sampled speckle).

### Parameters UNVERIFIED — datasheet (`config/tank.yaml`, sonar section)

| Parameter | Value used |
|---|---|
| Frequencies | LF 750 kHz, HF 1.2 MHz |
| Fan | 130° × 20° (LF), 70° × 12° (HF) |
| Beams | 512 |
| −3 dB beam width | 1.0° (LF), 0.6° (HF) |
| Range resolution | 4 mm (LF), 2.5 mm (HF) |
| Minimum range (blind zone) | 0.1 m |
| Beam spacing | uniform in angle |
| Near-field focusing | `focus: true` |
| Receive taper | Taylor, −28 dB |
| Horizontal gain at the fan edges | two-way −3 dB |
| Vertical aperture | uniform (sinc), two-way −3 dB at ±fov_v/2 |
| Ring-down | 60 dB above the beam noise floor at range 0, e-folding 0.08 m |

Assumed (not datasheet values):

| Parameter | Value used | Note |
|---|---|---|
| Source level | 206 dB re 1 µPa @ 1 m | |
| Receiver noise | 52 dB re 1 µPa²/Hz | Read as a spectrum level per element over the band c/(2Δr) = 188 kHz. The beams then have an 84.3 dB noise floor, and sand is 25–40 dB above it. Read as a band level, sand would sit about 75 dB above the noise. |
| Volume backscattering | −75 dB re 1/m | Below the noise in the tank. |
| Calm surface | 0.2 mm rms | |
| Plume boil | 5 mm rms | |
| Speckle dynamics | 0.1 rad, 2 s | Tests use 0. |
| Display window | 0–80 dB above the noise floor | Display gain only. |

### Performance (linus, RTX 5050, warm kernel cache; `tools/sonar_preview.py`, `python -m uw sonar-only`)

| | scatterers | plume-box paths | static precompute | sonar ping, mean (max) | bubble step per ping | peak VRAM |
|---|---|---|---|---|---|---|
| tank, exposed | 2.53 M | 4.9 M | 0.12 s | 32 ms (36) | 16 ms | 0.65 GB |
| tank, buried | 2.41 M | 6.0 M | 0.12 s | 37 ms (43) | 17 ms | 0.65 GB |
| basin, exposed | 4.75 M | 12.1 M | 0.3 s | 86 ms (94) | 16 ms | 1.3 GB |
| basin, buried | 4.62 M | 14.2 M | 0.3 s | 97 ms (107) | 16 ms | 1.3 GB |

- **Tank:** within the 100 ms budget, with or without the bubble step.
- **Basin:** the sonar ping alone is within budget on average, but with the bubble step it is 2–13 % over. The
  time goes into the per-ping extinction of the static paths that cross the plume region, 74–85 ms in the basin.
  Fidelity was not reduced.
- **Possible fixes**, not done: skip plume-box paths whose amplitude is far below the noise floor, or march the
  extinction at one sample per voxel instead of two.
- **First run after a change to `uw/sonar.py`:** Warp recompiles the kernels (about 15–35 s).
- **Datasets:** writing a ping (compressed npz + 3 PNGs) takes about 100 ms on the CPU. The 200-ping smoke dataset
  took 30 s.

### Physically questionable in the images (open)

- **Uniform vertical aperture.** Strong banding at 0.8–1.5 m (vertical sidelobes on the sand) and a bright nadir
  line at the sonar's altitude (0.75 m). Both follow from the sinc pattern the plan specifies; a real transducer is
  tapered and baffled, so these are probably too strong. To decide with the datasheet or a real recording.
- **Sidelobe arcs across the whole fan.** They come from specular returns of the far wall, the tank corners and the
  pipe. The levels are physical for smooth concrete (the wall acts as a mirror, about 187 dB re 1 µPa), but the far
  Taylor sidelobes of a real array may be lower than the ideal taper.
- **Specular glints.** The pipe and walls are built from many random-phase facets: their energy is right, but their
  coherence is that of speckle rather than a mirror.
- **Surface backscatter.** The calm surface is only a mirror (image paths); it scatters nothing itself.
- **Bubble image paths** use the backscattering cross-section for bistatic angles too.
- **Plume label** (bubble energy share > 0.5): it includes the upper plume seen through the vertical sidelobes and
  its surface-path ghosts, in the plume's own shadow, out to 3.4 m. That is consistent with the definition, but a
  stricter label (e.g. also above the noise floor + x dB) may suit detector training better.

### Real-data comparison (skeleton, `tools/compare_real.py`, on our 200-ping smoke dataset)

| Measure | Result | Note |
|---|---|---|
| Background, 2.15–2.8 m, ±25° | contrast 1.99, K shape 0.7 | The real tank with ripples, rocks and the plume shadow is K-distributed; pure speckle is the flat-sand test above. |
| Blind zone | 0.100 m | |
| Ring-down decay | −106 dB/m | −109 dB/m configured (e-folding 0.08 m). |
| Brightest target | the pipe | An extended target, so its "sidelobe" is meaningless here; `--target` selects a window holding a point target (e.g. a calibration sphere in a real recording). |
| Level vs range | | Shaped by the vertical beam pattern on the sand. Compare the curves with a real recording, not the fit constants. |
