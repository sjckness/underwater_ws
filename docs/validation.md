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

### Parameters VERIFIED and UNVERIFIED (`config/tank.yaml`, sonar section)

VERIFIED against the datasheet (Blueprint Subsea, "Oculus M Series Datasheet EN", M750d; confirmed 2026-10-26):

| Parameter | Value used |
|---|---|
| Frequencies | LF 750 kHz, HF 1.2 MHz |
| Fan | 130° × 20° (LF), 70° × 12° (HF) |
| Beams | 512 |
| Angular resolution (−3 dB beam width) | 1.0° (LF), 0.6° (HF) |
| Range resolution | 4 mm (LF), 2.5 mm (HF) |
| Minimum range (blind zone) | 0.1 m |
| Maximum range | 120 m (LF), 40 m (HF): `sonar.range` is checked against it |
| Maximum update rate | 40 Hz: `sonar.ping_rate_hz` is checked against it |

UNVERIFIED (not on the datasheet; assumed):

| Parameter | Value used |
|---|---|
| Beam spacing | uniform in angle |
| Near-field focusing | `focus: true` |
| Receive taper | Taylor, −28 dB |
| Horizontal gain at the fan edges | two-way −3 dB |
| Vertical aperture | Hann taper (M2.1; cosine and uniform available), two-way −3 dB at ±fov_v/2 |
| Wall roughness, lining | 0.5 mm rms bare concrete (M2.2); lining reflection 0.1 (assumed) |
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

### Physically questionable in the images (open; planned fixes in `docs/PLAN.md`, Known gaps)

- **Uniform vertical aperture.** Resolved in M2.1: the default is now a Hann taper (see below).
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

Superseded by "Real data: public sonar datasets" below, where the same statistics are run against real recordings; this table is what the measures read on our own data before any existed.

| Measure | Result | Note |
|---|---|---|
| Background, 2.15–2.8 m, ±25° | contrast 1.99, K shape 0.7 | The real tank with ripples, rocks and the plume shadow is K-distributed; pure speckle is the flat-sand test above. |
| Blind zone | 0.100 m | |
| Ring-down decay | −106 dB/m | −109 dB/m configured (e-folding 0.08 m). |
| Brightest target | the pipe | An extended target, so its "sidelobe" is meaningless here; `--target` selects a window holding a point target (e.g. a calibration sphere in a real recording). |
| Level vs range | | Shaped by the vertical beam pattern on the sand. Compare the curves with a real recording, not the fit constants. |

## M2.1: vertical taper, two plume labels, basin speed, dataset size (2026-10-26)

### Vertical aperture taper (`sonar.vertical_taper`, UNVERIFIED: not on the datasheet)

The default is now a Hann taper; `cosine` and `uniform` (sinc) remain options. Closed forms, x = π L sin(angle)/λ:
uniform sin x / x, cosine π² cos x / (π² − 4x²), Hann (sin x / x) π² / (π² − x²). L is set so that the two-way gain
is −3 dB at ±fov_v/2 for each taper. `tests/gpu/test_sonar.py::test_vertical_pattern` checks the measured level at
10° and 16° off axis against the closed form (±0.3 dB) for all three.

Tank, exposed, static ping, beams within ±40°:

| Taper | altitude line (0.70–0.82 m) re sand at 2.3–2.7 m | 0.85–1.5 m mean re sand | range-profile rms around its 0.1 m trend |
|---|---|---|---|
| uniform | +14.6 dB | −2.1 dB | 3.1 dB (19.5 dB peak-to-peak) |
| cosine | −5.9 dB | −5.7 dB | 2.7 dB |
| Hann | −21.8 dB | −6.3 dB | 2.4 dB (15.3 dB peak-to-peak) |

- **Altitude line:** gone, 36 dB lower with Hann.
- **Sidelobe banding:** the alternating bright and dark rings at 0.75–1.5 m are gone from the images.
- **What the rms column still measures:** ripples and rocks, not sidelobes. With the ripples switched off, the
  uniform and Hann apertures give 2.7 and 2.4 dB.

### Two plume labels per ping

- `mask_all`: the bubbles' share of the energy over all their paths (direct and surface ghosts) > 0.5.
- `mask_direct`: the share of their direct path only > 0.5.

Both are stored as PNGs. The shares are stored as uint8 (255 = all of the energy) in the npz, and the manifest
carries the definitions and the threshold. `test_plume_labels`: the direct label stays within the plume's own
ranges, while the all-paths label adds pixels beyond them (surface ghosts).

With the Hann taper the two labels nearly coincide in the tank (ping 100: 5180 vs 5159 pixels): the ghosts and the
upper plume seen through the vertical sidelobes are now far weaker.

### Basin: weak plume-region paths (`sonar.plume_path_floor_db`)

Scene paths through the plume region that are more than |floor| dB below the beam noise floor skip the per-ping
extinction: they join the static image unattenuated, so no energy is dropped. The floor is −20 dB in the `basin`
preset and off (null) in `tank`. Exact vs −20 dB, same bubbles and same noise seeds, 10 pings:

| | plume-region paths | sonar ping, mean (max) | per-pixel difference |
|---|---|---|---|
| basin, exposed | 12.13 M → 4.02 M | 87.4 (95.6) → 40.9 (92.8) ms | mean abs. 0.018 dB, 99th percentile 0.37 dB, max 33 dB |
| basin, buried | 14.15 M → 4.76 M | 100.8 (108.8) → 38.2 (46.5) ms | mean abs. 0.017 dB, 99th percentile 0.35 dB, max 43 dB |

- **Where the maxima are:** speckle nulls, 38 pixels in about a million, a median 21.7 dB below their local 5×5
  mean. A tiny change flips a near-cancellation there.
- **On 5×5-averaged intensity:** mean abs. difference 0.004 dB, 99.9th percentile 0.43 dB, max 1.6 dB.
- **Budget:** the basin is now within it with the bubble step included (about 55 ms per ping).

### Dataset size (`python -m uw sonar-only`, tank exposed, 200 pings)

| | MB per ping |
|---|---|
| M2 (float16 share, fan PNG) | 1.65 |
| M2.1, `recorder.fan_png: false` (default) | **0.98**: raw dB float16 0.60, two uint8 shares 0.05, polar PNG 0.32, two masks 0.01 |
| M2.1, `recorder.fan_png: true` | 1.30 |

## M2.2: coherent glints, wall roughness and lining, sonar roll, ping video (2026-10-26)

### Coherent glints (physical optics on smooth surfaces)

Smooth materials (`coherent: true`: coated steel, concrete, glass, lining) now have two parts:
- **Mirror-like share:** their triangles (edges ≤ `sonar.coherent_patch_m` = 1.5 cm, so the phase stays linear
  across one) carry the Kirchhoff field −(ik R₀/4π) ∫ n·(ŝ_i + ŝ_s) e^{ik(L_t+L_r)} dA with deterministic phase,
  times the Rayleigh factor exp(−½ k² h² (cos θ_i + cos θ_s)²).
- **Incoherent rest:** the random-phase scatterers keep the remaining specular energy (factor 1 − χ²), with rms
  slope √2 h / correlation length.
- **The triangle integral** is exact for planar facets: 2A times minus the second divided difference of e^{it} at
  the vertex phases (Hermite–Genocchi). It matches Gauss–Legendre quadrature to 10⁻⁶ of the area, so there are no
  sampling grating lobes.
- **Sand and rock** stay incoherent.

Smooth steel plate, 1 × 2 cm, at 2 m (far field from 0.4 m), tilted about the vertical axis
(`test_smooth_plate_kirchhoff_echo`):

| Tilt | Measured | Kirchhoff (R₀ A cos t / λ) · sinc(k W sin t) | Difference |
|---|---|---|---|
| 0° | 172.02 dB | 172.11 dB | −0.10 dB |
| 2° | 170.22 | 170.31 | −0.09 |
| 4° | 163.58 | 163.58 | 0.00 |
| 5.76° (first null) | 146.5 (−25.6 dB re peak) | 0 | finite pulse and near field fill the null |
| 8.26° (first sidelobe) | 158.16 | 158.76 | −0.60 |
| 12° | 141.33 | 142.42 | −1.09 |

A 2 cm wide plate (0.57°, comparable to the 1° beam) gives a 1.3 dB lower first sidelobe. That is the receive beam
tapering the plate, which is physical; the analytic formula assumes a plate much smaller than the beam footprint.

The tank has 1.04 M coherent facets: 224k per wall and 147k on the pipe.

**Kernel cache.** Warp cached PTX, so the CUDA driver re-compiled the large sonar module in every new process
(40 s; HOME is ephemeral in the container). Warp now emits cubin, cached in `~/.cache/uw_warp`, and the driver's
JIT cache is kept in `~/.cache/uw_cuda`. The first run after a change to `uw/sonar.py` compiles once (about
100 s); after that a precompute takes 0.2 s.

**Tank plume-region floor.** The facets added 3.5 M plume-region paths (ping 84 ms). `plume_path_floor_db` for
the `tank` preset is now −60 dB instead of off. Against exact, 6 pings with the plume:

| Floor | plume-region paths | ping | mean abs. difference | 99th percentile | pixels > 1 dB |
|---|---|---|---|---|---|
| off (exact) | 7.45 M | 84.0 ms | | | |
| **−60 dB (default)** | 3.95 M | 37.3 ms | 0.0001 dB | 0.002 dB | 0 % |
| −40 dB | 2.50 M | 22.1 ms | 0.0014 dB | 0.020 dB | 0.012 % |
| −20 dB (basin) | 0.53 M | 11.4 ms | 0.021 dB | 0.45 dB | 0.47 % |

### Walls: per-wall rms roughness and absorbing lining (`tank.walls`)

- **Rough walls:** the Rayleigh loss exp(−2 (k h cos θ)²) applies to the mirror (image) paths, and the walls'
  coherent glint share falls the same way.
- **Default:** bare concrete, moderately rough (0.5 mm rms, 1 cm correlation length; assumed).
- **Lining:** the `lining` material (reflection 0.1, Lambert −40 dB; assumed) replaces the wall material where
  `lining: true`.

Ghost world (multipath images beyond the walls), mean echo energy relative to smooth concrete, quiet static pings,
range 5 m:

| Walls | beyond the far wall (3.6–5 m, ±20°) | beyond the side walls (2.5–3 m, 50–62°) | sand inside (control) |
|---|---|---|---|
| smooth concrete (0 mm) | 0 dB | 0 dB | 0 dB |
| default (0.5 mm rms) | −37.6 dB | −8.3 dB | −0.5 dB |
| absorbing lining (R 0.1) | −43.9 dB | −23.7 dB | −0.5 dB |

The far wall is seen near normal incidence, where 0.5 mm of roughness at λ = 2 mm destroys the mirror. The side
walls are seen at grazing angles, where much of it survives.

### Sonar roll (`sonar.roll_deg`, `pitch_deg`, `yaw_deg`)

One rotation (intrinsic Z-Y-X: yaw, pitch nose-down, roll about the look axis) drives the TX/RX patterns, the beam
geometry (u is measured along the rotated array), the image paths and the labels.
`test_roll_maps_a_vertical_line_of_targets` checks, for roll 0°, 45° and 90°, that point targets on a vertical
line 2 m ahead appear at the range bin (±1) and bearing (±0.3°) predicted by an independent rotation (scipy).

Previews in `output/sonar_preview/exposed{,+roll45,+roll90}/`:
- **0°:** the rising plume is a compact blob (a point).
- **45°:** a short diagonal streak, where the column crosses the tilted 20° slab.
- **90°:** a long streak across the bearings, from the plume's base (1.7 m, +3°) to its top (2.5 m, −62°). In the
  fan view this is an arc to the left.

### Ping video (`tools/sonar_preview.py --video 50`)

H.264 (libx264 `-preset veryfast`, via a pinned `imageio-ffmpeg` in `~/.cache/uw_pydeps`), fan view, 10 Hz:
`output/sonar_preview/{exposed,buried}{,+roll90}/plume.mp4`. Statistics of the plume region (every pixel the
direct-path label marks in any of the 50 pings):

| | echo energy, std over pings | ping-to-ping rms | pixel correlation, next ping / 5 pings apart | label pixels |
|---|---|---|---|---|
| exposed, roll 0° | 0.40 dB | 0.52 dB | 0.70 / 0.70 | 4710 ± 179 |
| buried, roll 0° | 0.35 dB | 0.44 dB | 0.45 / 0.46 | 4500 ± 200 |
| exposed, roll 90° | 0.13 dB | 0.17 dB | 0.44 / 0.45 | 89 100 ± 1050 |
| buried, roll 90° | 0.09 dB | 0.12 dB | 0.41 / 0.41 | 91 700 ± 1520 |

Qualitatively the plume sparkles rather than flickers:
- **Speckle:** its texture renews completely from one ping to the next. The bubbles move about 6 cm per ping,
  against a 2 mm wavelength, and the correlation does not fall from 1 to 5 pings apart. (Corrected in M3: the
  "pixel correlation" above is the raw correlation of power over the union of the label, which the plume's shape,
  the range fall-off and static pixels in the union dominate. The speckle correlation of M3 gives about 0, see
  below.)
- **Brightness and size:** they hold to a few tenths of a dB and about ±4 % of the label area. The roll-90° streak
  integrates many more bubbles and is steadier still.
- **Probably steadier than a real seep**, which flares in puffs and clusters. The flow is constant, and the plume
  model has only slow meander and small-scale turbulence. Added to the Known gaps in `docs/PLAN.md`.

## M3: Isaac integration and ROS 2 (2026-10-26)

All on linus (RTX 5050 8 GB, i7-9700K, 16 GB RAM), tank preset, exposed pipe, unless noted. The GPU is shared with
the desktop (about 1.5 GB) and an unrelated process holding 2 GB, so about 4.5 GB were free.

### One geometry source

`usd_scene.check_geometry` runs at every `sim` start: the USD meshes of the occluders (sand, rocks, pipe, flanges,
supports), concatenated as `Scene.occluders` does, have exactly the vertices and triangles of the sonar's Warp BVH
mesh (130 827 vertices, float32 bit-equal); every other mesh's USD points equal its numpy points.
`tests/gpu/test_usd_scene.py` builds the stage in memory (pxr from the Isaac install, no Kit) for both pipe presets
and also checks that one vertex moved by one float32 ulp fails the check, the sonar, camera and lamp poses, and the
bubble instancer against the pool.

### The same pings in `sim` and `sonar-only`

The echoes and the bubble extinction were summed with float atomics, whose order varies: two `sonar-only` runs with
the same seed already differed (raw dB by up to 0.0625 dB, one float16 step; plume shares by 1/255). They are now
summed in 64-bit fixed point (2^-20 µPa; extinction 2^-32 1/m), whose sums do not depend on the order: a ping is
bit-identical for the same config and seed. The ping time did not change measurably (35 ms vs 36 ms in M2.2).
(Warp 1.8's `atomic_add` on int64 arrays silently does nothing; uint64 with two's complement works.)

`tests/isaac/test_sim_identity.py` (`python -m uw test --isaac`, 45 s): `sim --sink file` (Kit app, RTX camera
and bubble instancer running, 240 Hz substeps, a ping every 24) and `sonar-only` give identical arrays, PNGs and
JSON (except the timings) for 10 pings.

### Bubbles in the render

`UsdGeom.PointInstancer` of a unit icosphere (80 triangles), scaled to ellipsoids of the bubble's volume: aspect from
Wellek et al. (1966), short axis along the velocity relative to the water (slip and path oscillation). Material:
OmniGlass with IOR 0.75 (gas in water); `sim.bubble_material=inspect` draws them opaque for looking at the plume.
Update per frame, 62 000 bubbles:

| path | GPU compaction | to the host | set | total |
|---|---|---|---|---|
| Fabric (usdrt, GPU arrays; `sim`) | 0.6–0.9 ms | none | 0.3–0.4 ms | 1.0–1.3 ms |
| USD (host copy; tests, no Kit) | 0.6 ms | 1.0 ms (32 B per bubble) | 3.5 ms | 5.1 ms |

The cost that remains is in the renderer: RTX rebuilds the instances for every change, about 20 ms per update
(render of the camera frame 8 ms with the bubbles static, 13 ms hidden, 28–30 ms updated every frame; the material
does not matter). Hence `sim.bubble_render_hz` (10, the camera rate; the GUI shows the bubbles at 10 Hz while the
viewport refreshes at 20).

### Performance and GPU memory

VRAM: peak used memory of the `uw` process, `nvidia-smi` (`tools/gpu_mem.py`, host side). Timings from
`--stats-file`, means over the run (ping = `Sonar.ping` incl. waiting for the queued bubble substeps).

| mode | VRAM | real-time factor | ping mean / max | other stages per 100 ms of sim time |
|---|---|---|---|---|
| `sim`, headless, camera, ROS | 3.1 GB | 0.75–0.79 | 53 / 69 ms | bubble substeps 27 ms (launch), render 30 ms, camera read + publish 10 ms, sonar messages 9 ms |
| `sim`, headless, no camera, ROS | 2.8 GB | 1.00 (real time) | 36 / 73 ms | bubble substeps 25 ms, app update 5 ms |
| `sim --gui`, camera, ROS | 3.0 GB | 0.47 | 93 / 111 ms (107 of 300 over 100 ms) | 2 viewport renders 63 ms, overlay 14 ms |
| `sonar-only`, file sink | 0.64 GB | (no pacing) | 36 / 70 ms | bubbles 15 ms, writing 63 ms |
| `sonar-only`, ROS sink | 0.64 GB | (no pacing) | 35 / 63 ms | bubbles 16 ms, publishing 6 ms |

Targets: full sim with camera < 7 GB: met (3.1 GB). Ping ≤ 100 ms: met headless and in `sonar-only`; with the GUI
the mean is 93 ms and a third of the pings exceed 100 ms (the viewport competes for the GPU). Real time: only
headless without the camera; see Known gaps in docs/PLAN.md for where the time goes.

### ROS 2

Topics, rates and the relay: `output/m3/ros_topics.txt` (not in git) has the `ros2 topic hz` output. With the sim at
RTF 0.75–0.79, the sonar topics arrive at 6.7–6.8 Hz wall clock (10 Hz in sim time), `/clock` at 40 Hz (60 in sim
time). The big images (`/oculus/polar_db` 1.8 MB, `/camera/image_raw` 2.7 MB, the relay's `/oculus/sonar_image`)
measure 5.0–5.8 Hz: `ros2 topic hz` (Python) does not keep up with deserialising them. Twelve `ros2 topic hz`
processes at once slowed the sim to RTF 0.5 (CPU). `ros/sonar_relay` republished 283 pings in 45 s as
ProjectedSonarImage (frame `oculus_projected`, 512 beam directions, ranges, 750 kHz, 1506.7 m/s); `/oculus/meta`,
`/tf_static` (4 frames) and `/sim/leak` arrive latched.

### Container and Kit app as a non-root user

- Kit writes into its portable folders `/isaac-sim/kit/cache` and `/isaac-sim/kit/data`, which the image keeps
  read-only for us: without them the data store, the shader cache and the OmniGraph node cache fail and the app
  crashes (exit 139). Both are now mounted from `~/.cache/uw_home/kit_{cache,data}`, and HOME is `~/.cache/uw_home`
  (`.cache/ov`, GLCache, ComputeCache, config, logs persist): later starts reuse the compiled shaders (Kit up in
  25 s, scene and plume setup 6–8 s).
- `sim --gui` mounts the X11 socket and the X authority file and passes DISPLAY.
- Replicator annotators deliver data only if the timeline already plays when they are attached (otherwise they stay
  empty for good, with or without a World): `sim` plays it first.
- The lamps were 75 cd (60 000 nits on a 2 cm sphere): the camera image was black. Now 4000 cd each
  (800 000 nits, 4 cm radius); with no ambient light the camera sees the sand, pipe and rocks inside the two
  cones and nothing else (`output/m3/camera_t25.png`).

### Addendum: plume echo between pings, range rings, videos (review of the M2.2 roll-90° video)

**The plume echo is not frozen.** The bubbles are stepped between pings in the video tool and in both run modes
(median displacement 5.0 cm per ping, 25 wavelengths; 3 of 62 000 bubbles move less than 4 mm). Nothing caches the
bubble contribution: it is deposited from the GPU pool every ping. What made it look frozen was the measure: the
M2.2 "pixel correlation" (0.70) is the raw correlation of power over the union of the label, dominated by the plume's
shape and the range fall-off. Measured instead by `sonar_stats.speckle_correlation` (each image over its local mean,
7 bins × 5 beams, only pixels whose whole window is inside the region):

| | plume, next ping | plume, 5 pings apart | static scene, next ping |
|---|---|---|---|
| exposed, roll 0°, pitch 15° | 0.01 | −0.01 | 1.00 |
| exposed, roll 90° | −0.00 | 0.00 | 0.99 |
| exposed, pitch 30° | 0.05 | −0.06 | 1.00 |
| exposed, pitch 45° | −0.08 | (few pixels) | 1.00 |

A plume from another seed gives 0.05 against the same ping: the plume speckle is new every ping. Without the window
erosion the plume gave 0.19 (0.12 with an independent plume): the ratio at a region's edge follows the edge.
`test_plume_speckle_renews_every_ping_static_sand_does_not` (speckle dynamics off): plume |r| < 0.1, sand > 0.9.
The plume's brightness and outline do change slowly (energy ±0.4 dB at roll 0°): the Known gap "plume
intermittency" stands.

**The range rings are sidelobe leakage, not an artifact.** With the fan vertical, the beams looking down see the
sand at near-normal incidence (strong: Kirchhoff with rms slope 0.25), and the plume column lies in the fan plane.
Each strong echo leaks through the array's sidelobes into every beam at its range, so the range-to-range speckle of
that echo is copied across the fan: rings. Evidence, roll 90°, static scene: the beams looking up (water only,
0.8–1.6 m) are 13 dB above the noise floor, and their range profile correlates 0.93 with that of the beams on the
sand (two up-looking beams 60° apart: 0.69). Taylor −45 dB instead of −28 dB lowers them by 7 dB. Multipath, noise,
reverberation, ring-down and focusing change nothing. The beamformer itself is exact: a point target's sidelobes
match the Taylor pattern of the same aperture within 0.1 dB from 10° to 110° off the beam (−48 to −52 dB), and the
integrated sidelobe energy beyond 10° is −19.9 dB in both. Kept as is; the two inputs that set it (the M750d's
sidelobe level, the sand's backscatter near normal incidence) are in the Known gaps. The same leakage of the plume
column makes the roll-90° plume label an annulus across all beams at 1.5–2.4 m.

**Videos** (`tools/sonar_preview.py --video 50`; fan in metres, pose in the title, the plume_direct label's outline,
a side view of the tank with the sonar and its field of view):
`output/sonar_preview/{exposed,exposed+roll90,exposed+pitch30,exposed+pitch45}/plume.mp4`.

| | ping | plume-box paths | plume energy std / ping to ping | label pixels |
|---|---|---|---|---|
| pitch 15° (roll 0°) | 35 ms | 3.95 M | 0.40 / 0.53 dB | 4710 ± 184 |
| pitch 30° | 22 ms | 2.33 M | 0.44 / 0.57 dB | 1710 ± 73 |
| pitch 45° | 12 ms | 0.90 M | 0.58 / 0.82 dB | 900 ± 38 |
| roll 90° (pitch 15°) | 40 ms | 4.31 M | 0.12 / 0.15 dB | 89 500 ± 995 |

Steeper pitch sees less of the column (only its base above the pipe), so the label shrinks and the echo fluctuates
more; it also sees more sand in front of the pipe, and fewer far-wall and plume-box paths make the ping cheaper.


## M3.1: geometric plume label, CUDA graph for the bubble substeps (2026-10-26)

### Geometric label `mask_geom`

Each alive bubble in line of sight from the sonar (shadow ray against the occluders) marks the range bins its direct
echo falls in (the two of `deposit`) and the beams whose main lobe contains it: |u − u_beam| ≤ sin(beam width / 2)
(the −3 dB receive width, constant in u) and |elevation| ≤ fov_v / 2. No echo levels, so no sidelobe leakage; it
also marks bubbles too weak to stand out. Stored as `<seq>_mask_geom.png` next to mask_all and mask_direct; the
manifest's `labels` describes all three.

| | mask_direct pixels | mask_geom pixels | mask_geom inside mask_direct | most beams in one range bin (direct / geom) |
|---|---|---|---|---|
| exposed, roll 0° (one ping) | 4963 | 1111 | 91 % | 161 / 30 |
| exposed, roll 90° (one ping) | 88 940 | 28 676 | 89 % | 508 (all) / 227 |
| exposed, pitch 30° (50 pings) | 1710 | 480 | | |

mask_direct is 3–5 times larger: the sidelobe leakage (at roll 90° an annulus over all 512 beams), the bubbles' echo
beyond the −3 dB main lobe, and the upper plume seen through the vertical sidelobes. In the videos mask_geom is the
cyan outline: at roll 90° it follows the column, the orange mask_direct the annulus. Tests:
`test_geometric_plume_label` (synthetic bubbles: on a beam, above the vertical aperture, under the sand, behind the
sonar), `test_geometric_label_has_no_sidelobe_leakage`.

### CUDA graph for the bubble substeps (`plume.cuda_graph`)

A batch of substeps (up to 240; in `sim` and `sonar-only` the 24 between two pings) now runs in two parts: the host
part of every substep in the original order (vents, meander, new bubbles: all host random numbers) into pinned
staging arrays and one upload; then the GPU part of all substeps, captured once per batch length as a CUDA graph.
Kernels read the per-substep values (new-bubble count, ring head, seed) from device arrays. `sim` now steps the
bubbles only when something reads them (ping, bubble render, /sim/plume), all pending substeps at once.

The bubble pool is bit-identical to the code before (both pipe presets, against a saved state), with or without
the graph, in one batch or one substep at a time (`test_cuda_graph_and_batching_do_not_change_the_bubbles`); the
video statistics and the sim/sonar-only identity are unchanged.

| 24 substeps (one ping period) | before | batched, no graph | graph |
|---|---|---|---|
| `sonar-only` / standalone | 15 ms | 11 ms | 4.2–5.8 ms |
| `sim` (Kit app) | 27 ms (launch) | | 5.4 ms |

Real-time factor, `sim` headless with the camera and the ROS sink: **0.75, unchanged**. The bubbles were not the
limit; the GPU is. Before each ping the loop now waits about 40 ms for the GPU (the camera's RTX render, queued
asynchronously), the render call itself takes 24 ms, the ping 36 ms. Unpaced and without a sink the sim reaches
0.86; with `sim.bubble_stride=2` 0.87 and with `sim.bubble_render_hz=5` 0.84, so neither the instance count nor the
instancer updates matter; it is the camera render sharing the GPU with CUDA. Without the camera, unpaced: 1.41 x
real time (before: real time at 1.0 with little margin).

## M3.5: mobile sonar, kinematic trajectories (2026-10-26)

`sonar.pose_source`: fixed | trajectory | ros (uw/pose.py). Trajectories are waypoints with a speed per segment,
interpolated by a monotone cubic (no overshoot into the walls at the lawnmower's corners), timed so that the mean
speed is the configured one; presets approach (towards the leak), lawnmower, orbit (around the leak, looking at it),
hover (Ornstein-Uhlenbeck drift of position and yaw). `pitch: track` keeps the sonar's axis on the leak.
`tests/test_pose.py`: every preset stays inside the water margin, has no jumps (speed < 2 x the set one), mean speed
within 20 %, 99 % of the accelerations below 5 m/s² (hover excepted: a random walk at the ping rate); the tracking
presets look at the leak within 3°; approach closes in monotonically; orbit goes once round at its radius (±3 cm).

`sonar.static_cache`: precomputed (fixed pose only; moving it is an error) | on_motion (recompute when the pose moved
more than `static_cache_motion`, 5 mm / 0.25° by default) | none. The reverberation table moved to a Warp kernel
for this (0.6 ms instead of 166 ms per pose; the two agree except 8 of 1.5 M cells where a sample lies exactly on
the tank's boundary). With a trajectory, the adaptive scatterer spacing uses the closest distance from each surface
patch to the whole path.

Checks (`tests/gpu/test_mobile.py`):
- a point target at the sonar's height stays where it is: the sonar moves 0.6 m and turns 50° in yaw, and every
  ping's brightest pixel, taken back to the world with that ping's pose, is within 2 range bins + 1 beam of it;
- a sonar moved with `set_pose` gives bit-identically the ping of a sonar configured at that pose;
- on_motion recomputes only beyond its thresholds (3 mm and 0.2° no; 7 mm yes; 2 mm after that no).

Every ping's pose goes into its JSON (position, roll/pitch/yaw, rotation; the leak's range/bearing/elevation in that
pose's frame) and, live, on /tf (tank → oculus; the camera rides on the sonar: oculus → camera on /tf_static). In
Isaac, sonar, camera and lamps form one rig that moves with the sonar; the GUI's fan wireframe follows it.

Ping time with a moving sonar (40 pings, `sonar-only`, exposed pipe; linus):

| | tank: ping (static recompute), recomputes | basin: ping (static recompute), recomputes |
|---|---|---|
| fixed, precomputed | 37 ms, – | 23 ms, – |
| approach, none | 179 ms (154 ms), 40/40 | 372 ms (352 ms), 40/40 |
| approach, on_motion | 176 ms (151 ms), 40/40 | 372 ms (352 ms), 40/40 |
| hover (2 cm drift), none | 171 ms (143 ms), 40/40 | 339 ms (319 ms), 40/40 |
| hover (2 cm drift), on_motion | 162 ms (144 ms), 37/40 | 311 ms (315 ms), 37/40 |

At 0.15 m/s (1.5 cm per ping) and with a 2 cm drift the pose moves beyond 5 mm nearly every ping: on_motion only
pays for stop-and-go motion. Both are inside the mobile budget (no limit on linus: offline datasets).

Videos (`tools/sonar_preview.py --video`, tank exposed, static cache none; side and top views with the current fan
and the path): `output/sonar_preview/exposed+approach+none/plume.mp4` (59 pings, 0.15 m/s, 1.5 → 0.6 m from the
leak) and `output/sonar_preview/exposed+orbit+none/plume.mp4` (209 pings, one orbit of radius 1 m at 0.3 m/s). In
the orbit the pipe sweeps through the fan, the walls and corners swing around, the plume stays at the fan's centre.
The static speckle correlation from one ping to the next is 0.18–0.22 while moving (0.99 fixed): the speckle of the
scene changes with the pose, as it should.

Not modelled: motion during a ping, Doppler (Known gaps).

## M5: dataset sweeps and the first generation (2026-10-26)

### Remaining physics from the Known gaps

**Bubble multipath, bistatic.** The bubbles' image paths now cover every multipath boundary (surface and the four
walls, before: the surface only), and the paths in and out by different routes use the gas sphere's bistatic
cross-section: the modal series |f(θ)|², as a ratio to the backscatter on a (log ka, θ) table, each averaged over
±10 % in ka so that the ideal sphere's narrow interior resonances do not dominate. Checks (tests/test_scattering.py):
f(π) is the modal backscatter, ∫|f|² dΩ the modal extinction (lossless sphere, optical theorem, 0.2 %), forward
scattering dominates at ka 12 (> 30x backscatter), nearly isotropic at ka 0.05. Cost: +3.2 ms per ping (bubbles
1.9 → 5.1 ms). Effect, tank exposed: the plume-caused echo energy changes by less than 0.1 dB (the direct echo
dominates); the all-paths label grows 46 % at roll 0° (4929 → 7216 pixels: the wall ghosts of the plume), 1 % at
roll 90°; the direct-path label is unchanged.

**Surface boil.** uw/surface.py: the bubble simulation counts bursts per 1 cm surface cell around where the plume
surfaces (integer atomics: deterministic). Per ping each cell gets a smoothed burst rate (0.3 s), a disturbed share
f = 1 − exp(−rate / 20 000 m⁻²s⁻¹), and a point scatterer jittered in the cell (new phase every ping) with the
Kirchhoff backscatter of a rough pressure-release surface (rms slope 0.3 per axis; the static scene's convention).
It goes to the sonar as generic point scatterers (direct path, counted in the all-paths label only); sonar.py stays
independent of the bubble code. Checks (tests/gpu/test_surface.py): a saturated cell straight above the sonar has
σ = A / (8π s²); the boil's energy shows up in the all-paths label and not in the direct one; bursts land where the
plume surfaces (tests/gpu/test_bubbles.py). In the default geometry the boil is outside the vertical beam (+47°) and
adds 0.1 dB of plume-caused energy with the fan vertical. Its parameters are assumed.

### Leaks for sweeps

Leak presets (config/presets/leak): pinhole 0.1–5 L/min (d50 2 mm), crack 5–50 (3 mm), rupture 50–500 (5 mm, pool
400 k). Source `flow_jet`: the set flow (at the hole's pressure) with the jet of the hole that gives it at the pipe
pressure (mass flow ∝ hole area: 0.06 mm for 0.1 L/min, 4.4 mm for 500 L/min at 3 bar g). Time profiles: constant,
ramp (0 → full over ramp_s, from the first ping), intermittent (on for duty of each period). Checks: emitted gas
1/8 of the constant leak after 1 s of a 4 s ramp, 30 % with a 30 % duty (±8 %); flow_jet gives the set flow exactly.

### Sweep tooling

`tools/sweep.py` samples scenarios from `config/sweeps/v0.1.yaml` (deterministic for its seed), assigns splits by
scenario (largest remainder per stratum: kind / pipe preset / flow decade), and generates them with worker
containers that claim scenarios with an O_EXCL file, write them through the flat FileSink and mark them DONE or
FAILED. The orchestrator pauses (stops its workers) for f1tenth runs, a PAUSE file, or less than 27 GB free on /;
a scenario without DONE is redone from scratch. Scatterers are not cached for sweep scenarios (60–300 MB each:
every scenario has its own geometry). tests/test_dataset.py: sampled scenarios are valid configs with flows inside
their preset's range and unique seeds; the splits are ~70/15/15 overall and every stratum of ≥ 10 has all three;
the loader returns the polar image, masks with row 0 at range 0, targets.

Pilots (12 + 9 scenarios × 5 pings, every kind, profile, mobile preset, both sizes and modes): one failure, a
distractor whose `--set materials.steel_coated.rms_height_m=1e-05` YAML 1.1 read as a string; --set now takes
exponents without a dot as numbers.

### First generation: v0.1 (linus, 2026-10-26)

`python3 tools/sweep.py run` (config/sweeps/v0.1.yaml, physics git 2ac4675), 2 workers, data on the HDD
(`/HDD2T/underwater_data/v0.1`; / kept at 31 GB free):

| | |
|---|---|
| scenarios | 300: 236 leak, 50 negative, 14 distractor; 0 failed, no pauses |
| pings | 18 000 (60 per scenario at 10 Hz); 12 900 LF, 5 100 HF; 2 580 from a moving sonar; 2 460 with the fan vertical |
| splits (scenarios) | train 209, val 49, test 42 |
| size | 25.7 GiB (1.53 MB per ping on average: LF tank 0.93 MB, HF and basin more) |
| time | 71 min wall clock, 4.2 pings/s; 2.32 h of worker time, 15–90 s per scenario (scene and scatterer sampling dominate: 12–20 s tank, about 60 s basin) |
| ping | 31 ms fixed, 183 ms moving (static scene recomputed every ping) |

Sanity check (`tools/dataset_stats.py`; `stats/report.md`, `overview.png`, `examples.png` in the dataset):
- Image checks on 1000 random pings: no NaN outside the blind zone, no empty or saturated images (0.08 % of pixels at
  255 on average), no label pixels in negative or distractor scenarios.
- **The leak is often out of the main beam.** 45 of the 236 leak scenarios (19 %) never have a bubble in a cell's
  main lobe (mask_geom empty in every ping); 10 751 of 14 160 leak pings (76 %) have it in view. The random fixed
  pitch (10–45°) and yaw (±30°) miss the leak's elevation in about a fifth of the poses.
- **mask_direct is much larger than mask_geom** (median 6 000 vs 716 pixels in the leak pings), and 3 169 leak pings
  (22 %) have mask_direct without mask_geom: the plume seen through the vertical sidelobes, and the arcs of its
  sidelobe leakage (rings across the fan at the plume's range when it is strong and close). As designed; a
  consumer should pick the label for the task.
- 1 536 leak pings have the leak off (ramp start, intermittent off phase) while bubbles are still in the water:
  labels without flow, by design.
- **The leak's range barely varies**: 1.5–2.3 m for most pings in view, in the basin too, because the pipe stays at
  x = 1.65 m in both sizes and the fixed sonar is always on the -x wall.
- Distractors are few (14; the test split has one).

### Second generation: v0.2 (linus, 2026-10-26/27)

`config/sweeps/v0.2.yaml` at git 6893055, `/HDD2T/underwater_data/datasets/v0.2`, 22:33–00:10 (97 min). Changes
from v0.1: 80 % of the fixed views aimed at the leak, the sonar pole on any of the four walls with the leak 1–2.5 m
(tank) or 2.5–7 m (basin) away, 25 % distractors, three views of each fixed scene (30 pings each), a common-noise
stripe at bearing 0 in half the scenarios, no zlib (`recorder.compress=false`, `png_level=0`).
`tools/dataset_stats.py` on both datasets (1000 random pings image-checked each, seed 1):

| | v0.1 | v0.2 |
|---|---|---|
| scenarios (leak / negative / distractor) | 300 (236 / 50 / 14) | 400 (246 / 53 / 101) |
| distractor share (scenarios) | 4.7 % | 25.3 % |
| pings | 18 000 (60 per scenario) | 36 000 (90; fixed: 3 views × 30) |
| fixed / mobile scenarios | 257 / 43 | 347 / 53 |
| sonar wall (x0 / x1 / y0 / y1) | all x0 | 216 / 68 / 58 / 58 |
| splits, scenarios (train / val / test) | 209 / 49 / 42 | 281 / 60 / 59 (70.3 / 15.0 / 14.8 %) |
| splits by kind (train / val / test) | leak 164/38/34, neg. 35/8/7, distr. 10/3/1 | leak 173/37/36, neg. 37/8/8, distr. 71/15/15 |
| splits by size (train / val / test) | tank 150/35/31, basin 59/14/11 | tank 198/42/38, basin 83/18/21 |
| scenarios in more than one split | none | none |
| **leak scenarios never in view** (mask_geom empty in every ping) | **45 of 236 (19.1 %)** | **1 of 246 (0.4 %)** |
| leak pings in view | 10 751 of 14 160 (75.9 %) | 20 645 of 22 140 (93.2 %) |
| fixed leak views in view: aimed / random | – | 527 of 527 / 97 of 130 (75 %) |
| leak range, pings p5 / p50 / p95: tank | 1.21 / 1.75 / 2.10 m | 1.06 / 1.73 / 2.37 m |
| leak range, pings p5 / p50 / p95: basin | 1.15 / 1.97 / 3.02 m | 2.01 / 4.15 / 6.71 m |
| leak pings with mask_direct but no mask_geom | 3 169 (22 %) | 1 461 (6.6 %) |
| label pixels per leak ping, median: geom / direct / all | 716 / 5 998 / 7 880 | 1 459 / 10 302 / 13 392 |
| leak pings with the leak off (ramp start, intermittent) | 1 536 | 1 529 |
| image checks (NaN, empty, saturated, labels without a leak) | no flags; 0.08 % of pixels at 255 | no flags; 0.13 % at 255 |
| ping: fixed / mobile | 31 / 183 ms | 38 / 271 ms |
| throughput, worker time | 4.2 pings/s, 2.3 h | 6.2 pings/s, 3.1 h |
| size | 25.7 GiB, 1.53 MB per ping | 202 GiB (216.6 GB), 6.0 MB per ping |

- The aiming works: every aimed view has the plume in a cell's main lobe, and the random views (20 %) are in view
  about as often as v0.1's were (75 %). The one leak scenario never in view, s0100, is a mobile lawnmower survey in
  the basin that passes the plume at a distance with the leak outside the fan's slab.
- The basin now gives the ranges it was added for (median 4.2 m, up to 6.7 m); the tank's stay at 1–2.4 m.
- Splits stay balanced per kind and per size. No held-out domain was set (both sizes in every split).
- The example grid (`stats/examples.png`) shows the plume in view at varied ranges and angles to the pipe, the
  bearing-0 stripe where it was switched on, and the energy label's leakage arcs around strong close plumes as in
  v0.1 (mask_geom stays on the column).
- Size: four times the 60–90 GB expected from the 8-ping measurement. Recompression: open, see PLAN.md Dataset v0.2.

### Recompressing v0.2 (measured 2026-10-27, applied 2026-10-28, in progress)

v0.2 was written with `recorder.compress=false` and `recorder.png_level=0` to keep zlib off the worker's critical
path, which cost 6.02 MB per ping. What that costs back in disk, and whether the planned uint16 change helps, was
measured read-only on the median-size scenario of each (size × motion) cell, 90 pings each: `s0077` tank/fixed,
`s0324` tank/mobile, `s0141` basin/fixed, `s0331` basin/mobile. Every candidate encoding went to an in-memory
buffer; the dataset was not modified.

Where the bytes are (per 20 pings of `s0077`, raw → deflate):

| | stored now | deflate / PNG level 6 | ratio | distinct values |
|---|---|---|---|---|
| `polar_db` (float16) | 17.94 MB | 12.09 MB | 1.48x | – |
| `plume_share_all_u8` | 8.97 MB | 2.21 MB | 4.06x | – |
| `plume_share_direct_u8` | 8.97 MB | 1.97 MB | 4.56x | – |
| polar PNG | 8.99 MB | 5.67 MB | 1.6x | 256 |
| `mask_all` PNG | 8.99 MB | 0.31 MB | 29x | 2 |
| `mask_direct` PNG | 8.99 MB | 0.28 MB | 33x | 2 |
| `mask_geom` PNG | 8.99 MB | 0.07 MB | 136x | 2 |

The masks are binary images in uncompressed 8-bit planes, which is where most of the waste is (in the basin, where
the plume covers less of a longer image, `mask_geom` deflates 860x). `polar_db` is the incompressible part.

**`polar_db` as uint16 is rejected.** Deflated size relative to the float16 in place:

| encoding | `s0077` | `s0331` |
|---|---|---|
| float16 (now) | 100 % | 100 % |
| uint16, 0.01 dB | **106.7 %** | **105.1 %** |
| uint16, 0.05 dB | 100.5 % | 100.3 % |
| uint16, 0.1 dB | 93.0 % | 91.9 % |
| uint16 0.01 dB, hi/lo byte planes split | 94.0 % | 91.9 % |

Quantising to 0.01 dB puts noise in the low bits and deflate does 5–7 % *worse* than on float16's smooth mantissa.
Two more things the proposal had wrong: `polar_db` goes negative (min −1.73 dB in `s0331`, against +37.1 dB in
`s0077`), so `round(db * 100)` clips at zero and loses the value — the scheme needs an offset; and uint16 at
0.01 dB is the *more* precise of the two (0.005 dB against float16's 0.0625–0.125 dB at the 100–167 dB the images
live in), so it was never a precision win. 0.1 dB steps or a byte split buy ~7 %, not worth a format change.

**What pays is plain deflate on what the sweep wrote uncompressed**: npz `ZIP_DEFLATED` + PNG `compress_level=6`,
lossless and bit-identical on a round trip.

| scenario | now | recompressed | ratio | encode |
|---|---|---|---|---|
| `s0077` tank, fixed | 324.0 MB | 101.1 MB | 3.20x | 9.0 s |
| `s0324` tank, mobile | 324.0 MB | 83.9 MB | 3.86x | 7.2 s |
| `s0141` basin, fixed | 739.6 MB | 202.2 MB | 3.66x | 17.6 s |
| `s0331` basin, mobile | 739.6 MB | 188.8 MB | 3.92x | 18.3 s |

Projected over all 400 scenarios, weighted by the real cell counts (237 tank/fixed, 41 tank/mobile, 110
basin/fixed, 12 basin/mobile) and each scenario's manifest `bytes`: **216.6 → 62.4 GB (201.7 → 58.1 GiB), 3.47x,
6.02 → 1.73 MB per ping, 154 GB freed**, and 217 GB read + 62 GB written.
Re-encoding the masks as 1-bit PNG on top of that saves under 1 % (100.19 against 101.11 MB on `s0077`) and is not
worth the format change.

**The pass as run** (`tools/recompress.py`, started 2026-10-28 10:20, single-threaded under `nice -n 10 ionice -c3`;
log `/HDD2T/underwater_data/datasets/v0.2_recompress.log`). It finished at 22:54 the same day: **400/400 scenarios,
0 failures, 216.5 → 58.4 GB (201.7 → 54.4 GiB), 3.71x, 6.02 → 1.62 MB per ping, 158 GB freed** (`/HDD2T` went from
281 to 136 GB used). The *size* projection held and was slightly beaten; the *time* projection was wrong by about
8x, and the reason is worth recording.

| scenario size | encode only (measured 2026-10-27) | the pass as run | factor |
|---|---|---|---|
| 324 MB | 9.0 s | 65–67 s | 7.4x |
| 518 MB | – | 104–108 s | – |
| 740 MB | 17.6 s | 149–156 s | 8.9x |

That is ~113 s per scenario over the real mix, so **~13 h single-threaded for all 400** against the 1.27 h projected,
and the process sits at **95 % of one core: the pass is CPU-bound, not disk-bound at 35–50 min**. The projection timed
the *encoder* alone, into an in-memory buffer. The pass does far more than encode, deliberately: it decodes every
original, encodes into `<root>/.recompress/<id>`, then re-decodes **both** copies twice — once comparing the npz
arrays and the PNG pixels raw, once comparing what `uw.dataset.load_ping` returns from each — and replaces a
scenario's files only if all of that matches. Roughly five decodes and one encode per ping, in place of one encode:
that is the whole factor. The verification is the right trade for rewriting 216 GB in place; what was missing is that
nobody measured *it*, only the encoding it protects.

Parallelising would need a `--shard i/n` option, since `--limit` takes the head of the todo list and two instances
would pick the same scenarios and collide on the same `.recompress/<id>` directory. Not done: one core for 12.5 h is
cheaper than changing the tool while it rewrites the dataset, and `/HDD2T` had 1.5 TB free, so the 158 GB was not
urgent.

It beat the 3.47x projection because the npz side gains more than the PNG side loses: PNGs go through
`uw.sinks.write_png` (single IDAT, filter 0, so `read_png` keeps the fast path the loader uses in training) rather
than Pillow, which would compress them 5.66x instead of 4.85x but cost that path.

**After the pass**, checked independently of the per-scenario verification: the loader still sees all 36 000 pings
through the manifest, `polar_db` is still float16 at its original shapes (876 or 2001 range bins x 512 beams), the
masks still carry their labels, every PNG still has exactly one IDAT chunk (five random pings across tank and basin,
fixed and mobile), and `tests/test_dataset.py` passes. `recompress.json` in the dataset root records the final
numbers; the sweep's `manifest.json` is untouched, so its per-scenario `bytes` are the pre-recompression sizes by
design — `recompress.json` and the per-scenario `RECOMPRESSED` markers are where the current sizes live.

Conclusion for the recorder: keep `compress=false` / `png_level=0` while a sweep is running, since the measurement
that motivated them stands (zlib was 63 of ~100 ms per write), and recompress afterwards as a separate pass
(`tools/recompress.py`).

## Real data: public sonar datasets (2026-10-26)

Two public datasets were downloaded and compared with the sim. Nothing here is committed: the data lives in
`~/underwater_data/real/` (see its `SOURCES.md`), outside the repo.

| Dataset | Sonar | Format | Licence |
|---|---|---|---|
| UATD (Figshare [10.6084/m9.figshare.21331143](https://doi.org/10.6084/m9.figshare.21331143), arXiv 2212.00352) | Tritech Gemini 1200ik, 1200 kHz (709/800) or 720 kHz (91/800) | **polar** 8-bit BMP, 1024 or 512 beams × 1024–2047 range bins, per-ping XML (range, azimuth 120°, elevation 12°/20°, sound speed, frequency) | CC BY 4.0 |
| DeeperSense metadata (Zenodo [10373154](https://doi.org/10.5281/zenodo.10373154)) | Blueprint Oculus M1200d | metadata only, 1.97 GB (SQLite + CSV) | CC BY 4.0 |
| DeeperSense basin-1 (Zenodo [7728089](https://doi.org/10.5281/zenodo.7728089)) | Tritech Gemini 720i | Cartesian fan, grayscale JPEG 2896 × 1448 | CC BY 4.0 |
| DeeperSense basin-2 (same record) | Oculus M1200d, 2 MHz, 80° × 12° | Cartesian fan, inferno-mapped PNG 512 × 512 | CC BY 4.0 |

`tools/compare_real.py` grew two loaders (`uatd`, `fan`) beside `uw`; `tools/deepersense_index.py` indexes the
metadatabase. Our sonar is an M750d (LF 750 kHz, HF 1.2 MHz); the M1200d is 1.2 / 2.1 MHz and the Gemini 1200ik is
1.2 / 0.72 MHz, so only the HF end overlaps, and only statistics that do not depend on the absolute level or on
the maker's TVG are compared at all.

### Which recordings are in the lab tank

Zenodo 10373154 holds **no sensor data**: DFKI release it on demand, and the metadatabase exists to pick what to
ask for. `modality.modality_description` says the published image datafiles are a *Cartesian projection* of the
sonar output, so only the ROS bags carry raw polar beams — a request must name bags.

**89 Oculus M1200d streams at the Maritime Exploration Hall (`location_key` `dfki_basin`), 11.58 h, 151.1 GB of
bags**, in two sessions: 35 in 2022-02 and 54 in 2023-04. (The third lab session, 2021-08, used a Gemini 720i, not
the Oculus.) Every stream is a single `/oculus/sonar_image` topic of `acoustic_msgs/SonarImage` plus one
`dynamic_reconfigure/Config`. The full table is in `docs/deepersense_tank_recordings.md`. Settings across them:

| | |
|---|---|
| `freqMode` | high 74, low 21 |
| `range` | 8 m (49), 6.5 m (15), 10 m (13), 9 m (13), 7 m (5) |
| `gain` | 10–80 %, mostly 30 % |
| `gamma` | 127 throughout (127/255 ≈ 0.5, so the 8-bit sample is roughly an amplitude) |
| `data_16bit` | **False in 92 of 95 configs** |
| `send_gain` | True in 9 (the per-row TVG gain is then recoverable) |
| `gain_assistance` | True in 45 (automatic gain: the level is then not comparable between pings) |

**Worth requesting, in this order** (17.3 GB in total, but the first two are 0.85 GB together):

1. `2023-04_dfki_basin/2023-04-27-10-13_test_high_data_res_low_freq/` (0.12 GB, 28 s, 283 pings, 16-bit, freqMode 1, 8 m)
2. `2023-04_dfki_basin/2023-04-27-10-15_test_high_data_res_high_freq/` (0.73 GB, 116 s, 992 pings, 16-bit, freqMode 2, 8 m)
   — the only two 16-bit lab-tank bags, and both are small. `gain_assistance` is on, so the level drifts between
   pings; they still settle the beam pattern, the ring-down and the speckle, which is what we need.
3. `2022-02_dfki_basin/2022-02-07-15-39_pipe_mounting_5/` and `…-15-52_pipe_mounting_6/` (2.69 GB each, 600 s,
   8-bit but `send_gain` True and `gain_assistance` False) — a fixed gain *and* the per-row TVG, so the applied
   gain can be divided out. The scene (a pipe being mounted) is also the closest to ours of anything on offer.
4. `2022-02-07-16-07_diver_moving_around_1` (2.81 GB, freqMode 1, 7 m, `send_gain` True) for the low-frequency mode.

Nothing in the corpus has a calibration sphere, so none of it fixes an absolute level or an array sidelobe.

### What the comparison measures, and what a published image destroys

Real recordings are 8-bit display images, not calibrated pressure, so `compare_real.py` reads a pixel as an
amplitude raised to a power p (intensity dB = (20/p) log10 v) and `--fit-pixel-law` can solve for the p that makes
the background a Rayleigh speckle. Fitted on a background window: **UATD p = 1.10, basin-1 p = 1.02** — both are
plain linear amplitude, which is now the default (`--pixel-exponent 1`). basin-2 fits p = 0.33, but at that
exponent its sidelobe reads −26.7 dB and its TVG 179 dB/decade, both nonsense: the fit is compensating for a lost
speckle, not finding a gamma.

Before trusting any number off a Cartesian fan image, the whole pipeline was run in reverse on data we can check:
20 UATD pings at 10 m were rendered into DeeperSense-style square fan images (aspect not kept, inferno, 8-bit) and
read back through the same `fan` loader, so the same scene is measured before and after the display pipeline.

| | native polar | fan 512 | fan 1024 | fan 2048 | 2048→1024 bilinear | 2048→512 bilinear |
|---|---|---|---|---|---|---|
| background contrast | 1.046 | 0.934 | 0.957 | 0.983 | 0.814 | 0.677 |
| KS vs Rayleigh | 0.070 | 0.124 | 0.112 | 0.099 | 0.167 | 0.217 |
| brightest target width [deg] | 0.96 | 0.63 | 0.75 | 0.90 | 0.87 | 1.25 |
| ring-down slope [dB/m] | −47 | +53 | +57 | +82 | | |
| TVG fit [dB/decade] | 41.5 | 141.7 | 145.6 | 153.5 | | |

- **Speckle contrast and angular width survive** a nearest-neighbour fan rendering at a decent size (contrast
  within 6 %, width within 6 % at 2048). An *interpolating* resize is what destroys them: 2048 → 512 bilinear
  takes the contrast to 0.68.
- **The near-range profile and the TVG fit do not survive at all** — the sign of the ring-down slope flips. The
  apex of a fan image is where its sampling is worst, and that is exactly where those two are measured.
- So from a published fan image: use the background statistics and (at high resolution) the beam width; never the
  blind zone, the ring-down or the TVG. basin-2's 512 × 512 is past even the first of those.

### Sim vs real

50 pings each, `output/compare_real/` (`compare.json`, `compare.png`). The fan datasets have no known display
range, so their range axis is a fraction of the outer radius and their windows were set in those units.

| Measure | sim `smoke_exposed` (M750d LF, tank, 3.5 m) | UATD (Gemini 1200ik) | basin-1 (Gemini 720i, fan) | basin-2 (Oculus 2 MHz, fan) |
|---|---|---|---|---|
| background contrast | 2.013 | 0.986 | 1.022 | 0.332 |
| K shape | 0.7 | 13.2 | 46.1 | ∞ |
| KS vs Rayleigh | 0.230 | 0.089 | 0.014 | 0.349 |
| brightest target width [deg] | 2.44 | 1.02 | 1.14 | 0.80 |
| "sidelobe" [dB] | −2.1 | −6.0 | −5.1 | −8.5 |
| blind zone | 0.100 m | 0.006 m | 0.003 | 0.003 |
| ring-down slope [dB/m] | −106 | −29 | −44 | +13 |
| level vs range [dB/decade] | −158.9 | +23.7 | −32.1 | +56.6 |

**What matches.** Where the model is supposed to give fully developed speckle, it does, and so does reality: our
flat-sand test is contrast 0.984 (M2, above), UATD's lake bottom is 0.986–1.046 and basin-1's is 1.022, all within
a few per cent of the Rayleigh 1. UATD's brightest-target width of 1.02° also sits on its sonar's ~1° resolution,
just as our measured 0.982° sits on our designed 1.0° — the measurement recovers a datasheet beam width from an
image, which is the method we would apply to an Oculus bag.

**What does not match.** Our *scene* is far more heterogeneous than any real recording here: at 2.15–2.8 m the
tank gives contrast 2.0 at ±25° and 2.3–3.7 in every other bearing window tried, against ~1.0 for real bottom.
Both real datasets stay near Rayleigh over wide windows; our tank has no window at that range that does. The
contributors are known and already listed as open gaps above — specular walls, corners and the pipe built from
random-phase facets, plus rocks, ripples and the plume shadow — and the round-trip control rules out the display
pipeline as the explanation (nearest-neighbour rendering kept basin-1's contrast at 1.02, and bilinear resizing
only ever *lowers* contrast, never raises it). It is not settled whether this is a model defect or simply a
smaller and more cluttered tank than the DFKI basin: that needs a recording from a tank of our geometry, which is
what the bag request above is for.

**What is not comparable.** The absolute level (nothing here is calibrated), the TVG profile (the maker's TVG is
applied in the 8-bit data and is only recoverable from a `send_gain` bag), and the sidelobe level: neither dataset
has a point target, so the −2 to −9 dB in that row is the skirt of an extended target, not an array sidelobe.

### What makes the tank's background contrast 2.0–3.7 (2026-10-28)

The real-data comparison above left the tank's background contrast at 2.0 (±25°) and 2.3–3.7 in the other bearing
windows, against ~1.0 for real bottom, with "specular walls, corners and the pipe built from random-phase facets,
plus rocks, ripples and the plume shadow" as a list of suspects. Seven paired 60-ping runs of the tank scene, same
seed (7) and same ping count, now separate them. Contrast is `sonar_stats.background` over 2.15–2.8 m, averaged
over the 60 pings (±1 sd of the per-ping spread), in the windows the comparison used. This needed the
`KeyError: 'rms_slope'` fix below, which is why it could not be run before.

| window | baseline | glints off | glints off, no wall paths | no leak | no leak, no wall paths | flat sand, no leak | flat sand, no wall paths |
|---|---|---|---|---|---|---|---|
| ±25° | 2.129 | 2.129 | 2.142 | 2.102 | 2.108 | **0.981** | 0.980 |
| +(35–55)° | 2.237 | 2.458 | 1.640 | 2.141 | 1.577 | 1.855 | 1.531 |
| −(35–55)° | 2.180 | 1.963 | 1.537 | 2.502 | 1.653 | 1.908 | 1.510 |
| +(40–60)° | 1.835 | 1.869 | 1.217 | 1.878 | 1.239 | 1.480 | 1.130 |
| −(40–60)° | 1.835 | 1.717 | 1.188 | 1.897 | 1.225 | 1.511 | 1.153 |

Per-ping sd is 0.002–0.025 throughout, so every difference above 0.05 is real. The runs are
`output/probe/coh_{baseline,diffuse,diffuse_nomp,noleak,noleak_nomp,flat,flat_nomp}`, "glints off" being
`materials.concrete.coherent=false materials.steel_coated.coherent=false`, "no wall paths"
`sonar.multipath=[surface]`, and "flat sand" additionally `rocks.count=0 sand.ripple_amp=0
sand.undulation_amp=0`; `output/probe/coh_contrast.py` reads them.

**The coherent glints are not the cause.** Switching the whole Kirchhoff specular term off leaves ±25° at 2.129,
identical to three decimal places, and moves the wall windows by ±0.2 in *either* direction — the specular energy
it removes comes back as the incoherent part of the same surfaces (factor 1 − χ²), which is what the model
intends. That suspect is cleared.

**The plume is not the cause either.** Switching the leak off leaves ±25° at 2.102. It does move the wall windows
by up to ±0.3, in both directions (extinction and shadowing of the wall image paths, which fall differently on the
two sides), so it perturbs those windows without explaining their level.

**The central window is sand relief.** With the ripples (8 mm), the undulation (2 cm) and the 14 rocks removed,
±25° falls from 2.10 to **0.981** — the Rayleigh 1.0, and the same 0.98 the flat-sand speckle test gives (M2).
So the excess at ±25° is entirely the relief of the modelled bottom: resolved ripple and rock structure inside the
window, which is scene content, not speckle statistics. Nothing is wrong with the speckle model.

**The wall windows are relief plus wall image paths.** On the flat scene the wall windows still read 1.48–1.91,
and dropping the specular wall image paths takes them to 1.13–1.53; on the full scene the same change takes
2.14–2.50 down to 1.54–1.65. So those windows carry two contributors of similar size, the bottom relief and the
walls' mirror paths, and no single switch brings them to 1.0.

**What this means for the real comparison.** The tank's contrast is high because the tank is a small, cluttered,
rippled box seen at 2–3 m, not because of a defect in the scattering model: every mechanism that could have been
a defect (the coherent term, the plume) is ruled out, and the one that dominates is geometry we chose to put in
the scene. Comparing like for like against the DFKI basin's ~1.0 therefore needs either a window without ripples,
rocks and walls in it, or a recording from a tank of our geometry (the bag request). Two consequences for the
parameter table: `sand.ripple_amp` and `rocks.count` move from "scene dressing" to parameters the background
statistics are sensitive to, and the open gap "specular walls, corners and the pipe from random-phase facets" is
narrowed to the wall image paths alone.

### `KeyError: 'rms_slope'` with the coherent term off (fixed 2026-10-28)

`uw/sonar.py` reads `rms_slope` for every material when it uploads the scatterers, but
`geometry.surface_materials` derived it only inside `if m.get("coherent")`, so `materials.<m>.coherent=false`
crashed the run after 8.8 s — which is what blocked the comparison above on 2026-10-26. The slope is a property of
the surface (√2 h / correlation length of its Gaussian profile), while `coherent` only selects whether the
mirror-like Kirchhoff term is computed on top, so it is now derived whenever a material carries `rms_height_m` and
`correlation_m` and does not already state a slope. A wall's own `rms_height_m` is likewise applied whatever the
flag says, so forcing the walls diffuse no longer silently falls back to the base material's roughness.
`tests/test_geometry.py` covers both (every material keeps a positive, unchanged slope with the flag off; a wall
keeps its own roughness); both fail without the fix.

Side measurement from the paired runs, per ping on linus: the coherent facet integral costs 41.2 → 28.0 ms, and
the specular wall image paths a further 28.0 → 17.0 ms, of the sonar's time in the tank scene with the plume.

### Which UNVERIFIED parameters the data constrains

| Parameter | Verdict |
|---|---|
| `ringdown_db` 60, `ringdown_m` 0.08 | **Weakly constrained.** UATD's beam-averaged level peaks 23.6 dB at range 0 and is back within 1 dB of the local background (≈13 dB) by 0.02 m, −10 dB by 0.14 m; the fitted slope is −72.9 dB/m over the first 0.1 m. Our near-range excess is of a similar size (ring-down sits 7–22 dB over the sand at 0.1 m) but reaches ~0.3 m, several times further. Not changed yet: a Gemini is not an Oculus, and the Gemini's own TVG lifts the far side of that curve. Settled by request 1–2. |
| Beam spacing `uniform_angle` | Not constrained. UATD packs 1024 beams into 120° for a ~1° beam, which is consistent with uniform-in-angle but would be consistent with much else too. |
| `array_taper_sll_db` −28 | **Not constrained.** Needs a point target; nothing in either dataset has one. |
| `focus: true` (near-field focusing) | **Not constrained.** UATD works at 7–25 m, the far field for that array; the question only bites inside ~8 m. |
| `vertical_taper` hann, `vertical_edge_db`, `fan_edge_db` | **Not constrained.** Needs a flat bottom at a known altitude and known geometry; neither dataset documents the mounting. |
| Speckle model (Rayleigh, K shape) | **Confirmed** for homogeneous bottom, at contrast 1.0 ± 0.05 in two independent real datasets. |
| Pixel law of a published 8-bit image | **Newly measured**, not previously assumed: p = 1.0–1.1, i.e. linear amplitude, for both a Gemini 1200ik and a Gemini 720i. |
| `receiver_noise_common_db` (new, off) | **Existence shown, level not.** basin-2 has a broadside stripe of +3.4 dB (pixel as amplitude) to +10.2 dB (fitted law); see the subsection below. Settled by requests 1–2. |

### A stripe at bearing 0 on an Oculus M1200d

basin-2 has a bright stripe straight ahead. Measured on 200 frames read back to polar with the `fan` loader above
(`--fan-colormap inferno --fan-fov 80`): the mean intensity of the beams within ±0.5° of broadside over that of the
beams at 3–10° on either side (± = spread between frames).

| | far half (0.55–0.95 of the range) | near half (0.15–0.5) |
|---|---|---|
| basin-2, pixel read as amplitude (p = 1) | **+3.43 ± 0.17 dB** | +1.54 ± 0.25 dB |
| basin-2, pixel law fitted to the background (p = 0.325) | **+10.22 ± 0.59 dB** | −1.40 ± 1.87 dB |
| UATD (Gemini 1200ik, 20 pings at 10 m), native polar | −1.43 ± 1.24 dB | |
| the same UATD pings after a 512 × 512 inferno fan round trip | −1.19 ± 2.35 dB | |

- **It is in the sonar, not in the reading.** The fan round trip through the same loader draws no stripe on UATD,
  and in basin-2 it holds in every frame with a 0.17 dB spread while the divers move about.
- It is stronger in the far half, where the TVG-amplified background is closest to receiver noise, and it is about
  as wide as a beam (0.6–0.9° at half power).
- **Its level is not known.** It rests on the pixel law, which a 512 × 512 fan image cannot pin down: +3.4 dB if a
  pixel is an amplitude, +10 dB at the fitted exponent.

Candidate mechanism: receiver noise (or a DC offset) common to all elements, which sums coherently in the broadside
beam. Added to the model as an option, **off by default**: `sonar.receiver_noise_common_db`, the common share of the
per-element noise variance. Each beam's noise grows by share × |Σ w_m exp(i φ_m)|² / Σ w² (test
`test_common_mode_noise_is_a_stripe_at_broadside`, measured against that formula within 0.3 dB). On our LF array
(Σw)²/Σw² = 20.4 dB, so a share of −20 dB draws +3.2 dB at broadside and −10 dB draws +10.8 dB: exactly the span
basin-2 allows, if its far-half background is receiver noise. With −10 dB:

| | 0.1–0.5 m | beyond 2.5 m | beams at 8–12° |
|---|---|---|---|
| array not focusing | +10.1 dB | +9.9 dB | < 0.01 dB |
| array focusing (default) | +3.1 dB | +9.1 dB | 0.6 dB near, < 0.01 dB far |

A focusing array applies its focusing law to a signal that has no wavefront curvature, so it spreads the stripe at
near range; the weaker stripe in basin-2's near half fits that, though it does not prove it.

**Why off.** New physics goes in when real data shows a gap it closes. This data shows that a stripe exists, not its
level, and on an M1200d, not on our M750d. Requests 1–2 above (the two 16-bit lab-tank bags) settle it: raw polar
beams, where the stripe at a fixed beam index can be set against the noise in water-only rows and the share read off
directly. Until then, v0.2 randomises the share (docs/PLAN.md).

## M3.7: the calibration harness, a leak-free hold-out, and basin-2 settled (2026-11-04)

Groundwork before any sonar parameter is allowed to move, so that a later "it matches now" is evidence rather
than a tautology. Nothing in the model changed in this step; one default-preserving field was added
(`generator_version`), and one earlier conclusion was overturned.

### The real corpora, extracted in full

Only a fraction had been unpacked, and the native-polar half is the only part of the corpus whose near-range,
ring-down and level-vs-range survive at all (the fan round-trip control above). Now extracted, 7 GB on
`/HDD2T`, 76 s:

| corpus | frames before | frames now | format |
|---|---|---|---|
| UATD `UATD_Test_1` | 80 | **800** | native polar BMP, 1024 beams × 1026–2047 bins |
| UATD `UATD_Test_2` | 0 | **800** | the same |
| DeeperSense basin-1 | 60 | **1427** | fan JPEG 2896 × 1448 |
| DeeperSense basin-2 | 60 | **3847** | fan PNG 512 × 512, inferno |
| `deepersense_divertrack` | 1193 | 1193 | fan JPEG, M1200d (never used before) |

`~/underwater_data/real` is now `chmod -R a-w`: a write is refused rather than merely discouraged, and
`holdout.json` carries a sha256 per frame so a corpus that moves anyway is detectable.

### The publishers' split cannot be used as a hold-out

`UATD_Test_1` and `UATD_Test_2` look like two recordings and are not. Measured over all 1600 frames, on the
cosine similarity of 32 × 32 mean-removed thumbnails:

| | |
|---|---|
| Test_1 frames with a Test_2 twin at cos > 0.99 | **273 / 800 (34 %)** |
| at cos > 0.95 | **639 / 800 (80 %)** |
| at cos > 0.90 | 743 / 800 (93 %) |
| bit-identical pairs across the two sets | 1 (`Test_1/00070.bmp` = `Test_2/00277.bmp`) |
| distinct `soundspeed` values | 171, **every one split ~50/50** between the two sets |

Range settings and class counts match to within a few per cent as well (balls 197 vs 194, 10 m frames 340 vs
332). It is one pool drawn in two halves, so calibrating on one and verifying on the other would be verifying
on the training data. The XML carries no timestamp or sequence id, so sessions have to be derived.

**What the split is instead** (`tools/real_holdout.py`, written once to `output/compare_real/holdout.json`).
Frames are grouped by single-linkage on the same similarity, and whole groups are assigned, stratified by
range setting and frequency — the by-scenario rule `tools/sweep.py` already applies to the simulated sweep,
for the same reason. The guarantee is structural: at threshold *T* no pair in two different groups can exceed
*T*, so assigning whole groups bounds cross-half similarity by *T*.

| corpus | groups (largest) | calibrate | hold-out | max cross-half similarity |
|---|---|---|---|---|
| uatd | 345 (429) | 980 | 620 | **0.950** |
| basin1 | 8 (547) | 852 | 575 | 0.942 |
| basin2 | 1 (3847) | 3847 | — | — |
| divertrack | 1 (1193) | 1193 | — | — |

Against 80 % of frames having a near-twin across the publishers' halves, that is the whole difference between
a hold-out and a second look at the same data.

**Two corpora refused to be split, and that is a result.** basin-2 and divertrack chain into a single group:
each is one continuous near-identical recording, so no subset of it is independent of the rest. They can
calibrate and they can be diagnosed; they cannot verify. The tool records the refusal explicitly rather than
returning an empty half, because an empty half that looked like a configuration slip would eventually be
"fixed" by splitting per frame, which is the leak the tool exists to prevent. `compare_real.py --split` fails
loudly on such a corpus for the same reason.

The split is lumpy where a group is large: the dominant stratum (10 m, 1200 kHz, 588 frames) splits 359/229,
but 8 m and 9 m went wholly to the hold-out and 13 m is 12 % of it. Unavoidable with whole-group assignment,
and preferable to the alternative.

### basin-2's contrast 0.33: the colour map was not the cause

The standing suspicion was that the 0.33 came from reading an inferno-mapped image as luminance (`read_gray`
only inverts the map when `--fan-colormap` is given, and the run that produced 0.33 did not give it), through
a ±25° window that straddles the broadside stripe. **That is wrong.** 200 frames, the same statistics:

| run | colour map undone | window | pixel law | contrast | K shape | KS vs Rayleigh |
|---|---|---|---|---|---|---|
| A, as originally measured | no | ±25° | p = 1 | 0.459 | ∞ | 0.257 |
| B, **the run never done** | **yes** | +8…+35° | p = 1 | **0.348** | ∞ | 0.327 |
| C, the fitted law | yes | +8…+35° | p = 0.325 fitted | 1.013 | 72.7 | 0.019 |

Undoing the colour map and moving off broadside does not recover the speckle — it reads *lower*, not higher.
And our own loader is exonerated by the round-trip control above: a genuine nearest-neighbour fan render at
512 px, read back through this same loader, keeps contrast at 0.934. So the speckle was destroyed by
**basin-2's own rendering**, upstream of us, and no choice of pixel law recovers it: run C reaches contrast 1
only because `--fit-pixel-law` forces it there, and the exponent it needs implies a 164 dB/decade TVG and a
−28.8 dB sidelobe, both nonsense.

**Verdict:** basin-2 is not a reference for speckle, ring-down or level-vs-range at any pixel law. It keeps
one job, the broadside stripe, which is a within-frame ratio and survives. Recorded in
`output/compare_real/targets.json` under `admissibility`.

### UATD does contain compact targets

"Nothing in the corpus has a calibration sphere, so none of it fixes an absolute level or an array sidelobe"
(above) is half wrong. The 800 `UATD_Test_1` annotations hold **197 `ball` objects** (plus 172 cubes, 150
ROVs, 76 human bodies); `UATD_Test_2` holds 194 more. For an *absolute level* the original statement stands —
no target of known target strength, so no absolute calibration. For the **array sidelobe** it does not: a
sphere is compact and isolated.

But a ball is not a point target either. Measured over the 22 balls in the frames that were already
extracted, the annotated boxes span **5.4–15.0° in bearing** (median 6.3°) and 0.39–0.62 m in range, at 3.5–11
m, against a ~1° beam. So the skirt around a ball carries the target's own angular extent as well as the
array's sidelobes, and what can be read off it is an **upper bound**. That is what the new
`sonar_stats.far_sidelobe_floor` measures, and it is labelled a bound in `targets.json`; `array_taper_sll_db`
stays at −28 dB and stays UNVERIFIED.

### The level-vs-range row was comparing two different quantities

`display_kernel` applies `k log10 R + 2αR − nf_db` and the `display_db` window to the **8-bit image**, while
the stored `polar_db` stays absolute dB re 1 µPa. A published recording is always the first kind, with the
maker's equivalent chain already applied and not recoverable. The sim-vs-real table's −158.9 dB/decade
against UATD's +23.7 was therefore a physical level set against a displayed one, not a disagreement about a
TVG. `compare_real.py` grew `--uw-channel`:

- `db` — `polar_db` as before, absolute dB: the physics, and **not comparable with a real recording on level**;
- `tvg` — our own gain chain reapplied, relative to the beam noise floor: the shape to compare, and the safe
  default for anything level-related;
- `img` — that, encoded into 8 bits **the way a real head does it**, see below.

`sonar_stats.tvg_profile` and `near_range` also gained `normalise="median"`, which quotes a profile against
its own median over a stated window so a sim in physical dB and a real recording in sensor units can share an
axis.

### Our 8-bit display is linear in dB; a real head's is linear in amplitude

Found while building the `img` channel, and it matters beyond this comparison. Our display maps
`sonar.display_db` = 0–80 dB linearly onto 0–255, so a pixel *is* a dB value. An Oculus or Gemini 8-bit
sample is linear in **amplitude** — that is what the pixel-law fit measured on two real Geminis, p = 1.0–1.1.
The two encodings are not interchangeable:

| | sand at 25 dB | 30 dB | 35 dB | 40 dB |
|---|---|---|---|---|
| our encoding, linear in dB, window 0–80 | pixel 80 | 96 | 112 | 128 |
| a real head, linear in amplitude, saturating at 80 dB | pixel 0 | 1 | 1 | 3 |
| the same, saturating at 45 dB | pixel 26 | 45 | 81 | 143 |

Reading our image back through `pixels_to_db`, which assumes amplitude, is then a double conversion: it
compresses those 15 dB of sand into 4 dB and took the measured background contrast from 2.13 to **0.43**.
That is an artifact of the conversion, not a property of the sonar, and it is the same mechanism that
destroyed basin-2. So `img` now goes to amplitude, quantises, and reads back through the law a real image is
read with. An amplitude display has **one** knob, the gain that sets which level saturates
(`--uw-amp-top-db`), and it has to be set so the seabed sits mid-scale, exactly as a real head's gain is.

Two consequences worth keeping. The quantisation is coarse in the dark: 40 dB below saturation a 0.5 dB
difference lands on the *same* pixel, so speckle there is destroyed by the encoding rather than measured
through it. And v0.2's training images are the log-scaled kind, which is not what a real head produces —
relevant to any sim-to-real transfer, and not something this pass changes.

### Contrast off a display image is gain-dependent, so it needs an admissibility gate

The `img` channel does **not** make the tank's contrast agree with reality, and an early reading that it did
was wrong. Sweeping the one knob, same 60 pings, same window:

| gain saturates at | 35 dB | 40 | **45** | 50 | 55 | 60 | 70 | 80 |
|---|---|---|---|---|---|---|---|---|
| background contrast | 0.497 | 0.715 | **1.010** | 1.402 | 1.787 | 1.959 | 2.091 | 2.110 |
| K shape | ∞ | ∞ | 97.4 | 2.1 | 0.9 | 0.7 | 0.6 | 0.6 |
| KS vs Rayleigh | 0.321 | 0.175 | 0.132 | 0.221 | 0.267 | 0.284 | 0.301 | 0.297 |

Contrast spans 0.50 to 2.11 as the gain moves, and passes through 1.0 at 45 dB. So "sim 1.01 against UATD
1.10" was produced by choosing a gain, not by the model improving. **Contrast measured on a display image is
not a property of the sonar or the scene alone.**

What makes the comparison honest is asking whether a given display is clipping, and K shape and KS answer
that: a clipped image loses its bright tail, so K goes to ∞ and KS rises. At a gain of 70–80 dB the sim is
unclipped and its displayed contrast equals its physical one (2.09–2.11 against 2.13) — the display is
transparent. And the real data is unclipped too, measured directly on the raw pixels of 60 calibrate frames:

| | share of pixels at 255 | at 0 | 99.9th percentile pixel |
|---|---|---|---|
| UATD_Test_1 | **0.000 %** (max 0.002 %) | 5.5 % | 62.8 of 255 |
| basin-1 | 0.000 % | 52 % (the fan corners are outside the sector) | — |

UATD uses only the bottom quarter of its 8-bit range and saturates nothing, so its contrast of 1.095, with
K = 9.3 and KS = 0.088, is a genuine measurement of its scene.

**So the comparison that stands is unclipped against unclipped: sim 2.11, UATD 1.095, basin-1 1.101** — and
the gap is a real difference between the scenes, which is what the 2026-10-28 ablation already concluded
(sand relief and wall image paths in a small cluttered tank, against a flat basin bottom). The display work
does not close that gap and was never going to; what it buys is the ability to tell a display artifact from a
scene property before quoting either. That gate is now in `targets.json`: no statistic is read off a display
image whose K shape is ∞ or whose KS exceeds 0.15.

### The TVG fit constants cannot carry a comparison

`log10(R)` and `R` are collinear over any window a sonar actually covers — correlation 0.997 over 1.8–3.4 m,
0.992 over 3.0–8.0 m, condition number around 400 — so `k` and `b` trade off against each other almost
freely while the fitted curve barely moves. Measured over 60 UATD pings at one range setting, the slope comes
out **59.7 dB/decade with a per-frame spread of ±106**: the number is noise. The two fits of our own tank
scene, k = −158.9 before this pass and k = +82.9 after, describe curves that each span about 5 dB over the
window and differ in shape by 2.7 dB rms — 242 dB/decade of apparent disagreement for 2.7 dB of real one.

So the old table's "−158.9 against +23.7 dB/decade" row was comparing two unstable numbers, and that row is
not a finding. `sonar_stats.profile_distance` replaces it: rms and max difference in dB between two
median-normalised curves, insensitive to level and to sampling, which is what `targets.json` now sets a
tolerance on (rms ≤ 3 dB, max ≤ 6 dB).

One more method requirement fell out of this. UATD's range setting changes from ping to ping — 7 to 25 m
among 60 consecutive frames — so a fixed absolute range window samples a different part of the image in each,
and a near-range or level-vs-range statistic pooled over the corpus averages over incomparable things.
`compare_real.py --uatd-range` stratifies by range setting, and for those two metrics it is not optional.

### Smaller fixes in the same pass

- **Profiles are averaged over frames, on a common range grid.** UATD's range setting changes from ping to
  ping, so the frames share no range axis and the old code quoted frame 0 alone — one noisy profile standing
  in for fifty.
- **Every loader argument is recorded** in `compare.json`. The run that produced contrast 0.33 could not be
  explained from its own artifact, because the colour map it did not undo was never written down.
- **The loaders have tests** (`tests/test_compare_real.py`, 22 of them): the pixel law and its zero floor, the
  colour-map inverse, sector fitting and fan→polar resampling against drawn sectors with known geometry, UATD
  axes from the XML, the pixel-law fit recovering a planted exponent, the gain chain against
  `display_kernel`'s own formula, the blind zone surviving it, and the split gate's refusals. They were
  entirely untested before, and every number in the sim-vs-real table went through them.
- **One test pins the trap**: after `--fit-pixel-law` the contrast is 1 by construction, so contrast is then
  no evidence at all and only the KS distance and the other statistics are.

### `generator_version`

`config_hash` hashes the resolved config, so a change to the maths in `uw/sonar.py` is invisible to it and
only the repository-wide git hash moves — too coarse to tell a dataset made by a different sonar model from
one made by the same model with a tidier docstring. So `uw/sonar.py` now declares `SONAR_MODEL_VERSION`
(`m750d-1`, bumped when a change alters what a ping contains, not for refactors), and
`recorder.generator_version` composes `<model>/<git>/<config hash>` into every ping's JSON and every
scenario manifest. Joined with `/`, because `git_version` appends `+dirty` and a `+`-joined string could not
be split back into three fields. `tools/sweep.py` now reuses `uw.recorder.git_version` instead of its own
copy: two implementations could disagree about whether the tree is dirty, and then a dataset's manifest and
its own pings would claim different provenance. `uw.dataset` warns — does not refuse — when one split's
scenarios carry more than one version, reading one ping per scenario.

A guard test (`tests/test_module_boundaries.py`) pins the separation the sonar work depends on: `uw/sonar.py`
and `uw/scattering.py` import no `bubbles` and no `plume`, in either direction, checked by reading the import
statements rather than by importing the modules — `uw/sonar.py` needs warp and torch and cannot be imported
on the host at all, and a static check also catches an import added inside a function, which is where an
accidental dependency shows up first.

### Disclosed: the hold-out was read once, to test the gate

While checking that `compare_real.py --split holdout` works at all, 20 UATD hold-out frames at 10 m were
measured: contrast 1.030, K shape 71.4, KS 0.082, beam width 0.93°. Recorded here rather than left silent,
because a hold-out whose contents have been seen is only still a hold-out if the seeing is on the record.
Nothing was tuned against it -- no sonar parameter has moved at all in this pass -- and the values agree with
the calibrate half (1.095, K 9.3, width 1.12°), which is itself mild evidence that the split is sound. The
gate's three refusals were also checked: a corpus absent from the manifest, a corpus whose hold-out half is
empty because it refused to split, and a second attempt to write `holdout.json`.

### Pass/fail fixed before tuning

`output/compare_real/targets.json`, written in the same pass: which corpus may settle which metric
(`admissibility`), and the criterion per metric. Two are blocking — the flat-sand speckle contrast must stay
at 1.00 ± 0.05, and the stock config must keep reproducing a reference ping bit-identically, so that v0.2 and
the detector trained on it stay comparable. The rest are written as relations against the hold-out's own
measured spread rather than as constants, because for ring-down and level-vs-range we do not know the real
numbers for an M750d at all — only the shape of a Gemini's, which is a different sonar.
