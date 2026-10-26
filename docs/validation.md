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
