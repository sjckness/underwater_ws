# Data request to DFKI: Oculus M1200d ROS bags from the Maritime Exploration Hall

**Status:** draft, not sent (2026-11-04). Contact to send from: fabio.carapellese@seamorphrobotics.com.
**Why it exists:** `docs/validation.md` establishes that the published DeeperSense image datasets cannot
settle the near-range behaviour, the TVG or the receiver-noise level of an Oculus, because a Cartesian fan
image destroys exactly those quantities. Raw polar beams can. These four requests are the smallest set that
would settle them, ordered by value per gigabyte; the first two are 0.85 GB together.

---

## Draft message

Subject: Request for Oculus M1200d sonar bags (Maritime Exploration Hall) for simulator validation

Dear DeeperSense team,

We are developing an open simulation model of a forward-looking multibeam sonar (Blueprint Subsea Oculus
M750d) for underwater gas-leak detection, and we are validating it against real recordings rather than
against our own expectations. Your Zenodo record 10373154 was invaluable for working out what to ask for: we
indexed the metadatabase and identified 89 Oculus M1200d streams at the Maritime Exploration Hall
(`location_key` `dfki_basin`), 11.58 h over the 2022-02 and 2023-04 sessions.

We have already made what use we can of the published image datasets (records 7728089 and 10220989, both
CC BY 4.0, gratefully acknowledged). They let us confirm that our background speckle statistics match reality
closely — real basin bottom reads a Rayleigh contrast of 1.02 against our 0.98. But we also ran a control in
which real polar data was rendered to a Cartesian fan image and read back, and it showed that the apex of a
fan image is where its sampling is worst: speckle statistics and beam width survive, while the near-range
profile and the TVG fit do not — the ring-down slope changes sign. Those are precisely the quantities we
still cannot validate, and `modality.modality_description` notes that the published image datafiles are a
Cartesian projection, so only the bags carry raw beams.

We would therefore like to ask for the following, in order of usefulness to us. We would be glad to receive
only the first two if that is easier — together they are about 0.85 GB.

1. `2023-04_dfki_basin/2023-04-27-10-13_test_high_data_res_low_freq/` — 0.12 GB, 28 s, 283 pings, 16-bit,
   `freqMode` 1, 8 m range.
2. `2023-04_dfki_basin/2023-04-27-10-15_test_high_data_res_high_freq/` — 0.73 GB, 116 s, 992 pings, 16-bit,
   `freqMode` 2, 8 m range.

   These are the only two 16-bit lab-tank recordings in the corpus. `gain_assistance` is enabled, so the
   level drifts between pings, but they would still settle the beam pattern, the ring-down and the speckle,
   which is what we need most.

3. `2022-02_dfki_basin/2022-02-07-15-39_pipe_mounting_5/` and `2022-02-07-15-52_pipe_mounting_6/` — 2.69 GB
   each, 600 s. 8-bit, but `send_gain` is true and `gain_assistance` false, so the applied per-row TVG is
   recoverable and can be divided out. The scene — a pipe being mounted — is also the closest in the corpus
   to the one we simulate.
4. `2022-02-07-16-07_diver_moving_around_1` — 2.81 GB, `freqMode` 1, 7 m, `send_gain` true, for the
   low-frequency mode.

What we would use them for, concretely: the shape of the near-range return (ring-down amplitude and decay,
and where usable data actually begins); the level-vs-range profile with the applied gain divided out; and the
level of a broadside stripe we measured in basin-dataset-2 (+3.4 dB reading a pixel as an amplitude, up to
+10 dB at a fitted pixel law). We suspect receiver noise common to all elements, which sums coherently in the
broadside beam; we have implemented it as an option but left it disabled, because the published images show
that such a stripe exists without constraining its level. Raw beams would let us read it off directly.

We are happy to work within whatever terms suit you — including a restricted or non-redistributable
arrangement — and we do not need to republish the bags themselves. We would acknowledge DFKI and the
DeeperSense project in any resulting publication and would gladly share our validation results, which may be
useful as an independent characterisation of the sensor in that facility.

Two things we should be upfront about: our head is an M750d (750 kHz / 1.2 MHz) and yours an M1200d
(1.2 / 2.1 MHz), so only the high-frequency end overlaps and we will treat everything we derive as
indicative rather than as a calibration of our own sensor. And nothing in the corpus appears to contain a
target of known target strength, so we are not expecting to fix an absolute level from it.

Thank you for making the metadata public — it is what made a specific request possible instead of a vague one.

With best regards,

Fabio Carapellese
Seamorph Robotics

---

## Notes for us, not for the message

- Sizes and settings are from `tools/deepersense_index.py` over the record-10373154 CSV export; the full
  per-stream table is `docs/deepersense_tank_recordings.md`.
- Settings across the 95 configs: `freqMode` high 74 / low 21; `range` mostly 8 m; `gain` 10–80 %, mostly
  30 %; `gamma` 127 throughout; `data_16bit` **false in 92 of 95**; `send_gain` true in 9;
  `gain_assistance` true in 45.
- What each request unblocks, in plan terms: 1–2 make the ring-down and near-range measurement an Oculus
  measurement instead of a Gemini shape; 3 makes the TVG recoverable; 1–2 settle
  `sonar.receiver_noise_common_db`. None of them settles `array_taper_sll_db` — that needs a point target of
  known strength, which this corpus does not have.
- Until they arrive, steps 3–4 of the calibration plan stay shape-only and tagged UNVERIFIED for the M750d,
  and defaults do not move.
