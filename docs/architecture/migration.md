# Migration: from the contracts on paper to the contracts in the show

A proposed order — **for you to decide**; none of it is started. Each step is
separately mergeable, leaves the FR20 show behaving as today, and names how
that is verified. Steps 1–7 make the code arm- and camera-neutral; 8 and 9 are
the ports themselves ([porting.md](../porting.md)).

| # | Step | Touches runtime? | Risk | Verified by |
|---|---|---|---|---|
| 0 | **This change**: contracts, reference implementations, conformance, docs, ADRs | no — new files only | none | `python scripts/contracts/conformance.py` |
| 1 | Run the contract suites and every `--self-test` as one check command (and CI if wanted) | no | low | — |
| 2 | Split trajectory math out of `fairino_player.py`, re-exported ([ADR 006](../adr/006-split-trajectory-math-from-fairino-driver.md)) | yes (moved code) | low | player / motion_clip self-tests; rebuild `party`, diff with the tracked compiled show |
| 3 | Open the arm through a `BackendFactory` in `show_stream`, `track_mode`, `accel_probe`; FR20 facts (`SIM_NET`, fault codes, timeouts) into `BackendInfo` | **yes, the live stream path** | medium | stream self-test; a long SimMachine run with the report compared to a previous one (lag, skips, worst step); one `--rehearse` on the arm |
| 4 | Thread the profile id through the Python side (`"fr20"` → the show's `robot`, default `fr20`): `load_rig`, `load_model`, `RP.load`, `_tcp_path`, then drop the defaults | yes (111 sites, generators) | medium | rebuild `party`: compiled show identical (`rebuild_library.py`); `coupling_audit.py` count falls to the tests |
| 5 | Motion guards: the compiled show records its `robot`; the stream refuses another arm's; FR20 joint values of the show config into a per-arm overlay | yes (build, stream start) | low–medium | build diff; a test that a mismatched compiled show is refused |
| 6 | Directors live: ask during the running clip (the Runner knows `time_left`), `pick()` only reads a ready answer or falls back; off by default (`--director`); decision log beside the stream log | **yes, the tick thread** | medium–high | `BaselineDirector` live = `Selector` live (same history, tick timing unchanged) on SimMachine; then an example Director in Isaac; then the arm |
| 7 | Perception through `PerceptionSource` in `track_mode` / Isaac `run_tracking`; retire or alias `/track/target`; extrinsic file with its RMS and date | yes (tracking input) | low–medium | `track_eval.py` scenarios unchanged; a Femto recording replayed |
| 8 | UF850 (or xArm): porting.md steps 0–10 | new code + the arm | **high** (reach and payload may mean a different show; IK) | conformance on UFactory Studio, then the build, then the arm at 0.3 |
| 9 | OAK-D producer (DepthAI → SceneState OSC, its calibration) | new process | medium | `perception_suite`; `track_eval` on its recordings |

Dependencies: 3 needs nothing; 4 before 5 and 8; 6 needs only 0 (and is
independent of the arm); 7 before 9. 2 and 3 can go together if wanted.

Why this order: the cheap, behaviour-preserving moves first (2, 4) give the
port its foundations without touching how the arm is driven; the stream (3, 6)
is changed only when there is a reason and a measurement to compare against;
the expensive physical work (8) comes last because step 0 of porting.md — does
the show even fit a 0.85 m arm — may change what is worth porting.
