# Native BRep recovery for Claude

This branch preserves local development artifacts from November 6, 2026. Its parent is the historical base commit 0a113afcd4a5ae591e31a91010e7346ae1be2345. The branch source tree represents that base. Later implementation is supplied in the archive and requires reconciliation before integration.

Extract `native-brep-recovery.zip` into a temporary directory. `manifest.json` records every file hash. The archive includes historical alternative patches as recovery material. Apply only reviewed dependency sequences. Some alternatives were rejected.

## Start here

Read `range-foundation-root-promotion-20261106-v1.json` for the applied foundation patch sequence. Those patches depended on earlier working-tree groundwork beyond the base commit. Inspect source snapshots, base hashes and earlier patches before resolving application conflicts. Never assume the full archive is a sequential patch series.

Read `native-range-foundation-tests-20261106-v3/results.json` and its five adjacent test logs. The build passed. BRep passed 68/69, Classification 21/22, Measure 23/23, Section 47/47 and Split 46/46. Remaining failures were Boolean OCCT Box Configurations and Face Boundary Detects Ambiguous Support Association. The older resume-state and promotion status fields predate these completed results.

The foundation added literal original world and primary/secondary pcurve ranges, exact correspondence validation, JSON persistence, boundary extraction, classification and volume consumers. Restricted ranges still require migration in Section, assembly, Split and tessellation. Producer enablement remains pending.

The `same-parameter-original-range-worker-v1`, `curve-intersection-original-range-queue-v1` and `section-original-range-consumers-v1` packets are later external work. Read their manifests and review status. The foundation build did not qualify them.

Do not apply the rejected `split-required-vertex-owned-range` omission approach. The retained compressed-surface counterexample proves that deleting a positive UV boundary inside a small world-space cap corrupts winding.

## Priorities

Native C++ solid Booleans come first. Complete union, intersection, both differences and XOR for platonic and freeform solids, including the Rhino chairs. Offsets and Rust/Python ports follow Boolean acceptance. Preserve original laws and raw parameter identities. Treat unresolved coverage explicitly. Use OCCT source to analyze each failure and retain regressions.

OCCT is an external reference. Obtain it separately from https://github.com/Open-Cascade-SAS/OCCT.git and check out d3056ef80c9668f395da40f5fd7be186cae4501f. Production Session must work without an OCCT dependency.

## Viewer evidence

`native-boolean-viewer-20261106-v1` includes five actual Session Viewer box result PNGs and provenance. Original box reference volume values passed. Strict original-worker acceptance was unavailable because the oracle worker differed. Chair Boolean acceptance and result images remain pending.

## Portability

Historical scripts and receipts contain Windows paths. Adapt launch paths for the cloud environment. Rebuild and rerun tests against the reconciled source revision. The saved results apply to the recorded source and binary hashes.
