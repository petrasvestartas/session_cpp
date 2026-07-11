# boolean_steps -- STEP export of every boolean regression result

Every file is written by `main_7.exe` from the boolean test suites and verified to
import in OCCT's strict STEP reader (`validation/step_probe`) as SOLIDS 1 / VALID 1
(files marked EMPTY are genuinely empty results and import as 0 solids).

Regenerate with (from `session_cpp/`):

    SESSION_STEP_DIR=serialization/boolean_steps ./build/Release/main_7.exe            # 45-cell matrix
    SESSION_EDGE=1 SESSION_STEP_DIR=serialization/boolean_steps ./build/Release/main_7.exe   # edge-case battery
    SESSION_FREEFORM=1 SESSION_STEP_DIR=serialization/boolean_steps ./build/Release/main_7.exe "zzzz"  # freeform
    SESSION_STEP_PRIMS=serialization/boolean_steps ./build/Release/main_7.exe "zzzz"   # 5 primitives

What you should see when opening each file:

- **box_common_box2.step** -- intersection of A = 4x4x4 box centered at origin and B = 2x2x2 box at (2,0,0); expect 1 solid(s), 6 faces, volume 4.0000
- **box_common_cone.step** -- intersection of A = 4x4x4 box centered at origin and B = r=2 h=4 cone (base z=-2, apex up); expect 1 solid(s), 2 faces, volume 16.7420
- **box_common_cyl.step** -- intersection of A = 4x4x4 box centered at origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 3 faces, volume 28.2596
- **box_common_sph.step** -- intersection of A = 4x4x4 box centered at origin and B = r=2.5 sphere at origin; expect 1 solid(s), 7 faces, volume 54.4607
- **box_common_tor.step** -- intersection of A = 4x4x4 box centered at origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 5 faces, volume 15.4827
- **box_cut_box2.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = 2x2x2 box at (2,0,0); expect 1 solid(s), 11 faces, volume 60.0000
- **box_cut_cone.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = r=2 h=4 cone (base z=-2, apex up); expect 1 solid(s), 10 faces, volume 47.2448
- **box_cut_cyl.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 7 faces, volume 35.7404
- **box_cut_sph.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = r=2.5 sphere at origin; expect 1 solid(s), 7 faces, volume 9.5379
- **box_cut_tor.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 7 faces, volume 48.5173
- **box_fuse_box2.step** -- union of A = 4x4x4 box centered at origin and B = 2x2x2 box at (2,0,0); expect 1 solid(s), 11 faces, volume 68.0000
- **box_fuse_cone.step** -- union of A = 4x4x4 box centered at origin and B = r=2 h=4 cone (base z=-2, apex up); expect 1 solid(s), 10 faces, volume 64.0000
- **box_fuse_cyl.step** -- union of A = 4x4x4 box centered at origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 10 faces, volume 78.1519
- **box_fuse_sph.step** -- union of A = 4x4x4 box centered at origin and B = r=2.5 sphere at origin; expect 1 solid(s), 13 faces, volume 74.9878
- **box_fuse_tor.step** -- union of A = 4x4x4 box centered at origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 16 faces, volume 73.7835
- **cone_common_cone2.step** -- intersection of A = r=2 h=4 cone (base z=-2, apex up) and B = r=2 h=4 flipped cone (apex down); expect 1 solid(s), 2 faces, volume 4.1888
- **cone_common_cyl.step** -- intersection of A = r=2 h=4 cone (base z=-2, apex up) and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 3 faces, volume 14.1321
- **cone_common_tor.step** -- intersection of A = r=2 h=4 cone (base z=-2, apex up) and B = R=2 r=0.8 torus in the xy-plane; expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **cone_cut_cone2.step** -- difference (A minus B) of A = r=2 h=4 cone (base z=-2, apex up) and B = r=2 h=4 flipped cone (apex down); expect 1 solid(s), 3 faces, volume 12.5664
- **cone_cut_cyl.step** -- difference (A minus B) of A = r=2 h=4 cone (base z=-2, apex up) and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 3 faces, volume 2.6194
- **cone_cut_tor.step** -- difference (A minus B) of A = r=2 h=4 cone (base z=-2, apex up) and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 2 faces, volume 16.7552
- **cone_fuse_cone2.step** -- union of A = r=2 h=4 cone (base z=-2, apex up) and B = r=2 h=4 flipped cone (apex down); expect 1 solid(s), 4 faces, volume 29.3215
- **cone_fuse_cyl.step** -- union of A = r=2 h=4 cone (base z=-2, apex up) and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 6 faces, volume 45.0331
- **cone_fuse_tor.step** -- union of A = r=2 h=4 cone (base z=-2, apex up) and B = R=2 r=0.8 torus in the xy-plane; expect 2 solid(s), 3 faces, volume 42.0213
- **cyl_common_cyl2.step** -- intersection of A = r=1.5 h=6 vertical cylinder through the origin and B = r=1.5 h=6 horizontal cylinder (rotated 90 deg about y); expect 1 solid(s), 6 faces, volume 17.9983
- **cyl_common_tor.step** -- intersection of A = r=1.5 h=6 vertical cylinder through the origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 2 faces, volume 2.2593
- **cyl_cut_cyl2.step** -- difference (A minus B) of A = r=1.5 h=6 vertical cylinder through the origin and B = r=1.5 h=6 horizontal cylinder (rotated 90 deg about y); expect 2 solid(s), 7 faces, volume 24.4134
- **cyl_cut_tor.step** -- difference (A minus B) of A = r=1.5 h=6 vertical cylinder through the origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 5 faces, volume 40.1522
- **cyl_fuse_cyl2.step** -- union of A = r=1.5 h=6 vertical cylinder through the origin and B = r=1.5 h=6 horizontal cylinder (rotated 90 deg about y); expect 1 solid(s), 8 faces, volume 66.8248
- **cyl_fuse_tor.step** -- union of A = r=1.5 h=6 vertical cylinder through the origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 6 faces, volume 65.4184
- **ebox_common_ebox.step** -- intersection of A = 4x4x4 box centered at origin and B = 4x4x4 box centered at origin; expect 1 solid(s), 6 faces, volume 64.0000
- **ebox_common_eboxC.step** -- intersection of A = 4x4x4 box centered at origin and B = identical box at (2,2,0) (overlap with coplanar z-faces); expect 1 solid(s), 6 faces, volume 16.0000
- **ebox_common_eboxD.step** -- intersection of A = 4x4x4 box centered at origin and B = 2x2x2 box at (6,0,0), fully disjoint; expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ebox_common_eboxE.step** -- intersection of A = 4x4x4 box centered at origin and B = identical box at (4,4,0) (touching along one edge); expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ebox_common_eboxIN.step** -- intersection of A = 4x4x4 box centered at origin and B = 2x2x2 box strictly inside the big box; expect 1 solid(s), 6 faces, volume 8.0000
- **ebox_common_eboxP.step** -- intersection of A = 4x4x4 box centered at origin and B = 2x2x2 box at (3,0,0) (partial-face contact on x=2); expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ebox_common_eboxT.step** -- intersection of A = 4x4x4 box centered at origin and B = identical box shifted +4x (full-face contact at x=2); expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ebox_common_eboxV.step** -- intersection of A = 4x4x4 box centered at origin and B = identical box at (4,4,4) (touching at one corner); expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ebox_common_ecylF.step** -- intersection of A = 4x4x4 box centered at origin and B = r=1.5 h=4 cylinder with caps flush with the box z-faces; expect 1 solid(s), 3 faces, volume 28.2596
- **ebox_common_esphZ.step** -- intersection of A = 4x4x4 box centered at origin and B = r=2 sphere inscribed in the box (tangent to all 6 walls); expect 1 solid(s), 1 faces, volume 33.5103
- **ebox_cut_ebox.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = 4x4x4 box centered at origin; expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ebox_cut_eboxC.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = identical box at (2,2,0) (overlap with coplanar z-faces); expect 1 solid(s), 8 faces, volume 48.0000
- **ebox_cut_eboxD.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = 2x2x2 box at (6,0,0), fully disjoint; expect 1 solid(s), 6 faces, volume 64.0000
- **ebox_cut_eboxE.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = identical box at (4,4,0) (touching along one edge); expect 1 solid(s), 6 faces, volume 64.0000
- **ebox_cut_eboxIN.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = 2x2x2 box strictly inside the big box; expect 2 solid(s), 12 faces, volume 72.0000
- **ebox_cut_eboxP.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = 2x2x2 box at (3,0,0) (partial-face contact on x=2); expect 1 solid(s), 7 faces, volume 64.0000
- **ebox_cut_eboxT.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = identical box shifted +4x (full-face contact at x=2); expect 1 solid(s), 6 faces, volume 64.0000
- **ebox_cut_eboxV.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = identical box at (4,4,4) (touching at one corner); expect 1 solid(s), 6 faces, volume 64.0000
- **ebox_cut_ecylF.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = r=1.5 h=4 cylinder with caps flush with the box z-faces; expect 1 solid(s), 7 faces, volume 35.7404
- **ebox_cut_esphZ.step** -- difference (A minus B) of A = 4x4x4 box centered at origin and B = r=2 sphere inscribed in the box (tangent to all 6 walls); expect 2 solid(s), 7 faces, volume 97.5103
- **ebox_fuse_ebox.step** -- union of A = 4x4x4 box centered at origin and B = 4x4x4 box centered at origin; expect 1 solid(s), 6 faces, volume 64.0000
- **ebox_fuse_eboxC.step** -- union of A = 4x4x4 box centered at origin and B = identical box at (2,2,0) (overlap with coplanar z-faces); expect 1 solid(s), 14 faces, volume 112.0000
- **ebox_fuse_eboxD.step** -- union of A = 4x4x4 box centered at origin and B = 2x2x2 box at (6,0,0), fully disjoint; expect 2 solid(s), 12 faces, volume 72.0000
- **ebox_fuse_eboxE.step** -- union of A = 4x4x4 box centered at origin and B = identical box at (4,4,0) (touching along one edge); expect 2 solid(s), 12 faces, volume 128.0000
- **ebox_fuse_eboxIN.step** -- union of A = 4x4x4 box centered at origin and B = 2x2x2 box strictly inside the big box; expect 1 solid(s), 6 faces, volume 64.0000
- **ebox_fuse_eboxP.step** -- union of A = 4x4x4 box centered at origin and B = 2x2x2 box at (3,0,0) (partial-face contact on x=2); expect 1 solid(s), 11 faces, volume 72.0000
- **ebox_fuse_eboxT.step** -- union of A = 4x4x4 box centered at origin and B = identical box shifted +4x (full-face contact at x=2); expect 1 solid(s), 10 faces, volume 128.0000
- **ebox_fuse_eboxV.step** -- union of A = 4x4x4 box centered at origin and B = identical box at (4,4,4) (touching at one corner); expect 2 solid(s), 12 faces, volume 128.0000
- **ebox_fuse_ecylF.step** -- union of A = 4x4x4 box centered at origin and B = r=1.5 h=4 cylinder with caps flush with the box z-faces; expect 1 solid(s), 8 faces, volume 64.0000
- **ebox_fuse_esphZ.step** -- union of A = 4x4x4 box centered at origin and B = r=2 sphere inscribed in the box (tangent to all 6 walls); expect 1 solid(s), 6 faces, volume 64.0000
- **ecyl_common_ecyl.step** -- intersection of A = r=1.5 h=6 vertical cylinder through the origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 3 faces, volume 42.4115
- **ecyl_common_ecylP.step** -- intersection of A = r=1.5 h=6 vertical cylinder through the origin and B = identical cylinder at x=3 (line tangency along x=1.5); expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ecyl_cut_ecyl.step** -- difference (A minus B) of A = r=1.5 h=6 vertical cylinder through the origin and B = r=1.5 h=6 vertical cylinder through the origin; expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **ecyl_cut_ecylP.step** -- difference (A minus B) of A = r=1.5 h=6 vertical cylinder through the origin and B = identical cylinder at x=3 (line tangency along x=1.5); expect 1 solid(s), 3 faces, volume 42.4115
- **ecyl_fuse_ecyl.step** -- union of A = r=1.5 h=6 vertical cylinder through the origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 3 faces, volume 42.4115
- **ecyl_fuse_ecylP.step** -- union of A = r=1.5 h=6 vertical cylinder through the origin and B = identical cylinder at x=3 (line tangency along x=1.5); expect 2 solid(s), 7 faces, volume 84.8230
- **esph_common_esph.step** -- intersection of A = r=2.5 sphere at origin and B = r=2.5 sphere at origin; expect 1 solid(s), 1 faces, volume 65.4498
- **esph_common_esphX.step** -- intersection of A = r=2.5 sphere at origin and B = r=1 sphere at (3.5,0,0), externally tangent at (2.5,0,0); expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **esph_common_esphY.step** -- intersection of A = r=2.5 sphere at origin and B = r=1 sphere at (1.5,0,0), internally tangent at (2.5,0,0); expect 1 solid(s), 1 faces, volume 4.1888
- **esph_cut_esph.step** -- difference (A minus B) of A = r=2.5 sphere at origin and B = r=2.5 sphere at origin; expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **esph_cut_esphX.step** -- difference (A minus B) of A = r=2.5 sphere at origin and B = r=1 sphere at (3.5,0,0), externally tangent at (2.5,0,0); expect 1 solid(s), 1 faces, volume 65.4498
- **esph_cut_esphY.step** -- difference (A minus B) of A = r=2.5 sphere at origin and B = r=1 sphere at (1.5,0,0), internally tangent at (2.5,0,0); expect 2 solid(s), 2 faces, volume 69.6386
- **esph_fuse_esph.step** -- union of A = r=2.5 sphere at origin and B = r=2.5 sphere at origin; expect 1 solid(s), 1 faces, volume 65.4498
- **esph_fuse_esphX.step** -- union of A = r=2.5 sphere at origin and B = r=1 sphere at (3.5,0,0), externally tangent at (2.5,0,0); expect 2 solid(s), 2 faces, volume 69.6386
- **esph_fuse_esphY.step** -- union of A = r=2.5 sphere at origin and B = r=1 sphere at (1.5,0,0), internally tangent at (2.5,0,0); expect 1 solid(s), 1 faces, volume 65.4498
- **etor_common_ecylT.step** -- intersection of A = R=2 r=0.8 torus in the xy-plane and B = r=0.8 cylinder threading the torus hole without touching; expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **etor_common_etor.step** -- intersection of A = R=2 r=0.8 torus in the xy-plane and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 1 faces, volume 25.2662
- **etor_cut_ecylT.step** -- difference (A minus B) of A = R=2 r=0.8 torus in the xy-plane and B = r=0.8 cylinder threading the torus hole without touching; expect 1 solid(s), 1 faces, volume 25.2662
- **etor_cut_etor.step** -- difference (A minus B) of A = R=2 r=0.8 torus in the xy-plane and B = R=2 r=0.8 torus in the xy-plane; expect an EMPTY file (this boolean result is genuinely empty) -- imports as 0 solids by design
- **etor_fuse_ecylT.step** -- union of A = R=2 r=0.8 torus in the xy-plane and B = r=0.8 cylinder threading the torus hole without touching; expect 2 solid(s), 4 faces, volume 37.3299
- **etor_fuse_etor.step** -- union of A = R=2 r=0.8 torus in the xy-plane and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 1 faces, volume 25.2662
- **freeform_blob.step** -- freeform: perturbed-sphere NURBS blob (interior CVs +-12%), 1 face, volume 64.1292; expect 1 solid(s), 1 faces, volume 64.1292
- **freeform_common_box.step** -- freeform boolean: box intersect blob, volume 53.55; expect 1 solid(s), 7 faces, volume 53.5505
- **freeform_cut_box.step** -- freeform boolean: 4x4x4 box minus the blob, volume 10.45; expect 1 solid(s), 7 faces, volume 10.4495
- **freeform_fuse_box.step** -- freeform boolean: box union blob, volume 74.58; expect 1 solid(s), 13 faces, volume 74.5787
- **freeform_pillow.step** -- freeform: box with 6 bulged bicubic faces (pillow), volume 41.8720; expect 1 solid(s), 6 faces, volume 41.8720
- **prim_box.step** -- primitive: 4x4x4 box, 6 planar faces, volume 64; expect 1 solid(s), 6 faces, volume 64.0000
- **prim_cone.step** -- primitive: r=2 h=4 cone, 2 faces, volume 16.7552; expect 1 solid(s), 2 faces, volume 16.7552
- **prim_cylinder.step** -- primitive: r=1.5 h=6 cylinder, 3 faces, volume 42.4115; expect 1 solid(s), 3 faces, volume 42.4115
- **prim_sphere.step** -- primitive: r=2.5 sphere, 1 face, volume 65.4498; expect 1 solid(s), 1 faces, volume 65.4498
- **prim_torus.step** -- primitive: R=2 r=0.8 torus, 1 face, volume 25.2662; expect 1 solid(s), 1 faces, volume 25.2662
- **sph_common_cone.step** -- intersection of A = r=2.5 sphere at origin and B = r=2 h=4 cone (base z=-2, apex up); expect 1 solid(s), 3 faces, volume 15.9175
- **sph_common_cyl.step** -- intersection of A = r=2.5 sphere at origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 3 faces, volume 31.9395
- **sph_common_sph2.step** -- intersection of A = r=2.5 sphere at origin and B = r=2 sphere at (2,0,0); expect 1 solid(s), 3 faces, volume 17.3836
- **sph_common_tor.step** -- intersection of A = r=2.5 sphere at origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 2 faces, volume 20.3701
- **sph_cut_cone.step** -- difference (A minus B) of A = r=2.5 sphere at origin and B = r=2 h=4 cone (base z=-2, apex up); expect 2 solid(s), 4 faces, volume 49.5323
- **sph_cut_cyl.step** -- difference (A minus B) of A = r=2.5 sphere at origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 2 faces, volume 33.5103
- **sph_cut_sph2.step** -- difference (A minus B) of A = r=2.5 sphere at origin and B = r=2 sphere at (2,0,0); expect 1 solid(s), 2 faces, volume 48.0671
- **sph_cut_tor.step** -- difference (A minus B) of A = r=2.5 sphere at origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 3 faces, volume 45.0798
- **sph_fuse_cone.step** -- union of A = r=2.5 sphere at origin and B = r=2 h=4 cone (base z=-2, apex up); expect 1 solid(s), 4 faces, volume 66.2875
- **sph_fuse_cyl.step** -- union of A = r=2.5 sphere at origin and B = r=1.5 h=6 vertical cylinder through the origin; expect 1 solid(s), 5 faces, volume 75.9218
- **sph_fuse_sph2.step** -- union of A = r=2.5 sphere at origin and B = r=2 sphere at (2,0,0); expect 1 solid(s), 2 faces, volume 81.5770
- **sph_fuse_tor.step** -- union of A = r=2.5 sphere at origin and B = R=2 r=0.8 torus in the xy-plane; expect 1 solid(s), 4 faces, volume 70.3459
- **tor_common_tor2.step** -- intersection of A = R=2 r=0.8 torus in the xy-plane and B = same torus shifted to (2,0,0); expect 2 solid(s), 12 faces, volume 6.5364
- **tor_cut_tor2.step** -- difference (A minus B) of A = R=2 r=0.8 torus in the xy-plane and B = same torus shifted to (2,0,0); expect 2 solid(s), 9 faces, volume 18.7297
- **tor_fuse_tor2.step** -- union of A = R=2 r=0.8 torus in the xy-plane and B = same torus shifted to (2,0,0); expect 1 solid(s), 6 faces, volume 43.9959
