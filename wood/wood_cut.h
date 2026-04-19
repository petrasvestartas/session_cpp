// ═══════════════════════════════════════════════════════════════════════════
// wood/wood_cut.h — verbatim port of wood's `wood_cut.h`.
//
// The `wood_cut::cut_type` enum labels each polyline emitted by a joint
// constructor as a specific boolean operation against the plate body:
// `hole` cuts a through-pocket, `edge_insertion` trims the edge, `drill`
// punches a vertical shaft, etc. The merge step reads these labels to
// decide what to do with each polyline.
//
// Each joint constructor populates `WoodJoint::m_cut_types[face]` and
// `f_cut_types[face]` with one value per corresponding entry in
// `m_outlines[face]` / `f_outlines[face]`. An empty `*_cut_types` array
// means "all polylines are `edge_insertion`" — the pre-Stage-3 default.
// ═══════════════════════════════════════════════════════════════════════════
#pragma once

namespace wood_cut {
enum cut_type : int {
    nothing                       = 0,

    // Plates (faces, top/bottom + edges)
    hole                          = 1,
    edge_insertion                = 2,
    insert_between_multiple_edges = 3,

    // Beams (always projected or inside volume)
    slice                         = 4,
    slice_projectsheer            = 5,
    mill                          = 6,
    mill_project                  = 7,
    mill_projectsheer             = 8,
    cut                           = 9,
    cut_project                   = 10,
    cut_projectsheer              = 11,
    cut_reverse                   = 12,
    conic                         = 13,
    conic_reverse                 = 14,

    // Vertical drill (plates & beams)
    drill                         = 15,
};
} // namespace wood_cut
