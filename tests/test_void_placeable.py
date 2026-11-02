"""void_placeable / generate_macros Close vs Extend order."""

from nest_graph.cohort_specs import generate_macros
from nest_graph.free_kind_util import void_phantom, void_placeable
from nest_graph.graph import FreeKind, MacroRegion


def test_void_placeable_swiss_and_large_void():
    assert void_placeable(FreeKind.LargeVoid, 1.0)
    assert void_placeable(FreeKind.SwissCheese, 1.0)
    assert not void_placeable(FreeKind.SwissCheese, 0.0)
    assert not void_placeable(FreeKind.Full, 1.0)
    assert void_phantom(FreeKind.LargeVoid, 0.0)
    assert not void_phantom(FreeKind.SwissCheese, 0.0)


def test_generate_macros_swiss_placeable_void_first():
    actions = generate_macros(
        [0, 1],
        rule_ids=(0,),
        free_kind=FreeKind.SwissCheese,
        void_fit_r=1.0,
    )
    regions = [a.region for a in actions if a.motif_id < 0]
    assert MacroRegion.Void in regions
    assert regions.index(MacroRegion.Void) < regions.index(MacroRegion.Rim)


def test_generate_macros_swiss_unplaceable_rim_first():
    actions = generate_macros(
        [0, 1],
        rule_ids=(0,),
        free_kind=FreeKind.SwissCheese,
        void_fit_r=0.0,
    )
    regions = [a.region for a in actions if a.motif_id < 0]
    assert regions.index(MacroRegion.Rim) < regions.index(MacroRegion.Void)
    assert regions.index(MacroRegion.Void) < regions.index(MacroRegion.Sheet)


def test_generate_macros_phantom_large_void_stays_void_first():
    actions = generate_macros(
        [0, 1],
        rule_ids=(0,),
        free_kind=FreeKind.LargeVoid,
        void_fit_r=0.0,
    )
    regions = [a.region for a in actions if a.motif_id < 0]
    assert regions.index(MacroRegion.Void) < regions.index(MacroRegion.Rim)
