# Copyright 2026 The Vikings. Licensed under the Apache License, Version 2.0.
"""Tests for the perception logic that does not need a model.

Three modules, one theme: every case here is one where a wrong answer looks
fine. A mis-tiered class, a confidently-reported dead detector, a depth
speckle read as an obstacle at the bumper -- none of them produce a visible
error. They produce a robot that behaves plausibly and is wrong.

No ROS, no ultralytics, no GPU.

    python test/test_perception.py
    pytest test/test_perception.py
"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drishti_perception import health as H            # noqa: E402
from drishti_perception.obstacle import (             # noqa: E402
    Detection, detection_distance, is_hazard, lethal_detections, nearest_obstacle)
from drishti_perception.taxonomy import (             # noqa: E402
    CLASS_TIER, TIER_POLICY, ClassId, Tier, all_classes, clearance_m,
    decayed_cost, decayed_lethal, from_detector_label, is_lethal, name_of,
    policy_of, semantic_cost, tier_of)

_checks = 0
_failures = 0
_case = ""


def case(name):
    global _case
    _case = name


def CHECK(cond, note=""):
    global _checks, _failures
    import inspect
    _checks += 1
    if not cond:
        _failures += 1
        print("  FAIL  [%s] line %d %s"
              % (_case, inspect.currentframe().f_back.f_lineno, note))


def CLOSE(a, b, tol=1e-12, note=""):
    global _checks, _failures
    import inspect
    _checks += 1
    if not abs(float(a) - float(b)) <= tol:
        _failures += 1
        print("  FAIL  [%s] line %d %s got %.12g want %.12g"
              % (_case, inspect.currentframe().f_back.f_lineno, note, a, b))


# =========================================================== taxonomy
def test_every_class_has_a_tier():
    case("every declared class has a tier")
    for cid in all_classes():
        CHECK(cid in CLASS_TIER, "%s unmapped" % cid.name)
    for tier in Tier:
        CHECK(tier in TIER_POLICY, "%s has no policy" % tier.name)


def test_class_ids_are_frozen():
    case("class ids are frozen")
    # These values live in recorded bags and in mono8 masks. Changing one
    # silently reinterprets every bag ever recorded, so they are pinned here
    # deliberately: this test failing means someone renumbered, and the fix is
    # to append a new id rather than edit an old one.
    expected = {
        "UNKNOWN": 0, "DIRT": 10, "ROAD": 11, "GRASS": 12,
        "GRAVEL": 20, "UNEVEN_GRASS": 21, "ROUGH_GROUND": 22,
        "MUD": 30, "WATER": 31, "DEEP_VEGETATION": 32, "STEEP_SLOPE": 33,
        "DITCH": 40, "CLIFF_EDGE": 41, "ROCK": 42, "TREE_TRUNK": 43, "WALL": 44,
        "PERSON": 50, "VEHICLE": 51, "ANIMAL": 52,
    }
    for name, value in expected.items():
        CHECK(int(ClassId[name]) == value, "%s moved" % name)
    CHECK(len(expected) == len(list(all_classes())), "a class was added or removed")


def test_class_ids_fit_in_a_mono8_mask():
    case("class ids fit in a mono8 mask")
    for cid in all_classes():
        CHECK(0 <= int(cid) <= 255, "%s out of byte range" % cid.name)


def test_an_unrecognised_class_is_never_traversable():
    case("an unrecognised class is never traversable")
    # The safety property of this module. A model emitting a class outside the
    # vocabulary, or a corrupted mask byte, must not decay to "road".
    for bogus in (7, 99, 255, -1, 1000, 13, 45):
        CHECK(tier_of(bogus) is Tier.UNKNOWN, "id %d" % bogus)
        CHECK(semantic_cost(bogus) > 0.5, "id %d was cheap" % bogus)
        CHECK(not is_lethal(bogus), "id %d should not be lethal" % bogus)


def test_unknown_cost_matches_the_traversability_default():
    case("unknown cost agrees with traversability.yaml")
    # Both encode SPEC.md 6.2. If they drift, an unclassified pixel and an
    # unobserved cell would be priced differently for no reason.
    CLOSE(semantic_cost(ClassId.UNKNOWN), 0.85, 1e-12)


def test_tiers_are_ordered_by_consequence():
    case("tier costs increase with consequence")
    order = [Tier.TRAVERSABLE, Tier.CAUTION, Tier.HIGH_COST, Tier.LETHAL]
    costs = [TIER_POLICY[t].cost for t in order]
    for a, b in zip(costs, costs[1:]):
        CHECK(b > a, "costs not increasing: %s" % costs)
    CLOSE(TIER_POLICY[Tier.TRAVERSABLE].cost, 0.0)
    CLOSE(TIER_POLICY[Tier.LETHAL].cost, 1.0)


def test_only_lethal_and_dynamic_saturate():
    case("only lethal and dynamic tiers saturate a cell")
    for cid in all_classes():
        expected = tier_of(cid) in (Tier.LETHAL, Tier.DYNAMIC)
        CHECK(is_lethal(cid) is expected, name_of(cid))


def test_only_dynamic_classes_get_clearance():
    case("only dynamic classes get a clearance margin")
    for cid in all_classes():
        margin = clearance_m(cid)
        if tier_of(cid) is Tier.DYNAMIC:
            CHECK(margin > 0.0, name_of(cid))
        else:
            CLOSE(margin, 0.0, 0.0, name_of(cid))


def test_static_hazards_never_decay():
    case("static hazards never decay")
    # A ditch seen once is still a ditch an hour later.
    for cid in (ClassId.DITCH, ClassId.CLIFF_EDGE, ClassId.WALL, ClassId.MUD):
        CLOSE(decayed_cost(cid, 3600.0), semantic_cost(cid), 1e-12, name_of(cid))
        if is_lethal(cid):
            CHECK(decayed_lethal(cid, 3600.0), name_of(cid))


def test_dynamic_hazards_decay_fast():
    case("dynamic hazards decay fast")
    # A pedestrian who walked out of frame must not become a permanent phantom.
    half_life = TIER_POLICY[Tier.DYNAMIC].half_life_s
    CLOSE(decayed_cost(ClassId.PERSON, 0.0), 1.0, 1e-12)
    CLOSE(decayed_cost(ClassId.PERSON, half_life), 0.5, 1e-12)
    CLOSE(decayed_cost(ClassId.PERSON, 2 * half_life), 0.25, 1e-12)
    CHECK(decayed_lethal(ClassId.PERSON, 0.0))
    CHECK(not decayed_lethal(ClassId.PERSON, 3 * half_life),
          "should have stopped blocking")
    CHECK(decayed_cost(ClassId.PERSON, 10.0) < 0.01)


def test_negative_age_does_not_inflate_cost():
    case("a negative age does not inflate cost")
    CLOSE(decayed_cost(ClassId.PERSON, -5.0), 1.0, 1e-12)


def test_detector_labels_map_or_become_unknown():
    case("detector labels map, or become UNKNOWN")
    CHECK(from_detector_label("person") is ClassId.PERSON)
    CHECK(from_detector_label("TRUCK") is ClassId.VEHICLE)
    CHECK(from_detector_label("  Dog  ") is ClassId.ANIMAL)
    # An unmapped label is still something the model saw; dropping it would
    # turn a real object into empty space.
    CHECK(from_detector_label("toaster") is ClassId.UNKNOWN)
    CHECK(from_detector_label("") is ClassId.UNKNOWN)
    CHECK(from_detector_label(None) is ClassId.UNKNOWN)


# ============================================================= health
def base_stats(**kw):
    stats = H.FrameStats(now=100.0, last_rgb_stamp=99.99,
                         last_depth_stamp=99.99, pipeline_ok=True,
                         latency_ms=25.0)
    for k, v in kw.items():
        setattr(stats, k, v)
    return stats


def test_a_dead_pipeline_reports_zero_confidence():
    case("a dead pipeline reports zero confidence")
    # The failure this module exists to prevent: a detector that silently died
    # returns an empty list forever. It must not read as "nothing to worry
    # about".
    h = H.compute(base_stats(pipeline_ok=False))
    CLOSE(h.mean_confidence, 0.0)
    CHECK("did not run" in h.confidence_basis)


def test_an_empty_frame_is_not_treated_as_danger():
    case("an empty frame from a healthy pipeline is not danger")
    # Open safe terrain is the common case. Pinning the supervisor to SLOW here
    # would make the safety mechanism something people switch off.
    h = H.compute(base_stats(detection_confidences=()))
    CLOSE(h.mean_confidence, 0.80)
    CHECK("nothing detected" in h.confidence_basis)
    CHECK(h.detection_count == 0)


def test_detection_confidence_is_the_mean_not_the_max():
    case("detection confidence is the mean, not the max")
    # One certain detection does not mean the rest of the frame is understood.
    h = H.compute(base_stats(detection_confidences=(0.9, 0.3, 0.3)))
    CLOSE(h.mean_confidence, 0.5, 1e-12)
    CHECK(h.detection_count == 3)


def test_segmentation_outranks_detections():
    case("segmentation confidence outranks detections")
    h = H.compute(base_stats(detection_confidences=(0.95,),
                             segmentation_confidence=0.4,
                             segmentation_coverage=0.9))
    CLOSE(h.mean_confidence, 0.4, 1e-12)
    CHECK(h.confidence_basis == "segmentation")


def test_a_sliver_of_mask_does_not_earn_full_confidence():
    case("thin segmentation coverage is scaled down")
    # A confident opinion about 3% of the image says almost nothing about the
    # frame.
    h = H.compute(base_stats(segmentation_confidence=0.95,
                             segmentation_coverage=0.03))
    CLOSE(h.mean_confidence, 0.95 * 0.03, 1e-12)
    CHECK("coverage" in h.confidence_basis)


def test_non_finite_confidences_are_discarded_not_trusted():
    case("non-finite confidences are discarded")
    h = H.compute(base_stats(detection_confidences=(float("nan"), 0.6,
                                                    float("inf"))))
    CLOSE(h.mean_confidence, 0.6, 1e-12)
    CHECK(h.detection_count == 1)


def test_confidences_outside_zero_one_are_clamped():
    case("confidences outside [0, 1] are clamped")
    h = H.compute(base_stats(detection_confidences=(5.0, -2.0)))
    CLOSE(h.mean_confidence, 0.5, 1e-12)


def test_stamp_age_matches_the_supervisor_rules():
    case("stamp_age matches the supervisor's rules")
    CHECK(H.stamp_age(100.0, None) >= H.NEVER_SEEN_AGE)
    CLOSE(H.stamp_age(100.0, 99.5), 0.5, 1e-12)
    CLOSE(H.stamp_age(100.0, 100.01), 0.0, 1e-12)      # small jitter
    CHECK(H.stamp_age(100.0, 130.0) >= H.NEVER_SEEN_AGE)  # clocks disagree
    CHECK(H.stamp_age(float("nan"), 99.0) >= H.NEVER_SEEN_AGE)


def test_never_seen_streams_are_not_ok():
    case("a stream that never arrived is not ok")
    h = H.compute(H.FrameStats(now=100.0, pipeline_ok=True))
    CHECK(not h.rgb_ok)
    CHECK(not h.depth_ok)
    CHECK(h.rgb_age >= H.NEVER_SEEN_AGE)


def test_staleness_thresholds_are_respected():
    case("staleness thresholds are respected on both sides")
    fresh = H.compute(base_stats(last_rgb_stamp=99.8), t_camera_stale=0.30)
    CHECK(fresh.rgb_ok)
    stale = H.compute(base_stats(last_rgb_stamp=99.6), t_camera_stale=0.30)
    CHECK(not stale.rgb_ok)


def test_a_changing_camera_never_looks_frozen():
    case("a changing camera never looks frozen")
    tr = H.FrameChangeTracker()
    for i in range(50):
        static = tr.update("frame-%d" % i, i * 0.1)
        CHECK(static == 0.0, "changed content must reset the clock")


def test_a_frozen_camera_accumulates_static_time():
    case("a frozen camera accumulates static time")
    # D18/D19: this is the signal t_camera_stale cannot produce. The stamps
    # keep advancing; only the content stands still.
    tr = H.FrameChangeTracker()
    tr.update("same", 0.0)
    CLOSE(tr.update("same", 1.0), 1.0)
    CLOSE(tr.update("same", 2.5), 2.5)
    CLOSE(tr.update("same", 10.0), 10.0)
    # One genuinely new frame clears it.
    CLOSE(tr.update("different", 10.1), 0.0)
    CLOSE(tr.update("different", 11.1), 1.0)


def test_the_first_frame_is_not_immediately_static():
    case("the first frame is not immediately static")
    tr = H.FrameChangeTracker()
    CLOSE(tr.update("first", 5.0), 0.0)


def test_a_missing_signature_does_not_clear_a_freeze():
    case("a missing signature does not clear a freeze")
    # If the signature path breaks, it must not look like fresh content --
    # that would let a bug mask the very failure this detects.
    tr = H.FrameChangeTracker()
    tr.update("same", 0.0)
    tr.update("same", 2.0)
    CLOSE(tr.update(None, 3.0), 3.0, 1e-12, "still frozen")


def test_a_non_finite_time_does_not_corrupt_the_tracker():
    case("a non-finite time does not corrupt the tracker")
    tr = H.FrameChangeTracker()
    tr.update("same", 0.0)
    tr.update("same", 2.0)
    tr.update("same", float("nan"))
    CLOSE(tr.update("same", 3.0), 3.0)


def test_reset_clears_the_tracker():
    case("reset clears the tracker")
    tr = H.FrameChangeTracker()
    tr.update("same", 0.0)
    tr.update("same", 5.0)
    tr.reset()
    CLOSE(tr.update("same", 6.0), 0.0)


def test_static_duration_reaches_the_health_report():
    case("static duration reaches the health report")
    h = H.compute(base_stats(rgb_static_for=4.2))
    CLOSE(h.rgb_static_for, 4.2)
    # A frozen camera still looks perfectly live on the age fields; that is the
    # whole point of carrying a separate signal.
    CHECK(h.rgb_ok, "a frozen camera reports fresh")


def test_a_missing_static_duration_defaults_to_zero():
    case("a missing static duration defaults to zero")
    # Absence must not stop the vehicle by itself: silence is already covered
    # by the health message going stale.
    CLOSE(H.compute(base_stats()).rgb_static_for, 0.0)
    CLOSE(H.compute(base_stats(rgb_static_for=float("nan"))).rgb_static_for, 0.0)


# =========================================================== obstacle
def depth_image(fill=5.0, shape=(120, 160)):
    return np.full(shape, float(fill))


def box(class_id, conf=0.9, x0=60, y0=40, x1=100, y1=80):
    return Detection(class_id=int(class_id), confidence=conf,
                     x0=x0, y0=y0, x1=x1, y1=y1)


def test_only_lethal_and_dynamic_count_as_hazards():
    case("only lethal and dynamic classes are hazards")
    CHECK(is_hazard(ClassId.ROCK))
    CHECK(is_hazard(ClassId.PERSON))
    CHECK(not is_hazard(ClassId.MUD))      # costly terrain, not a thing to hit
    CHECK(not is_hazard(ClassId.ROAD))
    CHECK(not is_hazard(ClassId.UNKNOWN))


def test_distance_is_read_from_inside_the_box():
    case("distance is read from inside the box")
    depth = depth_image(5.0)
    depth[40:80, 60:100] = 2.5
    CLOSE(nearest_obstacle(depth, [box(ClassId.ROCK)]), 2.5, 1e-9)


def test_a_speckle_does_not_trigger_an_emergency_stop():
    case("a depth speckle does not trigger an emergency stop")
    # THE failure this module exists to prevent. A handful of bad pixels at
    # 0.05 m inside a clean 6 m box would, under a raw minimum, stop the
    # vehicle every frame -- and the fix everyone reaches for is to raise
    # d_emergency, which disables the stop for real obstacles too.
    depth = depth_image(6.0)
    depth[40:80, 60:100] = 6.0
    # The speckle must sit INSIDE the valid range, or the min_range_m filter
    # removes it and this test passes without ever exercising the percentile.
    # (It did exactly that until a deliberate regression exposed it: replacing
    # the percentile with a raw minimum still passed.) 0.30 m is a plausible
    # bad reading, well above min_range_m and well inside d_emergency.
    #
    # 20 speckled pixels in a 40x40 box is 1.25%, comfortably under the 10th
    # percentile, so a correct implementation ignores them.
    rng = np.random.default_rng(4)
    ys = rng.integers(40, 80, size=20)
    xs = rng.integers(60, 100, size=20)
    depth[ys, xs] = 0.30

    d = nearest_obstacle(depth, [box(ClassId.ROCK)])
    CHECK(d > 5.0, "speckle leaked through: %.3f" % d)
    # And the speckle really is present and valid, so the check above is about
    # the percentile rather than about the range filter.
    CHECK(float(np.min(depth[40:80, 60:100])) == 0.30, "speckle was filtered out")


def test_a_real_surface_is_still_detected():
    case("a real near surface is still detected")
    # The other half: rejecting speckle must not blind us to a genuine
    # obstacle, which occupies many pixels.
    depth = depth_image(6.0)
    depth[40:80, 60:100] = 0.7
    CLOSE(nearest_obstacle(depth, [box(ClassId.ROCK)]), 0.7, 1e-9)


def test_zero_and_negative_depth_are_no_reading_not_contact():
    case("zero depth is 'no reading', never 'touching the bumper'")
    # The most dangerous single misreading available: 0.0 means the sensor has
    # nothing, not that the obstacle is at the lens.
    depth = depth_image(4.0)
    depth[40:80, 60:100] = 0.0
    d = nearest_obstacle(depth, [box(ClassId.ROCK)])
    CHECK(math.isnan(d), "got %r" % d)

    depth[40:80, 60:100] = -1.0
    CHECK(math.isnan(nearest_obstacle(depth, [box(ClassId.ROCK)])))


def test_non_finite_depth_is_ignored():
    case("non-finite depth pixels are ignored")
    depth = depth_image(3.0)
    depth[40:80, 60:100] = np.nan
    depth[50:60, 70:80] = 1.5
    CLOSE(nearest_obstacle(depth, [box(ClassId.ROCK)]), 1.5, 1e-9)


def test_too_few_valid_pixels_reports_not_measured():
    case("too few valid pixels reports NaN, not a guess")
    depth = depth_image(np.nan)
    depth[40, 60] = 1.0
    depth[41, 61] = 1.0
    CHECK(math.isnan(nearest_obstacle(depth, [box(ClassId.ROCK)])))


def test_no_hazard_returns_nan_meaning_nothing_in_range():
    case("no hazard returns NaN meaning nothing in range")
    # drishti_safety reads a non-finite distance as absence, which is sound
    # only because it rules out stale depth first. The two packages agree on
    # this convention.
    depth = depth_image(2.0)
    CHECK(math.isnan(nearest_obstacle(depth, [])))
    CHECK(math.isnan(nearest_obstacle(depth, [box(ClassId.ROAD)])))
    CHECK(math.isnan(nearest_obstacle(depth, [box(ClassId.MUD)])))


def test_the_closest_hazard_wins():
    case("the closest of several hazards wins")
    depth = depth_image(9.0)
    depth[10:30, 10:30] = 4.0
    depth[40:80, 60:100] = 1.8
    depth[90:110, 120:150] = 7.0
    dets = [box(ClassId.ROCK, x0=10, y0=10, x1=30, y1=30),
            box(ClassId.PERSON, x0=60, y0=40, x1=100, y1=80),
            box(ClassId.WALL, x0=120, y0=90, x1=150, y1=110)]
    CLOSE(nearest_obstacle(depth, dets), 1.8, 1e-9)


def test_low_confidence_detections_do_not_hard_stop():
    case("low-confidence detections do not trigger a hard stop")
    # They still raise semantic cost and drag mean_confidence down, so the
    # vehicle slows. What they must not do is slam on the brakes for a maybe.
    depth = depth_image(6.0)
    depth[40:80, 60:100] = 0.5
    CHECK(math.isnan(nearest_obstacle(depth, [box(ClassId.ROCK, conf=0.05)])))
    CLOSE(nearest_obstacle(depth, [box(ClassId.ROCK, conf=0.9)]), 0.5, 1e-9)


def test_boxes_are_clipped_to_the_image():
    case("boxes outside the image are clipped, not crashed")
    depth = depth_image(3.0)
    partly_out = Detection(int(ClassId.ROCK), 0.9, x0=140, y0=100, x1=400, y1=400)
    CLOSE(nearest_obstacle(depth, [partly_out]), 3.0, 1e-9)

    fully_out = Detection(int(ClassId.ROCK), 0.9, x0=500, y0=500, x1=600, y1=600)
    CHECK(math.isnan(nearest_obstacle(depth, [fully_out])))

    inverted = Detection(int(ClassId.ROCK), 0.9, x0=100, y0=80, x1=60, y1=40)
    CHECK(math.isnan(nearest_obstacle(depth, [inverted])))


def test_out_of_range_depth_is_discarded():
    case("depth outside the sensor range is discarded")
    depth = depth_image(500.0)          # beyond max_range_m
    depth[50:70, 70:90] = 3.0
    CLOSE(nearest_obstacle(depth, [box(ClassId.ROCK)]), 3.0, 1e-9)


def test_lethal_detections_filter():
    case("lethal_detections filters by class and confidence")
    dets = [box(ClassId.ROCK, conf=0.9), box(ClassId.PERSON, conf=0.9),
            box(ClassId.MUD, conf=0.9), box(ClassId.ROCK, conf=0.01)]
    got = lethal_detections(dets)
    CHECK(len(got) == 2, "got %d" % len(got))


def test_detection_distance_rejects_a_non_2d_depth():
    case("detection_distance rejects a non-2D depth image")
    try:
        detection_distance(np.zeros((4, 4, 3)), box(ClassId.ROCK))
        CHECK(False, "should have raised")
    except ValueError:
        CHECK(True)


def main():
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
    print("\n%d checks, %d failures across %d tests"
          % (_checks, _failures, len(tests)))
    if _failures == 0:
        print("perception: OK")
    return 1 if _failures else 0


if __name__ == "__main__":
    sys.exit(main())
