import numpy as np
import slideslampy as ss
import time
import unittest


def place_recognition(budget_sec: float, step: float) -> ss.PlaceRecognition:
    """ SlideMatch with the given budget and xy step, every other gate off, so only the budget limits the search. """
    params = ss.SlideMatchParams()
    params.compute_budget_sec = budget_sec
    params.search_xy_step_size = step
    params.search_yaw_step_size_degrees = 5.0
    params.match_threshold_position = 0.5
    params.ignore_dimension = True
    params.min_num_inliers = 0
    params.min_num_map_objects_to_start = 1
    return ss.PlaceRecognition(params, ss.SlideGraphParams())


def random_map(n: int, extent: float, seed: int) -> np.ndarray:
    """ n objects (label x y z d1 d2 d3) spread uniformly over an extent x extent square. """
    rng = np.random.default_rng(seed)
    objects = np.zeros((n, 7))
    objects[:, 1:3] = rng.uniform(-extent / 2, extent / 2, (n, 2))
    objects[:, 4:7] = 1.0
    return objects


def run(pr: ss.PlaceRecognition, objects: np.ndarray) -> tuple[dict, float]:
    """ SlideMatch of a map against itself (every pair compatible); returns the result and its wall time. """
    compatibility = np.ones((len(objects), len(objects)), dtype=bool)
    start = time.perf_counter()
    result = pr.slidematch(objects, objects, compatibility)
    return result, time.perf_counter() - start


class TestSlideMatchBudget(unittest.TestCase):
    """
    SlideMatch's compute budget at sub-second resolution, checked before every search position (fork commit 4).
    Run from the SLIDE_SLAM root: python3 -m unittest discover -s equivalence_test. T3/T4 depend on timing; their
    margins assume a lightly loaded machine (one position costs a few ms here, a whole first ring well over 0.5 s).
    """

    # Large input: 200 objects over 30 m, 0.1 m step, so one ring (400 positions) takes far longer than the budgets
    LARGE: np.ndarray = random_map(200, 30.0, seed=0)
    RING_POSITIONS: int = 400

    def test_complete_search_unchanged_by_budget(self):
        # T1: when the budget never binds, the search is complete and identical
        objects = random_map(20, 10.0, seed=1)
        result_a, _ = run(place_recognition(1e9, 0.5), objects)
        result_b, _ = run(place_recognition(1e6, 0.5), objects)
        self.assertGreater(result_a['positions_searched'], self.RING_POSITIONS)
        self.assertEqual(result_a['positions_searched'], result_b['positions_searched'])
        self.assertEqual([list(p) for p in result_a['matched_pairs']], [list(p) for p in result_b['matched_pairs']])

    def test_zero_budget_searches_nothing(self):
        # T2: the check comes before the first position, so nothing is evaluated (the original would search ring 0)
        result, _ = run(place_recognition(0.0, 0.5), random_map(20, 10.0, seed=1))
        self.assertEqual(result['positions_searched'], 0)
        self.assertEqual(list(result['matched_pairs']), [])
        self.assertFalse(result['accepted'])

    def test_small_budget_stops_mid_ring_on_time(self):
        # T3: precondition, the first ring alone takes longer than 0.5 s on this input
        long_result, _ = run(place_recognition(0.5, 0.1), self.LARGE)
        self.assertLess(long_result['positions_searched'], self.RING_POSITIONS)

        # With 0.05 s, the search stops within a position's cost of the budget, part-way through the first ring
        result, elapsed = run(place_recognition(0.05, 0.1), self.LARGE)
        self.assertLess(elapsed, 0.05 + 0.03)
        self.assertGreater(result['positions_searched'], 0)
        self.assertLess(result['positions_searched'], long_result['positions_searched'])

    def test_more_budget_searches_further(self):
        # T4: the same search order is followed further with more budget, and repeated calls don't carry state over
        searched = [run(place_recognition(budget, 0.1), self.LARGE)[0]['positions_searched'] for budget in (0.02, 0.05, 0.2)]
        self.assertLessEqual(searched[0], searched[1])
        self.assertLessEqual(searched[1], searched[2])
        self.assertLess(searched[0], searched[2])
        self.assertGreater(searched[0], 0)


if __name__ == '__main__':
    unittest.main()
