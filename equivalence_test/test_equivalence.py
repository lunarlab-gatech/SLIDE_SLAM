"""SlideSLAM equivalence test: the installed slideslampy vs golden outputs from branch `original`.

Run from the SLIDE_SLAM root: python3 -m unittest discover -s equivalence_test
Discrete outputs must match exactly, floating-point outputs within ATOL. Cases without a golden file are skipped.
Regenerate the self goldens (only after an intended change): python3 equivalence_test/test_equivalence.py --write-self-goldens
"""
import glob
import json
import os
import sys
import unittest

import numpy as np
import slideslampy as ss

from make_fixture_inputs import CASES_DIR, INPUTS_DIR, read_case

GOLDEN_DIR = os.path.join(os.path.dirname(CASES_DIR), 'golden')
# Our own seeded SlideGraph results (the original's CLIPPER u0 is random), so later changes to them are caught
SELF_GOLDEN_DIR = os.path.join(os.path.dirname(CASES_DIR), 'self_golden')
ATOL = 1e-9  # the goldens were built with g++ 9.4 / Eigen 3.3.7, so floats may differ by rounding only

def load_map(name: str) -> np.ndarray:
    return np.loadtxt(os.path.join(INPUTS_DIR, name), ndmin=2)

def load_json(path: str) -> dict | None:
    if not os.path.exists(path):
        return None
    with open(path) as f:
        return json.load(f)

def cases(method: str):
    """(name, reference map, query map, params, golden or None) for every case file of this method."""
    for path in sorted(glob.glob(os.path.join(CASES_DIR, '*.case'))):
        case_method, reference, query, params = read_case(path)
        if case_method != method:
            continue
        name = os.path.splitext(os.path.basename(path))[0]
        golden = load_json(os.path.join(GOLDEN_DIR, f"{name}.json"))
        yield name, load_map(reference), load_map(query), params, golden

def build_place_recognition(params: dict) -> ss.PlaceRecognition:
    slidematch_params, slidegraph_params = ss.SlideMatchParams(), ss.SlideGraphParams()
    for key, value in params['place_recognition'].items():
        if key != 'visualize_matching_results':  # removed with the visualisation code in commit 1
            setattr(slidematch_params, key, value)
    for key, value in params['place_recognition_slidegraph'].items():
        setattr(slidegraph_params, key, value)
    return ss.PlaceRecognition(slidematch_params, slidegraph_params)

def zero_filter(objects: np.ndarray) -> tuple:
    """SlideGraph's skip of objects at exactly x = y = 0 (findInterLoopClosureWithClipper): kept indices, their x/y."""
    kept = [i for i, o in enumerate(objects) if not (o[1] == 0.0 and o[2] == 0.0)]
    return kept, objects[kept, 1:3]

def run_slidegraph(params: dict, reference: np.ndarray, query: np.ndarray) -> dict:
    """Our SlideGraph with a fixed u0 seed, as stored in the self goldens."""
    rng = np.random.RandomState(0)
    result = build_place_recognition(params).slidegraph(reference, query, lambda n: rng.rand(n))
    return {'accepted': result['accepted'], 'selected_pairs': [list(p) for p in result['selected_pairs']],
            'transform': result['transform'].tolist()}

def write_self_goldens() -> None:
    os.makedirs(SELF_GOLDEN_DIR, exist_ok=True)
    for name, reference, query, params, _ in cases('slidegraph'):
        with open(os.path.join(SELF_GOLDEN_DIR, f"{name}.json"), 'w') as f:
            json.dump(run_slidegraph(params, reference, query), f, indent=2)

class TestSlideSLAMEquivalence(unittest.TestCase):

    def test_slidematch(self):
        for name, reference, query, params, golden in cases('slidematch'):
            with self.subTest(case=name):
                if golden is None:
                    self.skipTest(f"{name}: no golden output yet")
                # Compatibility equal to the original's label equality, so the original's goldens still apply
                compatibility = reference[:, 0][:, None] == query[:, 0][None, :]
                result = build_place_recognition(params).slidematch(reference, query, compatibility)
                pairs = [list(p) for p in result['matched_pairs']]

                self.assertEqual(result['accepted'], golden['accepted'])
                if golden['searched']:
                    self.assertEqual(pairs, golden['matched_pairs'])
                    self.assertEqual(len(pairs), golden['num_inliers'])
                else:
                    self.assertEqual(pairs, [])
                if golden['accepted']:
                    np.testing.assert_allclose(result['transform'], golden['transform'], rtol=0, atol=ATOL)

    def test_slidegraph(self):
        for name, reference, query, params, golden in cases('slidegraph'):
            with self.subTest(case=name):
                if golden is None:
                    self.skipTest(f"{name}: no golden output yet")
                slidegraph_params = params['place_recognition_slidegraph']
                reference_kept, reference_points = zero_filter(reference)
                query_kept, query_points = zero_filter(query)
                ran = min(len(reference_kept), len(query_kept)) >= slidegraph_params['min_num_map_objects_to_start']

                self.assertEqual(reference_kept, golden['reference_kept'][0])
                self.assertEqual(query_kept, golden['query_kept'][0])
                self.assertEqual(ran, golden['ran'])
                candidates = []
                if ran:
                    for points, kept, key in [(reference_points, reference_kept, 'triangles_reference'),
                                              (query_points, query_kept, 'triangles_query')]:
                        triangles = [[kept[i] for i in triangle] for triangle in ss.delaunay_triangles(points)]
                        self.assertEqual(triangles, golden[key])
                    matched = ss.match_triangles(reference_points, query_points,
                                                 slidegraph_params['descriptor_matching_threshold'])
                    candidates = [[reference_kept[i], query_kept[j]] for i, j in
                                  zip(matched['candidate_model_indices'], matched['candidate_data_indices'])]
                    self.assertEqual(candidates, golden['candidate_pairs'])
                    np.testing.assert_allclose(matched['triangle_diffs'], golden['triangle_diffs'], rtol=0, atol=ATOL)

                # The original's final CLIPPER selection uses a random u0, so check ours against our own seeded results
                result = run_slidegraph(params, reference, query)
                self.assertTrue(all(p in candidates for p in result['selected_pairs']))
                self_golden = load_json(os.path.join(SELF_GOLDEN_DIR, f"{name}.json"))
                if self_golden is None:
                    self.fail(f"{name}: no self golden; run python3 equivalence_test/test_equivalence.py --write-self-goldens")
                self.assertEqual(result['accepted'], self_golden['accepted'])
                self.assertEqual(result['selected_pairs'], self_golden['selected_pairs'])
                np.testing.assert_allclose(result['transform'], self_golden['transform'], rtol=0, atol=ATOL)

if __name__ == '__main__':
    if sys.argv[1:] == ['--write-self-goldens']:
        write_self_goldens()
    else:
        unittest.main()
