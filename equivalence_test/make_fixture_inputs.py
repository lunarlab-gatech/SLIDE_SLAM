"""Generate the input maps and case files for the SlideSLAM equivalence test (test_equivalence.py).

Run once from the SLIDE_SLAM root: python3 equivalence_test/make_fixture_inputs.py
Outputs go to equivalence_test/fixtures/{inputs,cases}/ and are committed; the golden outputs in
equivalence_test/fixtures/golden/ are produced from these by golden_driver/ run against branch `original`.
"""
import os
import numpy as np

ROOT = os.path.dirname(os.path.abspath(__file__))
EXAMPLE_DATA = os.path.join(os.path.dirname(ROOT), 'backend/sloam/clipper_semantic_object/examples/data')
INPUTS_DIR = os.path.join(ROOT, 'fixtures/inputs')
CASES_DIR = os.path.join(ROOT, 'fixtures/cases')

# Map file format, one object per line: label x y z dim1 dim2 dim3

def convert_example_map(src_name: str, dst_name: str) -> None:
    """Copy one of SlideSLAM's example maps (label x y z) verbatim, appending zero dimensions."""
    with open(os.path.join(EXAMPLE_DATA, src_name)) as f_in, open(os.path.join(INPUTS_DIR, dst_name), 'w') as f_out:
        for line in f_in:
            tokens = line.split()
            assert len(tokens) == 4, f"{src_name}: expected 'label x y z', got {line!r}"
            f_out.write(' '.join(tokens) + ' 0 0 0\n')

def random_objects(rng: np.random.RandomState, n: int, extent: float, with_dims: bool) -> np.ndarray:
    labels = rng.randint(0, 5, n)
    xy = rng.uniform(-extent / 2, extent / 2, (n, 2))
    z = rng.uniform(0.0, 2.0, n)
    dims = -np.sort(-rng.uniform(0.3, 2.0, (n, 3)), axis=1) if with_dims else np.zeros((n, 3))
    return np.column_stack([labels, xy, z, dims])

def synthetic_pair(seed: int, n_ref: int, n_overlap: int, n_extra: int, extent: float, yaw_deg: float,
                   t_xyz: tuple, noise: float, with_dims: bool, zero_object: bool) -> tuple:
    """Reference map plus a query map seeing n_overlap of its objects, with ref = R(yaw) @ query + t."""
    rng = np.random.RandomState(seed)
    ref = random_objects(rng, n_ref, extent, with_dims)
    if zero_object:
        ref[0, 1:3] = 0.0
    yaw = np.deg2rad(yaw_deg)
    R = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
    seen = ref[rng.choice(n_ref, n_overlap, replace=False)].copy()
    seen[:, 1:3] = (seen[:, 1:3] - np.array(t_xyz[:2])) @ R + rng.normal(0, noise, (n_overlap, 2))
    seen[:, 3] += -t_xyz[2] + rng.normal(0, noise, n_overlap)
    if with_dims:
        seen[:, 4:7] = np.clip(seen[:, 4:7] + rng.normal(0, 0.1, (n_overlap, 3)), 0.05, None)
    extra = random_objects(rng, n_extra, extent, with_dims)
    query = np.vstack([seen, extra])[rng.permutation(n_overlap + n_extra)]
    if zero_object:
        query[0, 1:3] = 0.0
    return ref, query

def write_map(objects: np.ndarray, dst_name: str) -> None:
    xy = [tuple(row) for row in np.round(objects[:, 1:3], 6)]
    assert len(set(xy)) == len(xy), f"{dst_name}: duplicate x/y (golden_driver maps outputs to indices by x/y)"
    with open(os.path.join(INPUTS_DIR, dst_name), 'w') as f:
        for row in objects:
            f.write(f"{int(row[0])} " + ' '.join(f"{v:.6f}" for v in row[1:]) + '\n')

# Values follow backend/sloam/params/sloam.yaml except where noted per case
SLIDEMATCH_BASE = {
    ('double', 'compute_budget_sec'): 1e9,  # unbounded, so the search always completes and results don't depend on timing
    ('double', 'dilation_factor'): 1.2,
    ('double', 'search_xy_step_size'): 0.1,
    ('double', 'match_yaw_half_range'): 180.0,
    ('bool', 'disable_yaw_search'): False,
    ('double', 'search_yaw_step_size_degrees'): 15.0,
    ('double', 'match_threshold_position'): 0.75,
    ('double', 'match_threshold_dimension'): 1.0,
    ('bool', 'ignore_dimension'): True,
    ('int', 'min_num_inliers'): 8,
    ('bool', 'use_nonlinear_least_squares'): True,
    ('int', 'min_num_map_objects_to_start'): 5,
    ('bool', 'visualize_matching_results'): False,
}
SLIDEGRAPH_BASE = {
    ('int', 'num_inliners_threshold'): 5,
    ('double', 'descriptor_matching_threshold'): 0.1,
    ('double', 'sigma'): 0.1,
    ('double', 'epsilon'): 0.3,
    ('int', 'min_num_map_objects_to_start'): 30,
}

def format_param(ptype: str, value) -> str:
    if ptype == 'bool':
        return str(value).lower()
    return repr(float(value)) if ptype == 'double' else str(int(value))

def write_case(name: str, method: str, reference: str, query: str, slidematch_overrides: dict | None = None) -> None:
    """One case: method, map files, and every ROS parameter both methods read, written out explicitly."""
    params = [('place_recognition', {**SLIDEMATCH_BASE, **(slidematch_overrides or {})}),
              ('place_recognition_slidegraph', SLIDEGRAPH_BASE)]
    with open(os.path.join(CASES_DIR, f"{name}.case"), 'w') as f:
        f.write(f"method {method}\nreference {reference}\nquery {query}\n")
        for namespace, values in params:
            for (ptype, key), value in values.items():
                f.write(f"param {ptype} {namespace}/{key} {format_param(ptype, value)}\n")

def read_case(path: str) -> tuple:
    """Inverse of write_case: (method, reference map name, query map name, {namespace: {key: value}})."""
    fields, params = {}, {}
    parse = {'double': float, 'int': int, 'bool': lambda v: {'true': True, 'false': False}[v]}
    with open(path) as f:
        for line in f:
            tokens = line.split()
            if not tokens:
                continue
            if tokens[0] == 'param':
                _, ptype, name, value = tokens
                namespace, key = name.split('/')
                params.setdefault(namespace, {})[key] = parse[ptype](value)
            elif tokens[0] in ('method', 'reference', 'query'):
                fields[tokens[0]] = tokens[1]
            else:
                raise ValueError(f"{path}: unknown line {line!r}")
    return fields['method'], fields['reference'], fields['query'], params

def main() -> None:
    os.makedirs(INPUTS_DIR, exist_ok=True)
    os.makedirs(CASES_DIR, exist_ok=True)

    for env in ['indoor', 'parking', 'forest']:
        for robot in range(3):
            convert_example_map(f"robot{robot}Map_{env}.txt", f"{env}_r{robot}.txt")

    for name, pair in [
        ('synth_dims', synthetic_pair(seed=1, n_ref=25, n_overlap=15, n_extra=3, extent=20.0, yaw_deg=40.0,
                                      t_xyz=(3.0, -2.0, 0.3), noise=0.05, with_dims=True, zero_object=False)),
        ('synth_zero', synthetic_pair(seed=2, n_ref=40, n_overlap=28, n_extra=6, extent=30.0, yaw_deg=-70.0,
                                      t_xyz=(5.0, 4.0, 0.0), noise=0.02, with_dims=False, zero_object=True)),
    ]:
        write_map(pair[0], f"{name}_ref.txt")
        write_map(pair[1], f"{name}_query.txt")

    step = lambda s: {('double', 'search_xy_step_size'): s}
    dims_on = {('bool', 'ignore_dimension'): False}

    # SlideMatch on real indoor maps: sloam.yaml's step for r0-r1, coarser steps elsewhere to bound runtime
    write_case('sm_indoor_r0_r1', 'slidematch', 'indoor_r0.txt', 'indoor_r1.txt')
    write_case('sm_indoor_r0_r2', 'slidematch', 'indoor_r0.txt', 'indoor_r2.txt', step(0.25))
    write_case('sm_indoor_r1_r2', 'slidematch', 'indoor_r1.txt', 'indoor_r2.txt', step(0.25))
    # Large-environment counts with the dimension check on (dims are zero, so its cylinder branch is taken)
    write_case('sm_parking_r0_r1', 'slidematch', 'parking_r0.txt', 'parking_r1.txt',
               {**step(2.0), **dims_on, ('double', 'match_threshold_position'): 1.0,
                ('int', 'min_num_inliers'): 15, ('int', 'min_num_map_objects_to_start'): 15})
    # Synthetic with real dimensions: check on/off, LSQ off, rejection, and too few objects to start
    write_case('sm_synth_dims_on', 'slidematch', 'synth_dims_ref.txt', 'synth_dims_query.txt', {**step(0.25), **dims_on})
    write_case('sm_synth_dims_off', 'slidematch', 'synth_dims_ref.txt', 'synth_dims_query.txt', step(0.25))
    # Tight enough that the dimension check rejects some true matches (size noise is ~0.1 m)
    write_case('sm_synth_dims_tight', 'slidematch', 'synth_dims_ref.txt', 'synth_dims_query.txt',
               {**step(0.25), **dims_on, ('double', 'match_threshold_dimension'): 0.1})
    write_case('sm_synth_no_lsq', 'slidematch', 'synth_dims_ref.txt', 'synth_dims_query.txt',
               {**step(0.25), **dims_on, ('bool', 'use_nonlinear_least_squares'): False})
    write_case('sm_synth_rejected', 'slidematch', 'synth_dims_ref.txt', 'synth_dims_query.txt',
               {**step(0.25), ('int', 'min_num_inliers'): 1000})
    write_case('sm_synth_too_few', 'slidematch', 'synth_dims_ref.txt', 'synth_dims_query.txt',
               {**step(0.25), ('int', 'min_num_map_objects_to_start'): 100})

    # SlideGraph: sloam.yaml values throughout
    write_case('sg_indoor_r0_r1', 'slidegraph', 'indoor_r0.txt', 'indoor_r1.txt')
    write_case('sg_indoor_r0_r2', 'slidegraph', 'indoor_r0.txt', 'indoor_r2.txt')  # r2 has 29 < 30 objects
    write_case('sg_parking_r0_r1', 'slidegraph', 'parking_r0.txt', 'parking_r1.txt')
    write_case('sg_parking_r0_r2', 'slidegraph', 'parking_r0.txt', 'parking_r2.txt')
    write_case('sg_forest_r0_r1', 'slidegraph', 'forest_r0.txt', 'forest_r1.txt')
    write_case('sg_synth_zero', 'slidegraph', 'synth_zero_ref.txt', 'synth_zero_query.txt')

if __name__ == '__main__':
    main()
