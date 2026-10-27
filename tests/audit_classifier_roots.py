"""Independently audit saved bilinear ray hits using 70-digit arithmetic."""

import hashlib
import json
import sys
from decimal import Decimal, localcontext
from fractions import Fraction
from pathlib import Path


def vec(values):
    return [Decimal.from_float(float(value)) for value in values]


def add(a, b):
    return [x + y for x, y in zip(a, b)]


def sub(a, b):
    return [x - y for x, y in zip(a, b)]


def scale(a, factor):
    return [x * factor for x in a]


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def norm(a):
    return dot(a, a).sqrt()


def cross(a, b):
    return [a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]]


def solve(columns, rhs):
    a, b, c = columns
    determinant = dot(a, cross(b, c))
    return [dot(rhs, cross(b, c)) / determinant,
            dot(a, cross(rhs, c)) / determinant,
            dot(a, cross(b, rhs)) / determinant]


def numbers(values):
    return [float(value) for value in values]


def audit(path, ray_index=1, face_index=4):
    data = json.loads(path.read_text())
    surface = data['shape']['surfaces'][data['shape']['faces'][face_index]['surface_index']]
    assert not surface['is_rational']
    assert surface['cv_count_u'] == surface['cv_count_v'] == 2
    assert surface['order_u'] == surface['order_v'] == 2
    poles = [vec(surface['control_points'][i:i + 3]) for i in range(0, 12, 3)]
    origin = vec(data['query'])
    ray = data['rays'][ray_index]
    direction = vec(ray['direction'])
    length = norm(direction)
    base = poles[0]
    x = sub(poles[2], base)
    y = sub(poles[1], base)
    error = sub(sub(sub(poles[3], base), x), y)
    ranges = [vec(surface['nurbsknots_' + axis]) for axis in ('u', 'v')]

    def point(uv):
        return add(add(add(base, scale(x, uv[0])), scale(y, uv[1])),
                   scale(error, uv[0] * uv[1]))

    def residual(p):
        relative = sub(p, origin)
        return norm(sub(relative, scale(direction, dot(relative, direction) / dot(direction, direction))))

    affine = solve([x, y, scale(direction, Decimal(-1))], sub(origin, base))
    root = affine[:]
    for _ in range(8):
        gap = sub(point(root), add(origin, scale(direction, root[2])))
        change = solve([add(x, scale(error, root[1])), add(y, scale(error, root[0])),
                        scale(direction, Decimal(-1))], gap)
        root = sub(root, change)
    root_point = point(root)
    root_uv = [ranges[i][0] + (ranges[i][-1] - ranges[i][0]) * root[i] for i in range(2)]
    normal = cross(add(x, scale(error, root[1])), add(y, scale(error, root[0])))
    exact_poles = [[Fraction.from_float(float(value)) for value in surface['control_points'][i:i + 3]]
                   for i in range(0, 12, 3)]
    exact_x = sub(exact_poles[2], exact_poles[0])
    exact_y = sub(exact_poles[1], exact_poles[0])
    exact_error = sub(sub(sub(exact_poles[3], exact_poles[0]), exact_x), exact_y)
    exact_direction = [Fraction.from_float(float(value)) for value in ray['direction']]
    exact_origin = [Fraction.from_float(float(value)) for value in data['query']]
    exact_matrix = [exact_x, exact_y, scale(exact_direction, -1)]
    exact_affine = solve(exact_matrix, sub(exact_origin, exact_poles[0]))
    coefficient = solve(exact_matrix, exact_error)
    contraction = 2 * max(abs(coefficient[0]), abs(coefficient[1]))
    maps_unit_square = all(0 <= min(exact_affine[i], exact_affine[i] - coefficient[i]) and
                           max(exact_affine[i], exact_affine[i] - coefficient[i]) <= 1 for i in range(2))
    report = {
        'input': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
        'decimal_precision': 70, 'ray_index': ray_index, 'face_index': face_index,
        'tolerance': data['tolerance'], 'query': data['query'], 'direction': ray['direction'],
        'bilinear_mixed_coefficient': numbers(error),
        'bilinear_mixed_coefficient_norm': float(norm(error)),
        'affine_root_normalized_uv': numbers(affine[:2]),
        'affine_root_ray_distance': float(affine[2] * length),
        'original_bilinear_root_uv': numbers(root_uv),
        'original_bilinear_root_point': numbers(root_point),
        'original_bilinear_root_ray_distance': float(root[2] * length),
        'original_bilinear_root_residual': float(residual(root_point)),
        'abs_normal_dot_unit_ray': float(abs(dot(normal, direction)) / (norm(normal) * length)),
        'exact_rational_uv_fixed_point_contraction_bound': float(contraction),
        'exact_rational_contraction_less_than_one': contraction < 1,
        'exact_rational_fixed_point_maps_unit_square_into_itself': maps_unit_square,
        'hits': [],
    }
    previous = None
    for hit in ray['hits']:
        if hit['face'] != face_index:
            continue
        uv = vec(hit['uv'])
        normalized = [(uv[i] - ranges[i][0]) / (ranges[i][-1] - ranges[i][0]) for i in range(2)]
        actual = point(normalized)
        saved = vec(hit['point'])
        ray_parameter = dot(sub(actual, origin), direction) / dot(direction, direction)
        gap = sub(actual, add(origin, scale(direction, ray_parameter)))
        dx = add(x, scale(error, normalized[1]))
        dy = add(y, scale(error, normalized[0]))
        step = solve([dx, dy, scale(direction, Decimal(-1))], gap)
        world_step = add(scale(dx, step[0]), scale(dy, step[1]))
        row = {
            'saved_distance': hit['distance'], 'saved_uv': hit['uv'],
            'original_support_transverse_residual': float(residual(actual)),
            'saved_point_transverse_residual': float(residual(saved)),
            'saved_point_original_support_gap': float(norm(sub(saved, actual))),
            'original_support_point_to_true_root': float(norm(sub(actual, root_point))),
            'uv_gap_to_true_root': numbers(sub(uv, root_uv)),
            'ray_distance_error': float(Decimal.from_float(hit['distance']) - root[2] * length),
            'first_newton_world_correction_norm': float(norm(world_step)),
            'first_newton_ray_correction_norm': float(abs(step[2]) * length),
            'first_newton_bilinear_remainder_norm': float(norm(scale(error, step[0] * step[1]))),
        }
        if previous is not None:
            row['original_support_gap_from_previous'] = float(norm(sub(actual, previous)))
        previous = actual
        report['hits'].append(row)
    return report


if __name__ == '__main__':
    with localcontext() as context:
        context.prec = 70
        result = audit(Path(sys.argv[1]))
    text = json.dumps(result, indent=2)
    Path(sys.argv[2]).write_text(text + '\n')
    print(text)
