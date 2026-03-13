from sympy import cos, sin, Matrix

def validate_joint_string(s: str) -> bool:
    assert isinstance(s, str), "Joint string must be a string"
    assert len(s) > 0, "Joint string cannot be empty"

    
    i = 0
    while i < len(s):
        if s[i].isdigit():
            while i < len(s) and s[i].isdigit():
                i += 1
            assert i < len(s) and s[i] in ('R', 'P'), \
                f"Number must be followed by R or P at position {i}"
        elif s[i] in ('R', 'P'):
            i += 1
        else:
            assert False, f"Invalid character '{s[i]}' at position {i}"
    
    return True

def process_joint_string(s: str) -> str:
    result = ""
    i = 0
    while i < len(s):
        if s[i].isdigit():
            num = ""
            while i < len(s) and s[i].isdigit():
                num += s[i]
                i += 1
            result += s[i] * int(num)
            i += 1
        else:
            result += s[i]
            i += 1
    return result


def rot_x(theta):
    """Rotation matrix around the X axis."""
    return Matrix([
        [1, 0,          0         ],
        [0, cos(theta), -sin(theta)],
        [0, sin(theta), cos(theta) ]
    ])

def rot_y(theta):
    """Rotation matrix around the Y axis."""
    return Matrix([
        [cos(theta),  0, sin(theta)],
        [0,           1, 0         ],
        [-sin(theta), 0, cos(theta)]
    ])

def rot_z(theta):
    """Rotation matrix around the Z axis."""
    return Matrix([
        [cos(theta), -sin(theta), 0],
        [sin(theta), cos(theta),  0],
        [0,          0,           1]
    ])

def skew(v):
    """
    Returns the skew-symmetric matrix S such that skew(v) * w = v x w.
    Input v can be a list, tuple, or sympy Matrix with 3 elements.
    """
    return Matrix([
        [ 0,    -v[2],  v[1]],
        [ v[2],  0,    -v[0]],
        [-v[1],  v[0],  0   ]
    ])