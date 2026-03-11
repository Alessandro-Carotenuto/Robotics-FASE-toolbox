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