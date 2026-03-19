import re

CODES = '''
    // 战壕 (Trench) - 高度 35.00" (0.89m)
    new AprilTagCoordinate(1,  11.86,  7.41,  0.89,  -1,  0,  0, 1.41), // 180°
    new AprilTagCoordinate(6,  11.86,  0.63,  0.89,  -1,  0,  0, 1.41), // 180°
    new AprilTagCoordinate(7,  11.94,  0.63,  0.89,   1,  0,  0, 1.41), // 0°
    new AprilTagCoordinate(12, 11.94,  7.41,  0.89,   1,  0,  0, 1.41), // 0°

    // 中枢 (Hub) - 高度 44.25" (1.12m)
    new AprilTagCoordinate(2,  11.90,  4.62,  1.12,   0,  1,  0, 1.54), // 90°
    new AprilTagCoordinate(3,  11.30,  4.38,  1.12,  -1,  0,  0, 1.54), // 180°
    new AprilTagCoordinate(4,  11.30,  4.02,  1.12,  -1,  0,  0, 1.54), // 180°
    new AprilTagCoordinate(5,  11.90,  3.42,  1.12,   0, -1,  0, 1.54), // 270°
    new AprilTagCoordinate(8,  12.26,  3.42,  1.12,   0, -1,  0, 1.54), // 270°
    new AprilTagCoordinate(9,  12.51,  3.67,  1.12,   1,  0,  0, 1.54), // 0°
    new AprilTagCoordinate(10, 12.51,  4.02,  1.12,   1,  0,  0, 1.54), // 0°
    new AprilTagCoordinate(11, 12.26,  4.62,  1.12,   0,  1,  0, 1.54), // 90°

    // 哨站 (Outpost) - 高度 21.75" (0.55m)
    new AprilTagCoordinate(13, 16.50,  7.39,  0.55,  -1,  0,  0, 0.93), // 180°
    new AprilTagCoordinate(14, 16.50,  6.96,  0.55,  -1,  0,  0, 0.93), // 180°

    // 塔墙 (Tower Wall) - 高度 21.75" (0.55m)
    new AprilTagCoordinate(15, 16.50,  4.31,  0.55,  -1,  0,  0, 0.93), // 180°
    new AprilTagCoordinate(16, 16.50,  3.88,  0.55,  -1,  0,  0,0.93),  // 180°

    new AprilTagCoordinate(17,  4.649,  0.63,  0.89,  1,  0,  0, 1.41),
    new AprilTagCoordinate(18,  4.61,  3.42,  1.12,  0, -1,   0, 1.54),
    new AprilTagCoordinate(19,  5.22,  3.67,  1.12,  1,  0,  0, 1.54),
    new AprilTagCoordinate(20,  5.22,  4.02,  1.12,  1,  0,  0, 1.54),
    new AprilTagCoordinate(21,  4.61,  4.62,  1.12, 0,  1,  0, 1.54),
    new AprilTagCoordinate(22,  4.649,  7.41,  0.89,  1,  0,  0, 1.41),
    new AprilTagCoordinate(23,  4.57,  7.41,  0.89,  -1,  0,  0,1.41),
    new AprilTagCoordinate(24,  4.25,  4.62,  1.12,  0,  1,  0, 1.54),
    new AprilTagCoordinate(25,  4,  4.37,  1.12,  -1,  0,  0,1.54),
    new AprilTagCoordinate(26,  4,  4.02,  1.12,  -1,  0,  0,1.54),
    new AprilTagCoordinate(27,  4.25,  3.42,  1.12,  0,  -1,  0,1.54),
    new AprilTagCoordinate(28,  4.57,  0.63,  0.89,  -1,  0,  0,1.41),
    new AprilTagCoordinate(29,  0.0127,  0.65,  0.55,  1,  0,  0,0.93),
    new AprilTagCoordinate(30,  0.0127,  0.758,  0.55,  1,  0,  0,0.93),
    new AprilTagCoordinate(31,  0.0127,  3.73,  0.55,  1,  0,  0,0.93),
    new AprilTagCoordinate(32,  0.0127,  4.16,  0.55,  1,  0,  0,0.93)
'''

DATA = '''
1 Trench, Red (467.64) (292.31) (35.00) 180°
2 Hub, Red (469.11) (182.60) (44.25) 90°
3 Hub, Red (445.35) (172.84) (44.25) 180°
4 Hub, Red (445.35) (158.84) (44.25) 180°
5 Hub, Red (469.11) (135.09) (44.25) 270°
6 Trench, Red (467.64) (25.37) (35.00) 180°
7 Trench, Red (470.59) (25.37) (35.00) 0°
8 Hub, Red (483.11) (135.09) (44.25) 270°
9 Hub, Red (492.88) (144.84) (44.25) 0°
10 Hub, Red (492.88) (158.84) (44.25) 0°
11 Hub, Red (483.11) (182.60) (44.25) 90°
12 Trench, Red (470.59) (292.31) (35.00) 0°
13 Outpost, Red (650.92) (291.47) (21.75) 180°
14 Outpost, Red (650.92) (274.47) (21.75) 180°
15 Tower, Red (650.90) (170.22) (21.75) 180°
16 Tower, Red (650.90) (153.22) (21.75) 180°
17 Trench, Blue (183.59) (25.37) (35.00) 0°
18 Hub, Blue (182.11) (135.09) (44.25) 270°
19 Hub, Blue (205.87) (144.84) (44.25) 0°
20 Hub, Blue (205.87) (158.84) (44.25) 0°
21 Hub, Blue (182.11) (182.60) (44.25) 90°
22 Trench, Blue (183.59) (292.31) (35.00) 0°
23 Trench, Blue (180.64) (292.31) (35.00) 180°
24 Hub, Blue (168.11) (182.60) (44.25) 90°
25 Hub, Blue (158.34) (172.84) (44.25) 180°
26 Hub, Blue (158.34) (158.84) (44.25) 180°
27 Hub, Blue (168.11) (135.09) (44.25) 270°
28 Trench, Blue (180.64) (25.37) (35.00) 180°
29 Outpost, Blue (0.30) (26.22) (21.75) 0°
30 Outpost, Blue (0.30) (43.22) (21.75) 0°
31 Tower, Blue (0.32) (147.47) (21.75) 0°
32 Tower, Blue (0.32) (164.47) (21.75) 0°
'''



def parseData(data):
    pattern = r'^(\d+).*?\((\d+(?:\.\d+)?)\).*?\((\d+(?:\.\d+)?)\).*?\((\d+(?:\.\d+)?)\).*?(\d+(?:\.\d+)?)°'
    lines = DATA.split('\n')
    out = {}
    to_m = 0.0254
    for l in lines:
        l = l.strip()
        match = re.search(pattern, l)
        
        if match:
            apId = match.group(1)
            it = {
                "apId": int(apId),
                'x': float(match.group(2)) * to_m,
                'y': float(match.group(3)) * to_m,
                'z': float(match.group(4)) * to_m,
                'angle': float(match.group(5))
            }
            out[apId] = (it)
    return out

def parseCode(code):
    pattern = (
        r'^.*?\('
        r'\s*(-?\d+(?:\.\d+)?)\s*,\s*'
        r'\s*(-?\d+(?:\.\d+)?)\s*,\s*'
        r'\s*(-?\d+(?:\.\d+)?)\s*,\s*'
        r'\s*(-?\d+(?:\.\d+)?)\s*,\s*'
        r'\s*(-?\d+(?:\.\d+)?)\s*,\s*'
        r'\s*(-?\d+(?:\.\d+)?)\s*,\s*'
        r'\s*(-?\d+(?:\.\d+)?)\s*,\s*'
        r'\s*(-?\d+(?:\.\d+)?)\s*'
        r'\)'
    )
    lines = code.split('\n')
    out = []
    for l in lines:
        l = l.strip()
        if l.startswith('//'):
            out.append({'comment': l})
        elif l.startswith('new'):
            match = re.search(pattern, l)
            if match:
                apId = int(match.group(1))
                it = {
                    'apId': apId,
                    'p2': float(match.group(2)),
                    'p3': float(match.group(3)),
                    'p4': float(match.group(4)),
                    'p5': float(match.group(5)),
                    'p6': float(match.group(6)),
                    'p7': float(match.group(7)),
                    'p8': float(match.group(8)),
                }
                out.append(it)
    return out


data_out = parseData(DATA)
# print(data_out)

code_out = parseCode(CODES)
# print(code_out)

s = ''
for it in code_out:
    if it.get('comment'):
        s += it.get('comment') + '\n'
    else:
        apId = it['apId']
        obj = data_out[str(apId)]
        x = obj['x']
        y = obj['y']
        z = obj['z']
        angle = obj['angle']
        s += f"new AprilTagCoordinate({apId}, {x:0.3f}, {y:0.3f}, {z:0.3f}, {it['p5']}, {it['p6']}, {it['p7']}, {it['p8']}), // {angle}°\n"

print(s)