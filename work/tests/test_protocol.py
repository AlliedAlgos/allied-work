from work.pc.protocol import EXPECTED_FIELDS, parse_line

GOOD = "S,1200,12.5,1,-2,3.5,7900,10,20,5,30,40,6,-,-,-,-,-,-,RED,40,12,0,100,90"


def test_parses_full_line():
    r = parse_line(GOOD)
    assert r["ms"] == 1200 and r["battery_mv"] == 7900
    assert r["gyro"] == {"heading": 12.5, "pitch": 1, "roll": -2, "yaw_rate": 3.5}
    assert r["motors"]["A"] == {"angle": 10, "speed": 20, "load": 5}
    assert r["motors"]["E"] is None and r["motors"]["F"] is None
    assert r["color"]["name"] == "RED" and r["color"]["reflection"] == 40


def test_missing_color_and_gyro_extras():
    line = "S,5,0,-,-,-,8000" + ",1,2,3" * 4 + ",-" * 6
    r = parse_line(line)
    assert r["color"] is None and r["gyro"]["pitch"] is None


def test_bad_lines_are_skipped_not_fatal():
    for bad in ["", "hello", "S,1,2", GOOD + ",9", "S," + "x," * (EXPECTED_FIELDS - 2) + "x",
                GOOD[:40], "L,5,log text"]:
        assert parse_line(bad) is None
