"""이음매판 구간 침목 인양(밀착) 유압 클램프 - 파라메트릭 CAD (CadQuery)

좌표계: X=레일 직각(횡), Y=레일 길이방향, Z=상향, Z=0 은 침목 상면.
레일 저부는 침목 상면에서 GAP 만큼 떠 있는 상태(작동 전)로 모델링한다.
사용:  python build_clamp.py [GAP_mm]   -> out/ 에 STEP/STL 생성
치수는 개념설계용 근사값이다. 실제 제작 전 현장 실측(숄더 구멍, 이음매판)으로 교체할 것.
"""
import os, sys
import cadquery as cq

GAP = float(sys.argv[1]) if len(sys.argv) > 1 else 18.0   # 레일-침목 이격 (경험상 15~20)
OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "out")

# 레일 단면 근사 (mm): H 높이, B 저부폭, HW 두부폭, WEB 복부두께, HH 두부높이, TOE/ROOT 저부 두께
RAILS = {
    "50kg": dict(H=153, B=127, HW=65, WEB=15.0, HH=49, TOE=10, ROOT=28),
    "60kg": dict(H=174, B=145, HW=65, WEB=16.5, HH=51, TOE=11, ROOT=30),
    "60E1": dict(H=172, B=150, HW=72, WEB=16.5, HH=51, TOE=11, ROOT=30),
}
# 침목 숄더(e클립용) 가정치
SH_GAP, SH_X, SH_Y, SH_Z = 4.0, 30.0, 50.0, 46.0   # 저부끝~숄더 간격, 숄더 x폭/y폭/높이
HOLE_D, HOLE_Z = 20.0, 22.0                          # 체결구멍 지름/중심 높이(침목상면 기준)
# 클램프 가정치
PIN_D, PIN_L = 20.0, 30.0
LEG_T, LEG_W = 14.0, 44.0
BEAM_T, BEAM_W = 28.0, 56.0
RAM_D, RAM_H, PLUNGER_D, STROKE = 56.0, 80.0, 36.0, 40.0
PAD_T = 6.0


def prism_yz(pts_xz, length, y0=None):
    """XZ 평면 단면을 Y방향으로 length 만큼 압출 (Y 중심 정렬)."""
    wp = cq.Workplane("XZ").polyline(pts_xz).close().extrude(length / 2.0, both=True)
    return wp


def rail_solid(p, z0, length=640):
    H, B, HW, W, HH, T, R = (p[k] for k in ("H", "B", "HW", "WEB", "HH", "TOE", "ROOT"))
    hb, hw, hh = B / 2, HW / 2, W / 2
    xr = hh + 12
    right = [(hb, 0), (hb, T), (xr, R), (hh, R + 8), (hh, H - HH - 10),
             (hw - 6, H - HH + 6), (hw, H - 14), (hw - 8, H)]
    right = [(x, z0 + z) for x, z in right]
    left = [(-x, z) for x, z in reversed(right)]
    pts = right + left
    return prism_yz(pts, length)


def base_top(p, z0, x):
    hb, hh = p["B"] / 2, p["WEB"] / 2
    xr = hh + 12
    if x <= xr:
        return z0 + p["ROOT"]
    return z0 + p["ROOT"] + (p["TOE"] - p["ROOT"]) * (x - xr) / (hb - xr)


def joint_bar(p, z0, side, length=600):
    """이음매판(앵글바) 단순화 - side=+1/-1"""
    hh = p["WEB"] / 2
    zt = z0 + p["H"] - p["HH"] - 4
    zf = base_top(p, z0, hh + 34)
    pts = [(hh, z0 + p["ROOT"] - 2), (hh + 34, zf - 0.5), (hh + 34, zf + 14), (hh + 20, zf + 14),
           (hh + 20, zt), (hh, zt)]
    pts = [(side * x, z) for x, z in pts]
    s = prism_yz(pts, length)
    # 볼트 4개
    zb = (zt + zf) / 2 + 6
    for y in (-190, -95, 95, 190):
        shaft = cq.Workplane("YZ").center(y, zb).circle(11).extrude(side * (hh + 60) if side > 0 else -(hh + 60))
        head = cq.Workplane("YZ").workplane(offset=side * (hh + 20)).center(y, zb).polygon(6, 38).extrude(side * 14)
        s = s.union(head).union(shaft.translate((0, 0, 0)))
    return s


def sleeper(p):
    blk = cq.Workplane("XY").box(560, 250, 190, centered=(True, True, False)).translate((0, 0, -190))
    for side in (1, -1):
        x0 = p["B"] / 2 + SH_GAP
        sh = cq.Workplane("XY").box(SH_X, SH_Y, SH_Z, centered=(False, True, False)).translate((x0, 0, 0))
        hole = cq.Workplane("YZ").center(0, HOLE_Z).circle(HOLE_D / 2).extrude(SH_X + 2).translate((x0 - 1, 0, 0))
        sh = sh.cut(hole)
        if side < 0:
            sh = sh.mirror("YZ")
        blk = blk.union(sh)
    return blk


def clamp_parts(p, z0):
    hb = p["B"] / 2
    z_head = z0 + p["H"]
    x_leg_in = hb + SH_GAP + SH_X + 2          # 레그 안쪽면 (숄더 바깥면 +2)
    parts = {}
    # 절연 패드 (두부 상면)
    parts["pad"] = cq.Workplane("XY").box(p["HW"] - 6, 74, PAD_T, centered=(True, True, False)).translate((0, 0, z_head))
    # 실린더 (두부 위 설치, 5tf급 단동 단행정 램)
    cyl = cq.Workplane("XY").circle(RAM_D / 2).extrude(RAM_H).translate((0, 0, z_head + PAD_T))
    collar = cq.Workplane("XY").circle(RAM_D / 2 + 6).extrude(14).translate((0, 0, z_head + PAD_T + RAM_H - 14))
    parts["cylinder"] = cyl
    parts["lock_collar"] = collar
    z_plung_top = z_head + PAD_T + RAM_H + 6
    parts["plunger"] = cq.Workplane("XY").circle(PLUNGER_D / 2).extrude(z_plung_top - (z_head + PAD_T + RAM_H - 20)).translate((0, 0, z_head + PAD_T + RAM_H - 20))
    # 크로스빔 (요크)
    z_beam0 = z_plung_top
    xo = x_leg_in + LEG_T
    beam = cq.Workplane("XY").box(2 * xo, BEAM_W, BEAM_T, centered=(True, True, False)).translate((0, 0, z_beam0))
    # 레그 + 핀 (양측)
    yoke = beam
    z_leg_bot = HOLE_Z - LEG_W / 2
    for side in (1, -1):
        leg = cq.Workplane("XY").box(LEG_T, LEG_W, (z_beam0 + BEAM_T) - z_leg_bot, centered=(False, True, False)).translate((x_leg_in, 0, z_leg_bot))
        pin = cq.Workplane("YZ").center(0, HOLE_Z).circle(PIN_D / 2 - 0.5).extrude(PIN_L + 2).translate((x_leg_in - PIN_L, 0, 0))
        piece = leg.union(pin)
        if side < 0:
            piece = piece.mirror("YZ")
        yoke = yoke.union(piece)
    parts["yoke"] = yoke
    return parts


def build(name, p):
    z0 = GAP
    parts = {
        "sleeper": sleeper(p),
        "rail": rail_solid(p, z0),
        "joint_bar_R": joint_bar(p, z0, +1),
        "joint_bar_L": joint_bar(p, z0, -1),
    }
    parts.update(clamp_parts(p, z0))
    colors = {"sleeper": (0.62, 0.62, 0.60), "rail": (0.55, 0.38, 0.28), "joint_bar_R": (0.35, 0.40, 0.45),
              "joint_bar_L": (0.35, 0.40, 0.45), "pad": (0.95, 0.9, 0.7), "cylinder": (0.9, 0.55, 0.1),
              "lock_collar": (0.8, 0.15, 0.1), "plunger": (0.8, 0.8, 0.85), "yoke": (0.1, 0.45, 0.8)}
    d = os.path.join(OUT, name)
    os.makedirs(d, exist_ok=True)
    asm = cq.Assembly(name=f"clamp_{name}")
    for k, v in parts.items():
        asm.add(v, name=k, color=cq.Color(*colors[k]))
        cq.exporters.export(v, os.path.join(d, f"{k}.stl"), tolerance=0.05, angularTolerance=0.2)
    asm.save(os.path.join(OUT, f"clamp_{name}.step"))
    print(name, "OK", {k: round(v.val().Volume() / 1e3) for k, v in parts.items() if k in ("yoke", "cylinder")})


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for n, p in RAILS.items():
        build(n, p)
