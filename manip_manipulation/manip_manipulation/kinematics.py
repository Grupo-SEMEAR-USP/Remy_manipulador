import numpy as np

L1, L2, L3 = 0.311, 0.308, 0.172
XS, ZS = -0.015, 0.060
Y_PLANO = -0.107782
LIM = 1.57

def fk_plano(q1, q2, q3, tool_len=0.0):
    f1, f2, f3 = -q1, -q2, -q3
    p = np.array([XS, ZS])
    p = p + L1 * np.array([np.sin(f1), np.cos(f1)])
    p = p + L2 * np.array([np.sin(f1 + f2), np.cos(f1 + f2)])
    a = f1 + f2 + f3
    p = p + (L3 + tool_len) * np.array([np.sin(a), np.cos(a)])
    return p[0], p[1], a


def ik_plano(x, z, alpha, tool_len=0.0, lim=LIM):
    w = np.array([x, z]) - (L3 + tool_len) * np.array([np.sin(alpha), np.cos(alpha)])
    d = w - np.array([XS, ZS])
    c2 = (d @ d - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(c2) > 1:
        return []
    sols = []
    for s in (+1, -1):
        f2 = s * np.arccos(c2)
        f1 = np.arctan2(d[0], d[1]) - np.arctan2(L2 * np.sin(f2), L1 + L2 * np.cos(f2))
        q = tuple(-np.array([f1, f2, alpha - f1 - f2]))
        if all(abs(v) <= lim for v in q):
            sols.append(q)
    return sols


def solve_facing(x, z, standoff=0.10, tool_len=0.0,
                 alpha_pref=np.pi / 2, alpha_span=np.radians(40), n=41):
    for a in sorted(np.linspace(alpha_pref - alpha_span, alpha_pref + alpha_span, n),
                    key=lambda a: abs(a - alpha_pref)):
        tcp = np.array([x, z]) - standoff * np.array([np.sin(a), np.cos(a)])
        sols = ik_plano(tcp[0], tcp[1], a, tool_len)
        if sols:
            return dict(ok=True, q=sols[0], alpha_deg=float(np.degrees(a)))
    return dict(ok=False, motivo='fora_do_alcance')