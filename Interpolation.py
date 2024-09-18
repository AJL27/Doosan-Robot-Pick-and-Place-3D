from scipy.optimize import fsolve

def forward_robot(x, *jtvecin):
    r, s, t = x

    rst = r * s * t
    rs = r * s
    rt = r * t
    st = s * t

    n1 = -.125 * (1 - r) * (1 - t) * (1 - s) * rst
    n2 = .125 * (1 + r) * (1 - t) * (1 - s) * rst
    n3 = -.125 * (1 + r) * (1 - t) * (1 + s) * rst
    n4 = .125 * (1 - r) * (1 - t) * (1 + s) * rst

    n5 = .125 * (1 - r) * (1 + t) * (1 - s) * rst
    n6 = -.125 * (1 + r) * (1 + t) * (1 - s) * rst
    n7 = .125 * (1 + r) * (1 + t) * (1 + s) * rst
    n8 = -.125 * (1 - r) * (1 + t) * (1 + s) * rst

    n9 = .25 * (1 - r) * (1 - s) * (1 - t) * (1 + t) * rs
    n10 = -.25 * (1 + r) * (1 - s) * (1 - t) * (1 + t) * rs
    n11 = .25 * (1 + r) * (1 + s) * (1 - t) * (1 + t) * rs
    n12 = -.25 * (1 - r) * (1 + s) * (1 - t) * (1 + t) * rs

    n13 = .25 * (1 - r) * (1 + r) * (1 - s) * (1 - t) * st
    n14 = -.25 * (1 + r) * (1 - s) * (1 + s) * (1 - t) * rt
    n15 = -.25 * (1 - r) * (1 + r) * (1 + s) * (1 - t) * st
    n16 = .25 * (1 - r) * (1 - s) * (1 + s) * (1 - t) * rt
    n17 = -.5 * (1 + r) * (1 - r) * (1 + s) * (1 - s) * (1 - t) * t

    n18 = -.25 * (1 - r) * (1 + r) * (1 - s) * (1 + t) * st
    n19 = .25 * (1 + r) * (1 - s) * (1 + s) * (1 + t) * rt
    n20 = .25 * (1 - r) * (1 + r) * (1 + s) * (1 + t) * st
    n21 = -.25 * (1 - r) * (1 - s) * (1 + s) * (1 + t) * rt
    n22 = .5 * (1 + r) * (1 - r) * (1 + s) * (1 - s) * (1 + t) * t

    n23 = -.5 * (1 - r) * (1 + r) * (1 - s) * (1 + t) * (1 - t) * s
    n24 = .5 * (1 + r) * (1 - s) * (1 + s) * (1 - t) * (1 + t) * r
    n25 = .5 * (1 - r) * (1 + r) * (1 + s) * (1 - t) * (1 + t) * s
    n26 = -.5 * (1 - r) * (1 - s) * (1 + s) * (1 - t) * (1 + t) * r
    n27 = (1 - r) * (1 + r) * (1 - t) * (1 + t) * (1 - s) * (1 + s)

    n = [n1, n2, n3, n4,
         n5, n6, n7, n8,
         n9, n10, n11, n12,
         n13, n14, n15, n16, n17,
         n18, n19, n20, n21, n22,
         n23, n24, n25, n26, n27]

    jt1val, jt2val, jt3val, jt4val, jt5val, jt6val = 0,0,0,0,0,0

    for i in range(27):  # Each joint angle calculation
        jt1val += jtvecin[0][i][0] * n[i]
        jt2val += jtvecin[0][i][1] * n[i]
        jt3val += jtvecin[0][i][2] * n[i]
        jt4val += jtvecin[0][i][3] * n[i]
        jt5val += jtvecin[0][i][4] * n[i]
        jt6val += jtvecin[0][i][5] * n[i]

    return [jt1val, jt2val, jt3val, jt4val, jt5val, jt6val]


def forwardcubic(x, *ptvecin):
    r, s, t = x

    rst = r * s * t
    rs = r * s
    rt = r * t
    st = s * t

    n1 = -.125 * (1 - r) * (1 - t) * (1 - s) * rst
    n2 = .125 * (1 + r) * (1 - t) * (1 - s) * rst
    n3 = -.125 * (1 + r) * (1 - t) * (1 + s) * rst
    n4 = .125 * (1 - r) * (1 - t) * (1 + s) * rst

    n5 = .125 * (1 - r) * (1 + t) * (1 - s) * rst
    n6 = -.125 * (1 + r) * (1 + t) * (1 - s) * rst
    n7 = .125 * (1 + r) * (1 + t) * (1 + s) * rst
    n8 = -.125 * (1 - r) * (1 + t) * (1 + s) * rst

    n9 = .25 * (1 - r) * (1 - s) * (1 - t) * (1 + t) * rs
    n10 = -.25 * (1 + r) * (1 - s) * (1 - t) * (1 + t) * rs
    n11 = .25 * (1 + r) * (1 + s) * (1 - t) * (1 + t) * rs
    n12 = -.25 * (1 - r) * (1 + s) * (1 - t) * (1 + t) * rs

    n13 = .25 * (1 - r) * (1 + r) * (1 - s) * (1 - t) * st
    n14 = -.25 * (1 + r) * (1 - s) * (1 + s) * (1 - t) * rt
    n15 = -.25 * (1 - r) * (1 + r) * (1 + s) * (1 - t) * st
    n16 = .25 * (1 - r) * (1 - s) * (1 + s) * (1 - t) * rt
    n17 = -.5 * (1 + r) * (1 - r) * (1 + s) * (1 - s) * (1 - t) * t

    n18 = -.25 * (1 - r) * (1 + r) * (1 - s) * (1 + t) * st
    n19 = .25 * (1 + r) * (1 - s) * (1 + s) * (1 + t) * rt
    n20 = .25 * (1 - r) * (1 + r) * (1 + s) * (1 + t) * st
    n21 = -.25 * (1 - r) * (1 - s) * (1 + s) * (1 + t) * rt
    n22 = .5 * (1 + r) * (1 - r) * (1 + s) * (1 - s) * (1 + t) * t

    n23 = -.5 * (1 - r) * (1 + r) * (1 - s) * (1 + t) * (1 - t) * s
    n24 = .5 * (1 + r) * (1 - s) * (1 + s) * (1 - t) * (1 + t) * r
    n25 = .5 * (1 - r) * (1 + r) * (1 + s) * (1 - t) * (1 + t) * s
    n26 = -.5 * (1 - r) * (1 - s) * (1 + s) * (1 - t) * (1 + t) * r
    n27 = (1 - r) * (1 + r) * (1 - t) * (1 + t) * (1 - s) * (1 + s)

    n = [n1, n2, n3, n4,
         n5, n6, n7, n8,
         n9, n10, n11, n12,
         n13, n14, n15, n16, n17,
         n18, n19, n20, n21, n22,
         n23, n24, n25, n26, n27]

    val1 = 0
    val2 = 0
    val3 = 0


    for i in range(27):
        val1 += ptvecin[0][i][0] * n[i]
        val2 += ptvecin[0][i][1] * n[i]
        val3 += ptvecin[0][i][2] * n[i]

    return [val1, val2, val3]

def reverse_error(x, *args):
    # error function for finding p,q roots
    rst = args[0][0]
    ptvec = args[0][1]
    x1 = forwardcubic(x, ptvec)
    err = [(x1[0] - rst[0]) * (x1[0] - rst[0]),
           (x1[1] - rst[1]) * (x1[1] - rst[1]),
           (x1[2] - rst[2]) * (x1[2] - rst[2])]
    return err


def reverse(r, s, t, *ptvec):
    data = [[r, s, t], ptvec[0]]
    x0 = [-.25, -.25, -.25]
    p = fsolve(reverse_error, x0, data)
    return p


def map(r, s, t, rspts, xypts, jtpts):
    p = reverse(r, s, t, rspts)
    j = forward_robot(p, jtpts)
    y = forwardcubic(p, xypts)
    return y, j


#xyz, joints = map(0, 0, 0, avec, avec, jtvec)



