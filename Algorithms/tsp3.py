import cv2
import numpy as np
import random
import itertools as it
from func import *

def show_image(t=0):
    global a, v, _img, aa, vv
    img = np.zeros((_h, _w, 3))
    for i in range(n):
        cv2.circle(img, a[i], 2, (0, 0, 1), -1)
    resize_img = cv2.resize(img, (_w // 4 * 3, _h // 4 * 3))
    cv2.imshow("img", resize_img)
    return cv2.waitKey(t)

_h, _w = 600, 600
_img = np.zeros((_h, _w, 3))

a = []
n = 200
with open("TSP_data/tsp_%04d.in" % 7, "w+") as f:
    for i in range(n):
        x = random.randint(0, _w - 1)
        y = random.randint(0, _h - 1)
        a.append((x, y))
        f.write("%d,%d\n" % (x, y))

d = np.ones((n, n), dtype=np.float32) * 1e10
for i in range(n - 1):
    for j in range(i + 1, n):
        d[i][j] = calc_dist(a[i], a[j])
        d[j][i] = d[i][j]
print("Calculate distance.")

# e = [[[1e10 for k in range(n)] for j in range(n)] for i in range(n)]
# for k in range(n):
#     for i in range(n - 1):
#         for j in range(i + 1, n):
#             e[k][i][j] = increase_dist(a[k], a[i], a[j])
# print("Calculate increase distance.")

g = [i for i in range(n)]
_vi, _vj = [], []
d_order = np.unravel_index(np.argsort(d, axis=None), d.shape)
for k in range(len(d[d_order])):
    i, j = d_order[0][k], d_order[1][k]
    if i >= j: continue

    gi, gj = i, j
    while g[gi] != gi: gi = g[gi]
    while g[gj] != gj: gj = g[gj]
    if gi == gj: continue
    g[gj] = g[gi]
    _vi.append(i)
    _vj.append(j)

    _img = np.zeros((_h, _w, 3))
    for ii in range(n):
        cv2.circle(_img, a[ii], 2, (0, 0, 1), -1)
    for ii in range(len(_vi)):
        cv2.line(_img, a[_vi[ii]], a[_vj[ii]], (0, 1, 0), 1)
    resize_img = cv2.resize(_img, (_w // 4 * 4, _h // 4 * 4))
    cv2.imshow("img", resize_img)
    cv2.waitKey(1)

cv2.waitKey(0)

v = []
vc = [0 for i in range(n)]
while True:
    _img = np.zeros((_h, _w, 3))
    for i in range(n):
        cv2.circle(_img, a[i], 2, (0, 0, 1), -1)
    for i in range(len(_vi)):
        cv2.line(_img, a[_vi[i]], a[_vj[i]], (0, 1, 0), 1)
    for i in range(-1, len(v) - 1):
        found = False
        for k in range(len(_vi)):
            if v[i] == _vi[k] and v[i + 1] == _vj[k] or v[i] == _vj[k] and v[i + 1] == _vi[k]:
                found = True
                break
        if found: vc[v[i]] = 1
        color, thickness = (0, 1, 1), 1
        if vc[v[i]]: color, thickness = (1, 1, 0), 1
        cv2.line(_img, a[v[i]], a[v[i + 1]], color, thickness)
    resize_img = cv2.resize(_img, (_w // 4 * 4, _h // 4 * 4))
    cv2.imshow("img", resize_img)
    cv2.waitKey(1)

    aa = []
    for i in range(len(a)):
        if i not in v: aa.append(a[i])
    if len(aa) == 0: break

    vv = [
        np.argmin([i[0] for i in aa]),
        np.argmin([i[1] for i in aa]),
        np.argmax([i[0] for i in aa]),
        np.argmax([i[1] for i in aa])
    ]
    vv = list(dict.fromkeys(vv))

    while True:
        s = []
        flag = True
        for i in range(len(aa)):
            s.append(int(i in vv))
            if s[i]: continue
            for j in range(-1, len(vv) - 1):
                if is_intersected((0, 0), aa[i], aa[vv[j]], aa[vv[j + 1]]):
                    s[i] += 1
            if s[i] % 2 == 0: flag = False
        if flag: break

        target_i, target_j, target_d = -1, -1, 0
        for i in range(len(aa)):
            if s[i] % 2 == 1: continue
            min_j, min_d = -1, 1e10
            for j in range(-1, len(vv) - 1):
                d = increase_dist(aa[i], aa[vv[j]], aa[vv[j + 1]])
                if d < min_d:
                    min_d, min_j = d, j
            if min_d >= target_d:
                target_d, target_i, target_j = min_d, i, min_j

        vv.insert(target_j + 1, target_i)

    for k in range(len(vv)):
        i = a.index(aa[vv[k]])
        min_j, min_d = -1, 1e10
        for j in range(-1, len(v) - 1):
            d = increase_dist(a[i], a[v[j]], a[v[j + 1]])
            if d < min_d:
                min_d, min_j = d, j
        v.insert(min_j + 1, i)

cv2.waitKey(0)

_img = np.zeros((_h, _w, 3))
for i in range(n):
    cv2.circle(_img, a[i], 2, (0, 0, 1), -1)
for i in range(len(_vi)):
    cv2.line(_img, a[_vi[i]], a[_vj[i]], (0, 1, 0), 1)
for i in range(-1, len(v) - 1):
    color, thickness = (0, 1, 1), 1
    if vc[v[i]]: color, thickness = (1, 1, 0), 1
    cv2.line(_img, a[v[i]], a[v[i + 1]], color, thickness)
for i in range(-1, len(v) - 1):
    found = False
    for k in range(len(_vi)):
        if v[i] == _vi[k] and v[i + 1] == _vj[k] or v[i] == _vj[k] and v[i + 1] == _vi[k]:
            found = True
            break
    if found: continue
    # if vc[v[i - 1]] and vc[v[i + 1]]:
    #     cv2.line(img, a[v[i]], a[v[i + 1]], (1, 1, 0), 1)
resize_img = cv2.resize(_img, (_w // 4 * 4, _h // 4 * 4))
cv2.imshow("img", resize_img)
cv2.waitKey(0)

img2 = _img.copy()
for k in range(len(_vi)):
    i = v.index(_vi[k])
    j = v.index(_vj[k])
    if i + 1 == j or j + 1 == i: continue
    if (vc[v[i]] or vc[i - 1]) and (vc[v[j]] or vc[j - 1]):
        cv2.line(img2, a[_vi[k]], a[_vj[k]], (0, 0, 0), 1)

resize_img = cv2.resize(img2, (_w // 4 * 4, _h // 4 * 4))
cv2.imshow("img2", resize_img)
cv2.waitKey(1)

cv2.waitKey(0)
