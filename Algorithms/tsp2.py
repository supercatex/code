import cv2
import numpy as np
import random
import itertools as it
from func import *

def show_image(t=0):
    global a, v, img, aa, vv
    img = np.zeros((h, w, 3))
    for i in range(n):
        cv2.circle(img, a[i], 2, (0, 0, 1), -1)
    for i in range(-1, len(v) - 1):
        cv2.line(img, a[v[i]], a[v[i + 1]], (0, 1, 0), 1)
    for i in range(-1, len(vv) - 1):
        cv2.line(img, aa[vv[i]], aa[vv[i + 1]], (0, 1, 1), 1)
    resize_img = cv2.resize(img, (w // 4 * 3, h // 4 * 3))
    cv2.imshow("img", resize_img)
    return cv2.waitKey(t)

def found_best():
    global num_p, list_p, a, v
    for i in range(-1, len(v) - (num_p + 2)):
        # show_image(1)
        p = []
        for j in range(num_p + 2):
            p.append(a[v[i + j]])
        d = []
        for j in range(len(list_p)):
            d.append(0)
            # print(list_p[j])
            d[j] += calc_dist(p[0], p[list_p[j][0]])
            for k in range(num_p - 1):
                d[j] += calc_dist(p[list_p[j][k]], p[list_p[j][k + 1]])
            d[j] += calc_dist(p[list_p[j][-1]], p[num_p])
        min_j = np.argmin(d)

        vv = []
        for j in range(0, num_p + 1):
            vv.append(v[i + j])
        for j in range(1, num_p):
            v[i + j] = vv[list_p[min_j][j - 1]]

num_p = 6
list_p = []
for e in it.permutations([i for i in range(1, num_p + 1)], num_p):
    list_p.append(e)

h, w = 600, 600
img = np.zeros((h, w, 3))

while True:
    a = []
    # with open("TSP_data/tsp_0007.in", "r") as f:
    #     lines = f.readlines()
    #     for line in lines:
    #         x, y = map(int, line.split(","))
    #         a.append((x, y))
    # n = len(a)
    n = 100
    with open("TSP_data/tsp_%04d.in" % 7, "w+") as f:
        for i in range(n):
            x = random.randint(0, w - 1)
            y = random.randint(0, h - 1)
            a.append((x, y))
            f.write("%d,%d\n" % (x, y))

    v = []
    while True:
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
            # show_image(1)

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

        prev_total = 0
        while True:
            total = 0
            for i in range(-1, len(v) - 1):
                total += calc_dist(a[v[i]], a[v[i + 1]])
            print("Renew:", total, "N:", len(v))
            show_image(1)
            if prev_total == total: break
            if len(v) > len(a): break
            prev_total = total

            min_i, min_j, min_d = -1, -1, 1e10
            for i in range(0, len(v) - 1):
                for j in range(-1, len(v) - 1):
                    if i in [j, j + 1]: continue

                    d1 = calc_dist(a[v[i - 1]], a[v[i]]) + calc_dist(a[v[i]], a[v[i + 1]])
                    d2 = calc_dist(a[v[j]], a[v[j + 1]])

                    d3 = calc_dist(a[v[i - 1]], a[v[i + 1]])
                    d4 = calc_dist(a[v[j]], a[v[i]]) + calc_dist(a[v[i]], a[v[j + 1]])
                    if d1 + d2 < d3 + d4: continue
                    if d3 + d4 < min_d:
                        min_i, min_j, min_d = i, j, d3 + d4
            if min_i == -1: break
            t = v[min_i]
            if min_j < min_i:
                v.remove(v[min_i])
                v.insert(min_j + 1, t)
            else:
                v.remove(v[min_i])
                v.insert(min_j, t)
            found_best()
        found_best()

        renew = True
        while renew:
            renew = False
            for i in range(-1, len(v) - 1):
                for j in range(i + 1, len(v) - 1):
                    if abs(i - j) <= 1: continue
                    p1, p2 = a[v[i]], a[v[i + 1]]
                    p3, p4 = a[v[j]], a[v[j + 1]]
                    if is_intersected(p1, p2, p3, p4):
                        v[i + 1:j + 1] = list(reversed(v[i + 1:j + 1]))
                        renew = True
                        break
                if renew: break
        found_best()

    found_best()

    total = 0
    for i in range(-1, len(v) - 1):
        total += calc_dist(a[v[i]], a[v[i + 1]])
    print(total)

    img = np.zeros((h, w, 3))
    for i in range(n):
        cv2.circle(img, a[i], 2, (0, 0, 1), -1)
    for i in range(-1, len(v) - 1):
        cv2.line(img, a[v[i]], a[v[i + 1]], (0, 1, 0), 1)
    resize_img = cv2.resize(img, (w // 4 * 3, h // 4 * 3))
    cv2.imshow("img", resize_img)
    cv2.waitKey(0)
