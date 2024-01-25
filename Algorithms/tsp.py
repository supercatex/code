import cv2
import numpy as np
import random
import itertools as it
from func import *

def show_image(t=0):
    global a, v, img
    img = np.zeros((h, w, 3))
    for i in range(n):
        cv2.circle(img, a[i], 2, (0, 0, 1), -1)
    for i in range(-1, len(v) - 1):
        cv2.line(img, a[v[i]], a[v[i + 1]], (0, 1, 0), 1)
    cv2.imshow("img", img)
    return cv2.waitKey(t)

def found_best():
    global num_p, list_p, a, v
    for i in range(-1, len(v) - (num_p + 2)):
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

num_p = 5
list_p = []
for e in it.permutations([i for i in range(1, num_p + 1)], num_p):
    list_p.append(e)

h, w = 1000, 1000
img = np.zeros((h, w, 3))
n = 200
while True:
    a = []
    # for i in range(n):
    #     x = random.randint(0, w - 1)
    #     y = random.randint(0, h - 1)
    #     a.append((x, y))
    with open("TSP_data/tsp_0006.in", "r") as f:
        lines = f.readlines()
        for line in lines:
            x, y = map(int, line.split(","))
            a.append((x, y))
    n = len(a)

    v = [
        np.argmin([i[0] for i in a]),
        np.argmin([i[1] for i in a]),
        np.argmax([i[0] for i in a]),
        np.argmax([i[1] for i in a])
    ]
    v = list(dict.fromkeys(v))

    t = 1
    while True:
        show_image(t)

        s = []
        flag = True
        for i in range(n):
            s.append(int(i in v))
            if s[i]: continue
            for j in range(-1, len(v) - 1):
                if is_intersected((0, 0), a[i], a[v[j]], a[v[j + 1]]):
                    s[i] += 1
            if s[i] % 2 == 0: flag = False
        if flag: break

        # for i in range(n):
        #     if s[i] % 2 == 1:
        #         cv2.circle(img, a[i], 3, (0, 1, 1), -1)
        # cv2.imshow("img", img)
        # cv2.waitKey(t)

        target_i, target_j, target_d = -1, -1, 0
        for i in range(n):
            if s[i] % 2 == 1: continue
            min_j, min_d = -1, 1e10
            for j in range(-1, len(v) - 1):
                d = increase_dist(a[i], a[v[j]], a[v[j + 1]])
                if d < min_d:
                    min_d, min_j = d, j
            if min_d >= target_d:
                target_d, target_i, target_j = min_d, i, min_j

        # cv2.circle(img, a[target_i], 3, (0, 1, 1), -1)
        # cv2.line(img, a[v[target_j]], a[v[target_j + 1]], (0, 1, 1), 1)
        # cv2.imshow("img", img)
        # cv2.waitKey(t)

        v.insert(target_j + 1, target_i)

    t = 1
    while len(v) < len(a):
        show_image(t)

        target_i, target_j, target_d = -1, -1, 1e10
        for i in range(n):
            if i in v: continue
            min_j, min_d = -1, 1e10
            for j in range(-1, len(v) - 1):
                d = increase_dist(a[i], a[v[j]], a[v[j + 1]])
                if d < min_d:
                    min_d, min_j = d, j
            if min_d <= target_d:
                target_i, target_j, target_d = i, min_j, min_d

        v.insert(target_j + 1, target_i)

        found_best()

        # for i in range(-1, len(v) - (num_p + 2)):
        #     p = []
        #     for j in range(num_p + 2):
        #         p.append(a[v[i + j]])
        #     d = []
        #     for j in range(len(list_p)):
        #         d.append(0)
        #         # print(list_p[j])
        #         d[j] += calc_dist(p[0], p[list_p[j][0]])
        #         for k in range(num_p - 1):
        #             d[j] += calc_dist(p[list_p[j][k]], p[list_p[j][k + 1]])
        #         d[j] += calc_dist(p[list_p[j][-1]], p[num_p])
        #     min_j = np.argmin(d)
        #
        #     vv = []
        #     for j in range(0, num_p + 1):
        #         vv.append(v[i + j])
        #     for j in range(1, num_p):
        #         v[i + j] = vv[list_p[min_j][j - 1]]

        # while len(v) > 5:
        #     flag = True
        #     for i in range(-1, len(v) - 4):
        #         p0 = a[v[i + 0]]
        #         p1 = a[v[i + 1]]
        #         p2 = a[v[i + 2]]
        #         p3 = a[v[i + 3]]
        #         pN = a[v[i + 4]]
        #         d1 = dist(p0, p1) + dist(p1, p2) + dist(p2, p3) + dist(p3, pN)
        #         d2 = dist(p0, p2) + dist(p2, p1) + dist(p1, p3) + dist(p3, pN)
        #         d3 = dist(p0, p2) + dist(p2, p3) + dist(p3, p1) + dist(p1, pN)
        #         d4 = dist(p0, p3) + dist(p3, p1) + dist(p1, p2) + dist(p2, pN)
        #         d5 = dist(p0, p3) + dist(p3, p2) + dist(p2, p1) + dist(p1, pN)
        #         d6 = dist(p0, p1) + dist(p1, p3) + dist(p3, p2) + dist(p2, pN)
        #         min_d = min([d1, d2, d3, d4, d5, d6])
        #         if d1 == min_d: continue
        #         flag = False
        #         if d2 == min_d: v[i + 1], v[i + 2], v[i + 3] = v[i + 2], v[i + 1], v[i + 3]
        #         if d3 == min_d: v[i + 1], v[i + 2], v[i + 3] = v[i + 2], v[i + 3], v[i + 1]
        #         if d4 == min_d: v[i + 1], v[i + 2], v[i + 3] = v[i + 3], v[i + 1], v[i + 2]
        #         if d5 == min_d: v[i + 1], v[i + 2], v[i + 3] = v[i + 3], v[i + 2], v[i + 1]
        #         if d6 == min_d: v[i + 1], v[i + 2], v[i + 3] = v[i + 1], v[i + 3], v[i + 2]
        #     if flag: break

    total = 0
    for i in range(-1, n - 1):
        total += calc_dist(a[v[i]], a[v[i + 1]])
    print("1", total)
    show_image(1)

    for i in range(-1, n - 1):
        for j in range(i + 1, n - 1):
            if abs(i - j) <= 1: continue
            p1, p2 = a[v[i]], a[v[i + 1]]
            p3, p4 = a[v[j]], a[v[j + 1]]
            if is_intersected(p1, p2, p3, p4):
                v[i + 1:j + 1] = list(reversed(v[i + 1:j + 1]))

    found_best()

    for i in range(-1, n - 1):
        for j in range(-1, len(v) - 1):
            if i in [j, j + 1]: continue

            d1 = calc_dist(a[v[i - 1]], a[v[i]]) + calc_dist(a[v[i]], a[v[i + 1]])
            d2 = calc_dist(a[v[j]], a[v[j + 1]])

            d3 = calc_dist(a[v[i - 1]], a[v[i + 1]])
            d4 = calc_dist(a[v[j]], a[v[i]]) + calc_dist(a[v[i]], a[v[j + 1]])
            if d1 + d2 < d3 + d4: continue

            t = v[i]
            if j < i:
                v.remove(v[i])
                v.insert(j + 1, t)
            else:
                v.remove(v[i])
                v.insert(j, t)
            if show_image(1) in [27, ord('q')]:
                flag = False
            # break

    found_best()

    total = 0
    for i in range(-1, n - 1):
        total += calc_dist(a[v[i]], a[v[i + 1]])
    print("2", total)

    show_image(1)
    cv2.putText(img, "%.4f" % total, (10, 30), cv2.FONT_HERSHEY_PLAIN, 1.0, (0, 0, 1), 2)
    cv2.imshow("img", img)
    cv2.waitKey(0)
