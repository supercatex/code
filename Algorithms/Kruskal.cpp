#include <cstdio>
#include <iostream>
#include <vector>
#include <map>
#include <string>
using namespace std;

struct Edge {
    int u, v, p;
    Edge(int u, int v, int p) {
        this->u = u;
        this->v = v;
        this->p = p;
    }
};

string get_key(int u, int v)
{
    return to_string(u) + ", " + to_string(v);
}

void quick_sort(vector<string> &a, map<string, Edge*> &w, int L, int R)
{
    if (R - L <= 0) return;
    int k = L;
    for (int i = L + 1; i <= R; i++)
        if (w[a[i]]->p < w[a[L]]->p)
            swap(a[i], a[++k]);
    swap(a[L], a[k]);
    quick_sort(a, w, L, k - 1);
    quick_sort(a, w, k + 1, R);
}

int find_root(vector<int> &a, int x)
{
    if (a[x] == x) return x;
    return a[x] = find_root(a, a[x]);
}

int main()
{
    int n, c; cin >> n;
    vector<string> keys;
    map<string, Edge*> w;
    while (cin >> c) {
        if (c == 0) break;

        for (int i = 0; i < c; i++) {
            int u, v, p; cin >> u >> v >> p;
            if (u > v) swap(u, v);
            string k = get_key(u, v);
            if (w.count(k) == 0) {
                w[k] = new Edge(u, v, p);
                keys.push_back(k);
            } else w[k]->p = p;
        }
        quick_sort(keys, w, 0, keys.size() - 1);

        int ans = 0;
        vector<int> a(n + 1, 0);
        for (int i = 1; i <= n; i++) a[i] = i;

        for (int i = 0; i < keys.size(); i++) {
            string k = keys[i];
            int u = w[k]->u, v = w[k]->v, p = w[k]->p;
            int ru = find_root(a, u), rv = find_root(a, v);
            if (ru != rv) {
                a[ru] = a[rv];
                ans += p;
            }
        }
        printf("%d\n", ans);
    }
	return 0;
}
