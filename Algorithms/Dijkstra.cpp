#include <cstdio>
#include <iostream>
#include <vector>
#include <map>
#include <queue>
using namespace std;

#define INF 1000000000

struct Node {
    int index;
    int value;
    Node (int index, int value) {
        this->index = index;
        this->value = value;
    }
};

struct cmp {
    bool operator()(Node* a, Node* b) {
        return a->value > b->value;
    }
};

int main() {
    int n, m, s;
    cin >> n >> m >> s;

    vector<map<int, int>> E(n + 1, map<int, int>());
    for (int i = 1; i <= m; i++) {
        int u, v, w; cin >> u >> v >> w;
        if (!E[u].count(v)) E[u][v] = w;
        else E[u][v] = min(E[u][v], w);
    }

    vector<int> D(n + 1, INF);
    vector<int> S(n + 1, 0);
    priority_queue<Node*, vector<Node*>, cmp> Q;

    D[s] = 0;
    for (int i = 1; i <= n; i++)
        Q.push(new Node(i, D[i]));

    while (Q.size() > 0) {
        Node* node = Q.top(); Q.pop();
        int u = node->index;
        if (S[u] == 1) continue;

        S[u] = 1;
        for (auto it = E[u].begin(); it != E[u].end(); it++) {
            int v = it->first, w = it->second;
            if (S[v] == 1) continue;
            if (D[v] > D[u] + w) {
                D[v] = D[u] + w;
                Q.push(new Node(v, D[v]));
            }
        }
    }
    for (int i = 1; i <= n; i++)
        printf("%d ", D[i] == INF ? -1 : D[i]);
    return 0;
}
