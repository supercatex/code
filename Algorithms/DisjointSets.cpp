#include <cstdio>
#include <iostream>
#include <vector>
using namespace std;

// 尋到 x 在數列a 中的代表
int find_root(vector<int> &a, int x)
{
    if (a[x] == x) return x;
    return a[x] = find_root(a, a[x]);   // 路徑壓縮
}

int main()
{
    int n, m, p;
    cin >> n >> m >> p;

    vector<int> a(n + 1, 0);                // 可轉用 map 節省空間
    for (int i = 1; i <= n; i++) a[i] = i;  // 預設自己是代表

    for (int i = 0; i < m; i++) {
        int mi, mj; cin >> mi >> mj;
        a[find_root(a, mi)] = a[find_root(a, mj)];  // 合並
    }

    for (int i = 0; i < p; i++) {
        int pi, pj; cin >> pi >> pj;
        if (find_root(a, pi) == find_root(a, pj)) { // 查詢
            printf("Yes\n");
        } else {
            printf("No\n");
        }
    }
    return 0;
}
