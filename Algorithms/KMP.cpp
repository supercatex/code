#include <cstdio>
#include <iostream>
#include <vector>
#include <string>
using namespace std;

template <typename T> using V1 = vector<T>;
template <typename T> using V2 = vector<V1<T>>;

V1<int> get_LPS(string p)
{
    V1<int> res(p.size(), 0);
    for (int i = 1; i < p.size(); i++) {
        int j = res[i - 1];
        while (j > 0 && p[i] != p[j])
            j = res[j - 1];
        if (p[i] == p[j]) j++;
        res[i] = j;
    }
    return res;
}

V1<int> get_KMP(string s, string p, V1<int> &LPS)
{
    V1<int> res;
    for (int i = 0, j = 0; i < s.size(); i++) {
        while (j > 0 && s[i] != p[j])
            j = LPS[j - 1];
        if (s[i] == p[j]) j++;
        if (j == p.size()) {
            res.push_back(i - p.size() + 1);
            j = LPS[j - 1];
        }
    }
    return res;
}

int main()
{
    string s, p; cin >> s >> p;
    V1<int> LPS = get_LPS(p);
    V1<int> res = get_KMP(s, p, LPS);
    for (int i = 0; i < res.size(); i++)
        printf("%d\n", res[i] + 1);
    for (int i = 0; i < LPS.size(); i++)
       printf("%d ", LPS[i]);
    printf("\n");
    return 0;
}
