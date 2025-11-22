#include <bits/stdc++.h>
using namespace std;

void merge_sort(vector<int> &a, int L, int R) {
    if (L >= R) return;
    
    int M = (L + R) / 2;
    merge_sort(a, L, M);
    merge_sort(a, M + 1, R);
    
    vector<int> a1, a2;
    for (int i = L; i <= M; i++) a1.push_back(a[i]);
    for (int j = M + 1; j <= R; j++) a2.push_back(a[j]);
    int i = 0, j = 0;
    for (int k = L; k <= R; k++)
        if (j >= a2.size() || i < a1.size() && a1[i] < a2[j])
            a[k] = a1[i++];
        else
            a[k] = a2[j++];
}

void quick_sort(vector<int> &a, int L, int R) {
    if (L >= R) return;
    
    int k = L;
    for (int i = L + 1; i <= R; i++)
        if (a[i] <= a[k])
            swap(a[i], a[k]);
            
    swap(a[L], a[k]);
    quick_sort(a, L, k - 1);
    quick_sort(a, k + 1, R);
}

int main()
{
    int n; cin >> n;
    vector<int> a(n, 0);
    for (int i = 0; i < n; i++) cin >> a[i];
    
    // merge_sort(a, 0, n - 1);
    // quick_sort(a, 0, n - 1);
    
    for (int i = 0; i < n; i++)
        printf("%d ", a[i]);
    printf("\n");
    return 0;
}
