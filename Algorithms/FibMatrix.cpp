#include <cstdio>
#include <iostream>
#include <vector>
using namespace std;

#define MOD 1000000000
#define LL long long int

struct M {
    LL a, b, c, d;
    M(LL a, LL b, LL c, LL d) {
        this->a = a % MOD;
        this->b = b % MOD;
        this->c = c % MOD;
        this->d = d % MOD;
    }
};

M* mul_M(M* m1, M* m2) {
    LL a = m1->a * m2->a + m1->b * m2->c;
    LL b = m1->a * m2->b + m1->b * m2->d;
    LL c = m1->c * m2->a + m1->d * m2->c;
    LL d = m1->c * m2->b + m1->d * m2->d;
    return new M(a, b, c, d);
}

M* pow_M(M* m, int x)
{
    if (x == 1) return m;
    M* res = pow_M(m, x / 2);
    res = mul_M(res, res);
    if (x % 2 == 1) res = mul_M(m, res);
    return res;
}

int main()
{
    int a1, a2, n;
    while (cin >> a1 >> a2 >> n) {
        if (!a1 && !a2 && !n) break;
        M* m1 = new M(1, 1, 1, 0);
        M* m2 = new M(a2, a1, 0, 0);
        M* m3 = mul_M(m2, pow_M(m1, n - 2));
        printf("%d\n", m3->a);
    }
    return 0;
}
