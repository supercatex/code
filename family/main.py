from problog.program import Term, Var, SimpleProgram
from problog.engine import DefaultEngine


def print_relationship(a, b, terms, program):
    engine = DefaultEngine()
    db = engine.prepare(program)

    print("[%s]是[%s]的 --> " % (b, a), end="")
    for key in terms:
        query_term = terms[key](a, b)
        res = engine.query(db, query_term)
        if bool(res):
            print(terms[key], end=" ")
    print("")


t = dict()
t["myself"] = Term("自己")
t["husband"] = Term("丈夫")
t["wife"] = Term("妻子")
t["couple"] = Term("配偶")
t["son"] = Term("兒子")
t["daughter"] = Term("女兒")
t["child"] = Term("孩子")
t["parent"] = Term("父母")
t["mother"] = Term("媽媽")
t["father"] = Term("爸爸")
t["sibling"] = Term("手足")
t["grandfather"] = Term("祖父")
t["grandmother"] = Term("祖母")
t["grandparents"] = Term("祖父母")
t["grandson"] = Term("孫兒子")
t["granddaughter"] = Term("孫女兒")
t["grandchildren"] = Term("孫子女")
t["grandfather_"] = Term("外祖父")
t["grandmother_"] = Term("外祖母")
t["grandparents_"] = Term("外祖父母")
t["grandson_"] = Term("外孫兒子")
t["granddaughter_"] = Term("外孫女兒")
t["grandchildren_"] = Term("外孫子女")
t["son-wife"] = Term("媳婦")
t["daughter-husband"] = Term("女婿")
t["husband-mother"] = Term("婆婆")
t["husband-father"] = Term("公公")
t["wife-mother"] = Term("丈母娘")
t["wife-father"] = Term("丈人")

X, Y, Z = map(Var, ["X", "Y", "Z"])
sp = SimpleProgram()
sp += t["myself"](X, Y) << t["myself"](X, Y)
sp += t["husband"](X, Y) << t["wife"](Y, X)
sp += t["wife"](X, Y) << t["husband"](Y, X)
sp += t["couple"](X, Y) << (t["husband"](X, Y) | t["wife"](X, Y))
sp += t["couple"](X, Y) << t["couple"](Y, X)
sp += t["son"](X, Y) << (t["son"](X, Y) | (t["couple"](X, Z) & t["son"](Z, Y)))
sp += t["daughter"](X, Y) << (t["daughter"](X, Y) | (t["couple"](X, Z) & t["daughter"](Z, Y)))
sp += t["child"](X, Y) << (t["son"](X, Y) | t["daughter"](X, Y))
sp += t["child"](X, Y) << (t["father"](Y, X) | t["mother"](Y, X))
sp += t["child"](X, Y) << (t["couple"](X, Z) & t["child"](Z, Y))
sp += t["parent"](X, Y) << t["child"](Y, X)
sp += t["mother"](X, Y) << (t["wife"](Z, Y) & t["child"](Y, X))
sp += t["father"](X, Y) << (t["husband"](Z, Y) & t["child"](Y, X))
sp += t["sibling"](X, Y) << (t["parent"](X, Z) & t["parent"](Y, Z))
sp += t["grandfather"](X, Y) << (t["father"](X, Z) & t["father"](Z, Y))
sp += t["grandmother"](X, Y) << (t["father"](X, Z) & t["mother"](Z, Y))
sp += t["grandparents"](X, Y) << (t["grandfather"](X, Y) | t["grandmother"](X, Y))
sp += t["grandson"](X, Y) << (t["son"](X, Z) & t["son"](Z, Y))
sp += t["granddaughter"](X, Y) << (t["son"](X, Z) & t["daughter"](Z, Y))
sp += t["grandchildren"](X, Y) << (t["son"](X, Z) & t["child"](Z, Y))
sp += t["grandfather_"](X, Y) << (t["mother"](X, Z) & t["father"](Z, Y))
sp += t["grandmother_"](X, Y) << (t["mother"](X, Z) & t["mother"](Z, Y))
sp += t["grandparents_"](X, Y) << (t["grandfather_"](X, Y) | t["grandmother_"](X, Y))
sp += t["grandson_"](X, Y) << (t["daughter"](X, Z) & t["son"](Z, Y))
sp += t["granddaughter_"](X, Y) << (t["daughter"](X, Z) & t["daughter"](Z, Y))
sp += t["grandchildren_"](X, Y) << (t["daughter"](X, Z) & t["child"](Z, Y))
sp += t["son-wife"](X, Y) << (t["husband"](Y, Z) & t["parent"](Z, X))
sp += t["daughter-husband"](X, Y) << (t["wife"](Y, Z) & t["parent"](Z, X))
sp += t["daughter-husband"](X, Y) << (t["wife"](Y, Z) & t["parent"](Z, X))
sp += t["husband-mother"](X, Y) << (t["husband"](X, Z) & t["mother"](Z, Y))
sp += t["husband-father"](X, Y) << (t["husband"](X, Z) & t["father"](Z, Y))
sp += t["wife-mother"](X, Y) << (t["wife"](X, Z) & t["mother"](Z, Y))
sp += t["wife-father"](X, Y) << (t["wife"](X, Z) & t["father"](Z, Y))

n = dict()
n["林爸爸"] = Term("林爸爸")
n["林媽媽"] = Term("林媽媽")
n["林本人"] = Term("林本人")
n["林妹妹"] = Term("林妹妹")
n["林太太"] = Term("林太太")
n["林囡囡"] = Term("林囡囡")
n["男朋友"] = Term("男朋友")
n["張爸爸"] = Term("張爸爸")
n["張媽媽"] = Term("張媽媽")

sp += t["wife"](n["林爸爸"], n["林媽媽"])         # 林媽媽 是 林爸爸 的 妻子
sp += t["son"](n["林爸爸"], n["林本人"])          # 林本人 是 林爸爸 的 兒子
sp += t["daughter"](n["林媽媽"], n["林妹妹"])     # 林妹妹 是 林媽媽 的 女兒
sp += t["husband"](n["林太太"], n["林本人"])      # 林本人 是 林太太 的 丈夫
sp += t["father"](n["林囡囡"], n["林本人"])       # 林本人 是 林囡囡 的 爸爸
sp += t["husband"](n["林妹妹"], n["男朋友"])      # 男朋友 是 林妹妹 的 丈夫
sp += t["mother"](n["林太太"], n["張媽媽"])       # 張媽媽 是 林太太 的 媽媽
sp += t["wife"](n["張爸爸"], n["張媽媽"])         # 張媽媽 是 張爸爸 的 妻子

for i in n:
    for j in n:
        if i == j:
            continue
        print_relationship(n[j], n[i], t, sp)
    print("")
