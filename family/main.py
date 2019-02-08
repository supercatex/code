from problog.program import Term, Var, SimpleProgram
from problog.engine import DefaultEngine

import re

# verify the entry from the database
dbg_print = lambda xs: [print(k, v) for k, v in xs.items()]

# define a class to support read files and put the rows into dict()
class file_manager:
  def __init__(self, filename_):
    self.filename = filename_

  def get_dict(self):
    xs = dict()

    def fetch_lines(fd):
      for line in fd:
        yield line

    try:
      fd = open(self.filename, "r")
      rx = re.compile('([\n" "])')
      for line in fetch_lines(fd):
        data = rx.sub(r'', line).split("=")
        xs[data[0]] = Term(data[1])
      fd.close()
      # dbg_print(xs)
      return xs
    except Exception as err:
      print(err)

# define the rules
def create_rules(xs, ys):
  X, Y, Z = map(Var, ["X", "Y", "Z"])
  sp = SimpleProgram()
  sp += xs["myself"](X, Y) << xs["myself"](X, Y)
  sp += xs["husband"](X, Y) << xs["wife"](Y, X)
  sp += xs["wife"](X, Y) << xs["husband"](Y, X)
  sp += xs["couple"](X, Y) << (xs["husband"](X, Y) | xs["wife"](X, Y))
  sp += xs["couple"](X, Y) << xs["couple"](Y, X)
  sp += xs["son"](X, Y) << (xs["son"](X, Y) |
                            (xs["couple"](X, Z) & xs["son"](Z, Y)))
  sp += xs["daughter"](X, Y) << (xs["daughter"](X, Y) |
                                 (xs["couple"](X, Z) & xs["daughter"](Z, Y)))
  sp += xs["child"](X, Y) << (xs["son"](X, Y) | xs["daughter"](X, Y))
  sp += xs["child"](X, Y) << (xs["father"](Y, X) | xs["mother"](Y, X))
  sp += xs["child"](X, Y) << (xs["couple"](X, Z) & xs["child"](Z, Y))
  sp += xs["parent"](X, Y) << xs["child"](Y, X)
  sp += xs["mother"](X, Y) << (xs["wife"](Z, Y) & xs["child"](Y, X))
  sp += xs["father"](X, Y) << (xs["husband"](Z, Y) & xs["child"](Y, X))
  sp += xs["sibling"](X, Y) << (xs["parent"](X, Z) & xs["parent"](Y, Z))
  sp += xs["grandfather"](X, Y) << (xs["father"](X, Z) & xs["father"](Z, Y))
  sp += xs["grandmother"](X, Y) << (xs["father"](X, Z) & xs["mother"](Z, Y))
  sp += xs["grandparents"](
      X, Y) << (xs["grandfather"](X, Y) | xs["grandmother"](X, Y))
  sp += xs["grandson"](X, Y) << (xs["son"](X, Z) & xs["son"](Z, Y))
  sp += xs["granddaughter"](X, Y) << (xs["son"](X, Z) & xs["daughter"](Z, Y))
  sp += xs["grandchildren"](X, Y) << (xs["son"](X, Z) & xs["child"](Z, Y))
  sp += xs["grandfather_"](X, Y) << (xs["mother"](X, Z) & xs["father"](Z, Y))
  sp += xs["grandmother_"](X, Y) << (xs["mother"](X, Z) & xs["mother"](Z, Y))
  sp += xs["grandparents_"](
      X, Y) << (xs["grandfather_"](X, Y) | xs["grandmother_"](X, Y))
  sp += xs["grandson_"](X, Y) << (xs["daughter"](X, Z) & xs["son"](Z, Y))
  sp += xs["granddaughter_"](
      X, Y) << (xs["daughter"](X, Z) & xs["daughter"](Z, Y))
  sp += xs["grandchildren_"](X,
                             Y) << (xs["daughter"](X, Z) & xs["child"](Z, Y))
  sp += xs["son-wife"](X, Y) << (xs["husband"](Y, Z) & xs["parent"](Z, X))
  sp += xs["daughter-husband"](X, Y) << (xs["wife"](Y, Z) & xs["parent"](Z, X))
  sp += xs["daughter-husband"](X, Y) << (xs["wife"](Y, Z) & xs["parent"](Z, X))
  sp += xs["husband-mother"](X,
                             Y) << (xs["husband"](X, Z) & xs["mother"](Z, Y))
  sp += xs["husband-father"](X,
                             Y) << (xs["husband"](X, Z) & xs["father"](Z, Y))
  sp += xs["wife-mother"](X, Y) << (xs["wife"](X, Z) & xs["mother"](Z, Y))
  sp += xs["wife-father"](X, Y) << (xs["wife"](X, Z) & xs["father"](Z, Y))

  sp += xs["wife"](ys["林爸爸"], ys["林媽媽"])  # 林媽媽 是 林爸爸 的 妻子
  sp += xs["son"](ys["林爸爸"], ys["林本人"])  # 林本人 是 林爸爸 的 兒子
  sp += xs["daughter"](ys["林媽媽"], ys["林妹妹"])  # 林妹妹 是 林媽媽 的 女兒
  sp += xs["husband"](ys["林太太"], ys["林本人"])  # 林本人 是 林太太 的 丈夫
  sp += xs["father"](ys["林囡囡"], ys["林本人"])  # 林本人 是 林囡囡 的 爸爸
  sp += xs["husband"](ys["林妹妹"], ys["男朋友"])  # 男朋友 是 林妹妹 的 丈夫
  sp += xs["mother"](ys["林太太"], ys["張媽媽"])  # 張媽媽 是 林太太 的 媽媽
  sp += xs["wife"](ys["張爸爸"], ys["張媽媽"])  # 張媽媽 是 張爸爸 的 妻子
  return sp

# inference and show all possible result
def do_inference(a, b, terms, program):
  engine = DefaultEngine()
  xs = engine.prepare(program)

  print("[%s]是[%s]的 --> " % (b, a), end="")
  for key in terms:
    query_term = terms[key](a, b)
    res = engine.query(xs, query_term)
    if bool(res):
      print(terms[key], end=" ")
  print("")

# test suite
def test_inference():
  xs = file_manager("family_class.db").get_dict()
  ys = file_manager("family_instance.db").get_dict()
  sp = create_rules(xs, ys)
  ret = [do_inference(ys[j], ys[i], xs, sp) for i in ys for j in ys if i != j]
  assert None != ret

# entry point
if __name__ == '__main__':
  test_inference()
  print("")
