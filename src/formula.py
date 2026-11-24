# -*- coding: utf-8 -*-
"""Small, safe C-expression evaluator used by the formula plot.

This is deliberately not a C compiler.  It implements the C expression
grammar subset that is useful for measurement formulas:

  * integer / floating constants (decimal, hex, octal, exponents, f/F suffix)
  * variables (provided by the caller with an int/float type)
  * + - * / %  (C integer division/remainder semantics)
  * < <= > >= == !=  && || !  & | ^ ~ << >>
  * ?: conditional (short circuit)
  * casts: (int), (long), (unsigned), (float), (double)
  * calls to the standard <math.h> functions (sqrt, sin, pow, ...)
  * M_PI, M_E, NAN, INFINITY constants

Assignments, arrays, pointers, statements and side effects are rejected.
Runtime domain errors produce NaN rather than raising.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

__all__ = ["FormulaError", "compile_expr"]


class FormulaError(Exception):
    pass


# ---------------------------------------------------------------------------
# Tokenizer
# ---------------------------------------------------------------------------
@dataclass
class Tok:
    kind: str          # NUM, IDENT, OP, EOF
    value: object
    pos: int


_NUM_RE = re.compile(
    r"""(?x)
    (0[xX][0-9a-fA-F]+[uUlL]*)                           # hex integer
    |((?:\d+\.\d*|\.\d+)(?:[eE][+-]?\d+)?[fFlL]?    # float with dot
      |\d+[eE][+-]?\d+[fFlL]?)                          # float with exponent
    |(\d+[uUlL]*)                                       # plain integer
    """
)
_MULTI_OPS = ("==", "!=", "<=", ">=", "&&", "||", "<<", ">>")
_SINGLE_OPS = set("+-*/%()?:!~&|^<>,;")
_IDENT_RE = re.compile(r"[A-Za-z_][A-Za-z0-9_]*")


def tokenize(src: str):
    toks = []
    i, n = 0, len(src)
    while i < n:
        ch = src[i]
        if ch in " \t\r\n":
            i += 1
            continue
        if src.startswith("//", i):
            j = src.find("\n", i)
            i = n if j < 0 else j + 1
            continue
        if src.startswith("/*", i):
            j = src.find("*/", i + 2)
            if j < 0:
                raise FormulaError(f"未闭合的注释（位置 {i}）")
            i = j + 2
            continue
        m = _NUM_RE.match(src, i)
        if m:
            text = m.group(0)
            if m.group(1):
                toks.append(Tok("NUM", int(m.group(1).rstrip("uUlL"), 16), i))
            elif m.group(2):
                body = text.rstrip("fFlL")
                toks.append(Tok("NUM", float(body), i))
            else:
                body = text.rstrip("uUlL")
                if len(body) > 1 and body[0] == "0" and all(c in "01234567" for c in body[1:]):
                    val = int(body[1:] or "0", 8)
                else:
                    val = int(body, 10)
                toks.append(Tok("NUM", val, i))
            i = m.end()
            continue
        m = _IDENT_RE.match(src, i)
        if m:
            toks.append(Tok("IDENT", m.group(0), i))
            i = m.end()
            continue
        op = next((o for o in _MULTI_OPS if src.startswith(o, i)), None)
        if op:
            toks.append(Tok("OP", op, i))
            i += 2
            continue
        if ch in _SINGLE_OPS:
            toks.append(Tok("OP", ch, i))
            i += 1
            continue
        raise FormulaError(f"无法识别的字符 {ch!r}（位置 {i}）")
    toks.append(Tok("EOF", None, n))
    return toks


# ---------------------------------------------------------------------------
# Expression nodes
# ---------------------------------------------------------------------------
class Node:
    typ = "float"

    def eval(self, ctx):
        raise NotImplementedError


class Const(Node):
    def __init__(self, value, typ):
        self.value = value
        self.typ = typ

    def eval(self, ctx):
        return self.value


class Var(Node):
    def __init__(self, name, typ):
        self.name = name
        self.typ = typ

    def eval(self, ctx):
        return ctx[self.name]


class Unary(Node):
    def __init__(self, op, node):
        self.op = op
        self.node = node
        if op in ("+", "-"):
            self.typ = node.typ
        elif op == "!":
            self.typ = "int"
        elif op == "~":
            if node.typ != "int":
                raise FormulaError("~ 只能用于整数")
            self.typ = "int"
        else:
            raise FormulaError(f"未知一元运算符 {op}")

    def eval(self, ctx):
        v = self.node.eval(ctx)
        if self.op == "+":
            return v
        if self.op == "-":
            return -v
        if self.op == "!":
            return 0 if _truth(v) else 1
        return ~int(v)


class Cast(Node):
    def __init__(self, node, target):
        self.node = node
        self.typ = target

    def eval(self, ctx):
        v = self.node.eval(ctx)
        if self.typ == "int":
            try:
                if isinstance(v, float) and not math.isfinite(v):
                    return 0
                return int(v)  # C truncates toward zero
            except Exception:
                return 0
        return float(v)


def _truth(v):
    try:
        return bool(v)
    except Exception:
        return False


def _c_int_div(a, b):
    if b == 0:
        return float("nan")
    q = abs(int(a)) // abs(int(b))
    return q if (a >= 0) == (b >= 0) else -q


def _c_rem(a, b):
    if b == 0:
        return float("nan")
    return a - _c_int_div(a, b) * b


def _float_div(a, b):
    a = float(a)
    b = float(b)
    if b == 0.0:
        if a == 0.0:
            return float("nan")
        return math.copysign(float("inf"), a) * (1.0 if math.copysign(1.0, b) > 0 else -1.0)
    return a / b


class Binary(Node):
    def __init__(self, op, left, right):
        self.op = op
        self.left = left
        self.right = right
        if op in ("+", "-", "*"):
            self.typ = "int" if (left.typ == right.typ == "int") else "float"
        elif op == "/":
            self.typ = "int" if (left.typ == right.typ == "int") else "float"
        elif op == "%":
            if left.typ != "int" or right.typ != "int":
                raise FormulaError("% 只能用于整数")
            self.typ = "int"
        elif op in ("<<", ">>", "&", "|", "^"):
            if left.typ != "int" or right.typ != "int":
                raise FormulaError(f"{op} 只能用于整数")
            self.typ = "int"
        elif op in ("<", "<=", ">", ">=", "==", "!="):
            self.typ = "int"
        else:
            raise FormulaError(f"未知二元运算符 {op}")

    def eval(self, ctx):
        a = self.left.eval(ctx)
        b = self.right.eval(ctx)
        op = self.op
        if op == "+":
            return a + b
        if op == "-":
            return a - b
        if op == "*":
            return a * b
        if op == "/":
            return _c_int_div(a, b) if self.typ == "int" else _float_div(a, b)
        if op == "%":
            return _c_rem(a, b)
        if op == "<<":
            try:
                return int(a) << int(b)
            except Exception:
                return 0
        if op == ">>":
            try:
                return int(a) >> int(b)
            except Exception:
                return 0
        if op == "&":
            return int(a) & int(b)
        if op == "|":
            return int(a) | int(b)
        if op == "^":
            return int(a) ^ int(b)
        if op == "<":
            return 1 if a < b else 0
        if op == "<=":
            return 1 if a <= b else 0
        if op == ">":
            return 1 if a > b else 0
        if op == ">=":
            return 1 if a >= b else 0
        if op == "==":
            return 1 if a == b else 0
        if op == "!=":
            return 1 if a != b else 0
        raise FormulaError(f"未知运算符 {op}")


class Logical(Node):
    typ = "int"

    def __init__(self, op, left, right):
        self.op = op
        self.left = left
        self.right = right

    def eval(self, ctx):
        if self.op == "&&":
            return 1 if (_truth(self.left.eval(ctx)) and _truth(self.right.eval(ctx))) else 0
        return 1 if (_truth(self.left.eval(ctx)) or _truth(self.right.eval(ctx))) else 0


class Conditional(Node):
    def __init__(self, cond, a, b):
        self.cond = cond
        self.a = a
        self.b = b
        self.typ = "int" if (a.typ == b.typ == "int") else "float"

    def eval(self, ctx):
        v = self.a.eval(ctx) if _truth(self.cond.eval(ctx)) else self.b.eval(ctx)
        return int(v) if self.typ == "int" else float(v)


# Math functions.  All are treated as returning double, like <math.h>.
def _c_round(x):
    x = float(x)
    if math.isnan(x) or math.isinf(x):
        return x
    return math.floor(x + 0.5) if x >= 0 else math.ceil(x - 0.5)


def _c_fmin(a, b):
    a, b = float(a), float(b)
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    return min(a, b)


def _c_fmax(a, b):
    a, b = float(a), float(b)
    if math.isnan(a):
        return b
    if math.isnan(b):
        return a
    return max(a, b)


_MATH1 = {
    "fabs": math.fabs, "sqrt": math.sqrt, "sin": math.sin, "cos": math.cos,
    "tan": math.tan, "asin": math.asin, "acos": math.acos, "atan": math.atan,
    "exp": math.exp, "log": math.log, "log10": math.log10,
    "floor": math.floor, "ceil": math.ceil, "round": _c_round,
    "trunc": math.trunc, "sinh": math.sinh, "cosh": math.cosh, "tanh": math.tanh,
}
_MATH2 = {
    "pow": math.pow, "atan2": math.atan2, "fmod": math.fmod,
    "fmin": _c_fmin, "fmax": _c_fmax, "hypot": math.hypot,
}


def _call_math(name, args):
    try:
        if name in _MATH1:
            return float(_MATH1[name](float(args[0])))
        if name in _MATH2:
            return float(_MATH2[name](float(args[0]), float(args[1])))
    except (ValueError, OverflowError, ZeroDivisionError):
        return float("nan")
    except Exception:
        return float("nan")
    raise FormulaError(f"未知函数 {name}")


class Call(Node):
    typ = "float"

    def __init__(self, name, args):
        self.name = name
        self.args = args
        if name in _MATH1:
            if len(args) != 1:
                raise FormulaError(f"{name} 需要 1 个参数")
        elif name in _MATH2:
            if len(args) != 2:
                raise FormulaError(f"{name} 需要 2 个参数")
        else:
            raise FormulaError(f"不支持的函数 {name}")

    def eval(self, ctx):
        vals = [arg.eval(ctx) for arg in self.args]
        return _call_math(self.name, vals)


# ---------------------------------------------------------------------------
# Parser
# ---------------------------------------------------------------------------
_CONSTANTS = {
    "M_PI": (math.pi, "float"),
    "M_E": (math.e, "float"),
    "NAN": (float("nan"), "float"),
    "INFINITY": (float("inf"), "float"),
}
_TYPE_CASTS = {
    "int": "int", "short": "int", "long": "int", "char": "int",
    "unsigned": "int", "signed": "int",
    "float": "float", "double": "float",
}


class Parser:
    def __init__(self, src: str, variables: dict):
        self.src = src
        self.toks = tokenize(src)
        self.i = 0
        self.variables = dict(variables)

    def peek(self):
        return self.toks[self.i]

    def at_op(self, value):
        t = self.peek()
        return t.kind == "OP" and t.value == value

    def take_op(self, value):
        if not self.at_op(value):
            t = self.peek()
            raise FormulaError(f"期望 {value!r}，位置 {t.pos}")
        self.i += 1

    def advance(self):
        t = self.toks[self.i]
        self.i += 1
        return t

    def parse(self):
        if self.peek().kind == "EOF":
            raise FormulaError("公式为空")
        node = self.parse_conditional()
        t = self.peek()
        if t.kind != "EOF":
            raise FormulaError(f"公式末尾有多余内容（位置 {t.pos}）")
        return node

    # -- grammar levels ---------------------------------------------------
    def parse_conditional(self):
        cond = self.parse_logical_or()
        if self.at_op("?"):
            self.i += 1
            a = self.parse_conditional()
            self.take_op(":")
            b = self.parse_conditional()
            return Conditional(cond, a, b)
        return cond

    def parse_logical_or(self):
        node = self.parse_logical_and()
        while self.at_op("||"):
            self.i += 1
            node = Logical("||", node, self.parse_logical_and())
        return node

    def parse_logical_and(self):
        node = self.parse_bit_or()
        while self.at_op("&&"):
            self.i += 1
            node = Logical("&&", node, self.parse_bit_or())
        return node

    def parse_bit_or(self):
        node = self.parse_bit_xor()
        while self.at_op("|"):
            self.i += 1
            node = Binary("|", node, self.parse_bit_xor())
        return node

    def parse_bit_xor(self):
        node = self.parse_bit_and()
        while self.at_op("^"):
            self.i += 1
            node = Binary("^", node, self.parse_bit_and())
        return node

    def parse_bit_and(self):
        node = self.parse_equality()
        while self.at_op("&"):
            self.i += 1
            node = Binary("&", node, self.parse_equality())
        return node

    def parse_equality(self):
        node = self.parse_relational()
        while self.peek().kind == "OP" and self.peek().value in ("==", "!="):
            op = self.advance().value
            node = Binary(op, node, self.parse_relational())
        return node

    def parse_relational(self):
        node = self.parse_shift()
        while self.peek().kind == "OP" and self.peek().value in ("<", "<=", ">", ">="):
            op = self.advance().value
            node = Binary(op, node, self.parse_shift())
        return node

    def parse_shift(self):
        node = self.parse_add()
        while self.peek().kind == "OP" and self.peek().value in ("<<", ">>"):
            op = self.advance().value
            node = Binary(op, node, self.parse_add())
        return node

    def parse_add(self):
        node = self.parse_mul()
        while self.peek().kind == "OP" and self.peek().value in ("+", "-"):
            op = self.advance().value
            node = Binary(op, node, self.parse_mul())
        return node

    def parse_mul(self):
        node = self.parse_unary()
        while self.peek().kind == "OP" and self.peek().value in ("*", "/", "%"):
            op = self.advance().value
            node = Binary(op, node, self.parse_unary())
        return node

    def parse_unary(self):
        t = self.peek()
        if t.kind == "OP" and t.value in ("+", "-", "!", "~"):
            self.i += 1
            return Unary(t.value, self.parse_unary())
        # C-style cast: "(" type ")" unary
        if t.kind == "OP" and t.value == "(":
            if (self.toks[self.i + 1].kind == "IDENT" and
                    self.toks[self.i + 1].value in _TYPE_CASTS and
                    self.toks[self.i + 2].kind == "OP" and
                    self.toks[self.i + 2].value == ")"):
                target = _TYPE_CASTS[self.toks[self.i + 1].value]
                self.i += 3
                return Cast(self.parse_unary(), target)
        return self.parse_primary()

    def parse_primary(self):
        t = self.peek()
        if t.kind == "NUM":
            self.i += 1
            return Const(t.value, "int" if isinstance(t.value, int) else "float")
        if t.kind == "IDENT":
            self.i += 1
            name = t.value
            if self.at_op("("):
                self.i += 1
                args = []
                if not self.at_op(")"):
                    while True:
                        args.append(self.parse_conditional())
                        if self.at_op(","):
                            self.i += 1
                            continue
                        break
                self.take_op(")")
                return Call(name, args)
            if name in _CONSTANTS:
                return Const(*_CONSTANTS[name])
            if name in self.variables:
                return Var(name, self.variables[name])
            raise FormulaError(f"未知变量 {name!r}")
        if t.kind == "OP" and t.value == "(":
            self.i += 1
            node = self.parse_conditional()
            self.take_op(")")
            return node
        raise FormulaError(f"表达式不完整（位置 {t.pos}）")


class CompiledExpr:
    def __init__(self, text, variables, root):
        self.text = text
        self.variables = dict(variables)
        self.root = root
        self.typ = root.typ

    def eval(self, ctx):
        try:
            v = self.root.eval(ctx)
            if isinstance(v, bool):
                return int(v)
            return v
        except Exception:
            return float("nan")

    def eval_float(self, ctx):
        v = self.eval(ctx)
        try:
            return float(v)
        except Exception:
            return float("nan")


def compile_expr(text: str, variables: dict) -> CompiledExpr:
    """Compile a C-like expression.  variables maps name -> 'int'/'float'."""
    if not isinstance(text, str) or not text.strip():
        raise FormulaError("公式为空")
    for name, typ in variables.items():
        if typ not in ("int", "float"):
            raise FormulaError(f"变量 {name} 的类型必须是 int/float")
    parser = Parser(text, variables)
    root = parser.parse()
    return CompiledExpr(text, variables, root)
