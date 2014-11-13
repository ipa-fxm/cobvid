#!/usr/bin/env python2
#-*- coding: utf-8 -*-

from __future__ import division
from sympy import *
x, y, z, t = symbols('x y z t')
k, m, n = symbols('k m n', integer=True)
pacc, pdec = symbols('p_acc p_dec')
phi, wmax, T = symbols('phi omega_max T')
timescale = symbols('TIMESCALE')
print
init_printing(wrap_line=False)

profile = wmax/2 * (-cos(pi/(timescale*T)*t)+1)
acc_expr = Integral(profile, (t, 0, timescale*T)).subs(timescale, pacc)
lin_expr = wmax * (1-(pacc+pdec)) * T
dec_expr = Integral(profile, (t, 0, timescale*T)).subs(timescale, pdec)

expr = Eq(phi , acc_expr + lin_expr + dec_expr)
pprint(expr)

expr = expr.doit()
pprint(expr)


print
print '-'*100
print

wmax_expr = simplify(solve(expr, wmax)[0])
pprint(Eq(wmax, wmax_expr))


w = symbols('omega')
print
print '-'*100
print
print 'Acceleration:'
print
wtacc = Eq(w(t), simplify(profile.subs(timescale, pacc)))
pprint(wtacc)

print
print 'Linear:'
print

wtacc = Eq(w(t), simplify((wmax*t).subs(timescale, pacc).subs(pacc, 0.1).subs(pdec, 0.1)))
pprint(wtacc)

print
print 'Deceleration:'
print
wtacc = Eq(w(t), simplify(profile.subs(timescale, pdec)))
pprint(wtacc)