import casadi

opti = casadi.Opti()
opti.solver("ipopt")

sol = opti.solve()
print(type(sol))
print(type(opti.variable()))