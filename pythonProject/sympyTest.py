from sympy import *

# 
def sympyTestFunction(x):
	y = integrate(x*ln(x), x);
	return y

#
x = Symbol('x');
y = sympyTestFunction(x);
print y
