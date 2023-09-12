/* 
A task T is a set of linear equality and/or inequality constraints on x:
    A*x - b = w
T:{
    D*x - f <= v
with w, v slack variables to be minimized

If we have more tasks T1, T2, ..., Tn to be solved in a prioritized order, we need to find the solution of the 
(k+1)-th tasks, x_(k+1), in the null space of all higher priority equality constraints: Zp = Null(A_bar_p)
with A_bar_p = [A1' ...  Ap']'
from which x = x* + Zp * z_(p+1), with z_(p+1) a vector living the row space of Zp.

Solving for a new task T_(p+1) means computing z*_(p+1) and v*_(p+1) from the QP problem:
min_( z_(p+1), v_(p+1) ) 0.5*|| A_(p+1)*(x* + Zp*z_(p+1) - b_(p+1) ||^2 + 0.5*|| v_(p+1)||^2
subject to
D_(p+1)*(x* + Zp*z_(p+1) ) - f_(p+1) <= v_(p+1)
D_p*(x* +Zp*z_(p+1) ) - f_p <= v*_p
...
D_1*(x* + Zp*z_(p+1) ) - f_1 <= v*_1
v_(p+1) >= 0

Writing xi_p = [z_p; v_p] we can write
min_xi_(p+1) 0.5*xi_(p+1)' * H_(p+1) * xi_(p+1) + c_(p+1)' * xi_(p+1)
subject to
D_(p+1)^hat * xi_(p+1) <= f_(p+1)^hat
with
H_(p+1) = [Zp' * A_(p+1)' * A_(p+1) * Zp,     0;]
          [               0,                  I ]

c_(p+1) = [Zp' * A_(p+1)' * ( A_(p+1) * x* - b_(p+1) );]
          [               0                            ]

D_(p+1)^hat = [D_(p+1)*Zp, -eye();]
              [D_p*Zp,         0; ]
              [          ...      ]
              [D_1*Zp,         0; ]
              [   0,          -I  ]

f_(p+1)^hat = [ f_(p+1) - D_(p+1)*x*; ]
              [ f_p - D_p*x* + v_(p)*;]
              [           ...         ]
              [f_1 - D_1*x* + v_(1)*; ]
              [            0          ]

Iteratively
Z_2 = Null(A_1)*Null( A_2*Null(A_1) ) = Z_1 * Null( A_2 * Z_1 )

*/

