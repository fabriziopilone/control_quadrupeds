# quadprog: Quadratic Programming Solver

Forked from [`quadprog`](https://github.com/quadprog/quadprog) to create a ROS2 package.

## Usage
```c++
#include "quadprog/quadprog.hpp"

int result = solve_quadprog(
   G,
   g0,
   C,
   c0,
   x,
   m_eq [default = 0],
   factorized [default = 0]
);
```

Where the solution of the QP problem `x` is passed always as non-const reference.

`G`, `g0`, `C`, and `c0` can either be passed as non-const l-value references or as r-value references (if they are l-values use `std::move`).

```
Solve a strictly convex quadratic program

Minimize     1/2 x^T G x - g0^T x
Subject to   CE.T x - ce0  = 0
             CI.T x - ci0 >= 0

Where CE = C[:, :m_eq]
      CI = C[:, m_eq:]
      ce0 = c0[:meq]
      ci0 = c0[meq:]

This routine uses the the Goldfarb/Idnani dual algorithm [1].

References
---------
... [1] D. Goldfarb and A. Idnani (1983). A numerically stable dual
    method for solving strictly convex quadratic programs.
    Mathematical Programming, 27, 1-33.
```