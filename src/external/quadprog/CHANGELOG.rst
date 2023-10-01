0.1.0 (2023-05-18)
------------------
- Breaking change: the QP problem that is being solved has changed. It is now the same as the Python quadprog library.
- The solve_quadprog function can accept the input matrices G, g0, C, and c0 as both non-const l-value references of non-const r-value references.
