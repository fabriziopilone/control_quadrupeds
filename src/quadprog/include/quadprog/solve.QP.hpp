#pragma once

#include "quadprog/qr-update.hpp"
#include "quadprog/linear-algebra.hpp"

int qpgen2_(double *G, double *av, int n,
    double *xv, double *lagr, double *obj,
    double *C, double *bv, int q, int meq,
    int *iact, int *nact, int *iter,
    double *work, int factorized);