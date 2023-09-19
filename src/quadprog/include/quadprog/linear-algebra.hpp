#pragma once

void axpy(int n, double a, double x[], double y[]);
double dot(int n, double x[], double y[]);
void triangular_multiply(int n, double a[], double b[]);
void triangular_multiply_transpose(int n, double a[], double b[]);
void triangular_solve(int n, double a[], double b[]);
void triangular_solve_transpose(int n, double a[], double b[]);
void triangular_invert(int n, double a[]);
int cholesky(int n, double a[]);