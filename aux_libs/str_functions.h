#ifndef _STR_FUNCTIONS_H__
#define _STR_FUNCTIONS_H__

#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <string.h>
#include <math.h>

typedef struct {
    double *x, *y, *z;
    int npoints;
} t_point_cloud;

void read_point_cloud(t_point_cloud **ptr, char *file_name);

void describe_point_cloud(t_point_cloud *ptr);

void free_t_point_cloud(t_point_cloud *ptr);

void filter_point_cloud(t_point_cloud **ptr);

#endif //_STR_FUNCTIONS_H__