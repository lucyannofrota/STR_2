#include "str_functions.h"

#define MAX_LINE_LEN (3-DBL_MIN_10_EXP+1)*3+10 // Tamanho maximo que uma linha pode ter. Assumindo 3xDLOUBLE + 10.
//(strlen("-0.")-DBL_MIN_10_EXP+1) Sendo o maior valor que uma variavel double pode assumir em ascii

void read_point_cloud(t_point_cloud **ptr, char *file_name){
    // printf("File name: %s\n",file);
    // t_point_cloud *ret_ptr = (t_point_cloud*) malloc(sizeof(t_point_cloud));
    // (*ptr)->npoints
    (*ptr) = (t_point_cloud*) malloc(sizeof(t_point_cloud));
    (*ptr)->npoints = 0;
    (*ptr)->x = NULL;
    (*ptr)->y = NULL;
    (*ptr)->z = NULL;
    
    FILE* file = NULL;
    if(file == NULL) file = fopen(file_name, "r"); // Abre o ficheiro na primeira vez que é chamada
    else perror("Missing input file.");

    printf("File name: %s\n",file_name);
    int count = 0;
    char line_buffer[MAX_LINE_LEN];
    while(fgets(line_buffer,sizeof(line_buffer),file) != NULL) count++;
    fseek(file, 0, SEEK_SET);
    (*ptr)->npoints = count;
    (*ptr)->x = (double*) malloc(sizeof(double)*(*ptr)->npoints);
    (*ptr)->y = (double*) malloc(sizeof(double)*(*ptr)->npoints);
    (*ptr)->z = (double*) malloc(sizeof(double)*(*ptr)->npoints);
    char *ptr_c;
    count = 0;
    while(fgets(line_buffer,sizeof(line_buffer),file) != NULL){
        printf("Buff: %s\n",line_buffer);
        ptr_c = strtok(line_buffer, " ");
        (*ptr)->x[count] = atof(ptr_c);
        ptr_c = strtok(NULL, " ");
        (*ptr)->y[count] = atof(ptr_c);
        ptr_c = strtok(NULL, " ");
        (*ptr)->z[count] = atof(ptr_c);
        count++;
    }

    printf("Count: %i\n",count);
    fclose(file);
    // return ret_ptr;
}

void describe_point_cloud(t_point_cloud *ptr){
    double max[3] = {ptr->x[0],ptr->y[0],ptr->z[0]}, min[3] = {ptr->x[0],ptr->y[0],ptr->z[0]}, mean[3] = {0,0,0}, std[3] = {0,0,0};

    for(int i = 0; i < ptr->npoints; i++){
        // printf("Pt: [%10.6f|%10.6f|%10.6f]\n",ptr->x[i],ptr->y[i],ptr->z[i]);
        // Max
        if(ptr->x[i] > max[0]) max[0] = ptr->x[i];
        if(ptr->y[i] > max[1]) max[1] = ptr->y[i];
        if(ptr->z[i] > max[2]) max[2] = ptr->z[i];
        // Min
        if(ptr->x[i] < min[0]) min[0] = ptr->x[i];
        if(ptr->y[i] < min[1]) min[1] = ptr->y[i];
        if(ptr->z[i] < min[2]) min[2] = ptr->z[i];
        // Mean
        mean[0] += ptr->x[i];
        mean[1] += ptr->y[i];
        mean[2] += ptr->z[i];
    }
    mean[0] /= ptr->npoints;
    mean[1] /= ptr->npoints;
    mean[2] /= ptr->npoints;


    for(int i = 0; i < ptr->npoints; i++){
        std[0] = pow(ptr->x[i] - mean[0],2);
        std[1] = pow(ptr->y[i] - mean[1],2);
        std[2] = pow(ptr->z[i] - mean[2],2);
    }
    std[0] /= ptr->npoints;
    std[1] /= ptr->npoints;
    std[2] /= ptr->npoints;
    std[0] = sqrt(std[0]);
    std[1] = sqrt(std[1]);
    std[2] = sqrt(std[2]);



    printf("\n\nReport:");
    printf("\t      [         x,         y,         z]\n");
    printf("\tMax:  [%10.6f,%10.6f,%10.6f]\n",max[0],max[1],max[2]);
    printf("\tMin:  [%10.6f,%10.6f,%10.6f]\n",min[0],min[1],min[2]);
    printf("\tMean: [%10.6f,%10.6f,%10.6f]\n",mean[0],mean[1],mean[2]);
    printf("\tSTD:  [%10.6f,%10.6f,%10.6f]\n",std[0],std[1],std[2]);
}

void free_t_point_cloud(t_point_cloud *ptr){
    free(ptr->x);
    free(ptr->y);
    free(ptr->z);
    free(ptr);
}

void filter_point_cloud(t_point_cloud **ptr){
    const int n_rules = 3;
    int i, count = 0;
    int *valid_pts = (int*) malloc(sizeof(int)*(*ptr)->npoints);

    // looking for valid poiunts
    for(i = 0; i < (*ptr)->npoints; i++){
        valid_pts[i] = n_rules;

        // (a) remove x < 0 points
        if((*ptr)->x[i] >= 0) valid_pts[i]--;

        // (b) remove r <= 1.6 points
        if(sqrt(pow((*ptr)->x[i],2) + pow((*ptr)->y[i],2) + pow((*ptr)->z[i],2)) > 1.6) valid_pts[i]--;

        // (c) remove z < 0 points
        if((*ptr)->z[i] <= -0.3) valid_pts[i]--;


        if(!valid_pts[i]) count++;
    }

    // redefining the point_cloud
    int aux_count = 0;
    // double *aux_x, *aux_y, *aux_z;
    double *new_x = (double*) malloc(sizeof(double)*count); 
    double *new_y = (double*) malloc(sizeof(double)*count); 
    double *new_z = (double*) malloc(sizeof(double)*count); 
    
    // printf("ptr: %p\n",new_x);
    // printf("ptr: %p\n",new_y);
    // printf("ptr: %p\n",new_z);

    for(i = 0; i < (*ptr)->npoints; i++){
        if(!valid_pts[i]){
            new_x[aux_count] = (*ptr)->x[i];
            new_y[aux_count] = (*ptr)->y[i];
            new_z[aux_count] = (*ptr)->z[i];
            // printf("PT+: [%f,%f,%f]\n",new_x[aux_count],new_y[aux_count],new_z[aux_count]);
            // printf("PT_: [%f,%f,%f]\n",(*ptr)->x[aux_count],(*ptr)->y[aux_count],(*ptr)->z[aux_count]);
            aux_count++;
        }
    }
    printf("Old Size: %i\n",aux_count);
    (*ptr)->npoints = aux_count;
    free((*ptr)->x); free((*ptr)->y); free((*ptr)->z);
    (*ptr)->x = new_x; (*ptr)->y = new_y; (*ptr)->z = new_z;


    // for(i = 0; i < (*ptr)->npoints; i++){
    //   printf("PT+: [%f,%f,%f]\n",new_x[i],new_y[i],new_z[i]);
    //   printf("PT_: [%f,%f,%f]\n",(*ptr)->x[i],(*ptr)->y[i],(*ptr)->z[i]);
    // }



    printf("New Size: %i\n",count);

    free(valid_pts);
}

