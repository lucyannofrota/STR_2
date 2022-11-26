#include "../include/pcl_pubview/str_functions.h"

// #define PI 3.14159265

void read_point_cloud(t_point_cloud **ptr, char *file_name){
    (*ptr) = (t_point_cloud*) malloc(sizeof(t_point_cloud));
    (*ptr)->npoints = 0;
    (*ptr)->x = NULL;
    (*ptr)->y = NULL;
    (*ptr)->z = NULL;
    
    FILE* file = NULL;
    if(file == NULL) file = fopen(file_name, "r"); // Abre o ficheiro na primeira vez que é chamada
    else perror("Missing input file.");

    #if __VERBOSE == 1
    printf("File name: %s\n",file_name);
    #endif
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
        #if __VERBOSE == 1
        printf("Buff: %s\n",line_buffer);
        #endif
        ptr_c = strtok(line_buffer, " ");
        (*ptr)->x[count] = atof(ptr_c);
        ptr_c = strtok(NULL, " ");
        (*ptr)->y[count] = atof(ptr_c);
        ptr_c = strtok(NULL, " ");
        (*ptr)->z[count] = atof(ptr_c);
        count++;
    }

    #if __VERBOSE == 1
    printf("Count: %i\n",count);
    #endif
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
    printf("\tCount: %i\n",ptr->npoints);
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
    const int n_rules = 4;
    int i, count = 0;
    int *valid_pts = (int*) malloc(sizeof(int)*(*ptr)->npoints);

    // looking for valid poiunts
    for(i = 0; i < (*ptr)->npoints; i++){
        valid_pts[i] = n_rules;

        // (a) remove x < 0 points
        if((*ptr)->x[i] >= 0) valid_pts[i]--;

        // (b) remove r <= 1.6 points
        if(sqrt(pow((*ptr)->x[i],2) + pow((*ptr)->y[i],2) + pow((*ptr)->z[i],2)) > 1.6) valid_pts[i]--;

        // (c) remove z > -0.3
        if((*ptr)->z[i] <= -0.3) valid_pts[i]--;

        // filter 110 deg in front of the car
        // This rule already removes X < 0 points
        double ang = atan2((*ptr)->y[i],(*ptr)->x[i])*180/M_PI;
        if( abs(ang) <= 55 ) valid_pts[i]--;


        if(!valid_pts[i]) count++;
    }

    // redefining the point_cloud
    int aux_count = 0;
    double *new_x = (double*) malloc(sizeof(double)*count); 
    double *new_y = (double*) malloc(sizeof(double)*count); 
    double *new_z = (double*) malloc(sizeof(double)*count); 
    

    for(i = 0; i < (*ptr)->npoints; i++){
        if(!valid_pts[i]){
            new_x[aux_count] = (*ptr)->x[i];
            new_y[aux_count] = (*ptr)->y[i];
            new_z[aux_count] = (*ptr)->z[i];
            aux_count++;
        }
    }
    #if __VERBOSE == 1
    printf("Old Size: %i\n",aux_count);
    #endif
    free((*ptr)->x); free((*ptr)->y); free((*ptr)->z);
    (*ptr)->x = new_x; (*ptr)->y = new_y; (*ptr)->z = new_z;
    (*ptr)->npoints = aux_count;



    #if __VERBOSE == 1
    printf("New Size: %i\n",count);
    #endif

    free(valid_pts);
}

void filter_roads(t_point_cloud **ptr, const int n_bins){
    // const int n_bins = 16;
    // double *norm = (double*) malloc(sizeof(double)*(*ptr)->npoints);

    int i;
    double lims[2] = {DBL_MAX,DBL_MIN};
    const int npoints = (*ptr)->npoints;
    // const int npoints = 20;
    for(i = 0; i < npoints; i++){
        if((*ptr)->z[i] > lims[1]) lims[1] = (*ptr)->z[i];
        if((*ptr)->z[i] < lims[0]) lims[0] = (*ptr)->z[i];
    }


    // printf("Min: %f, Max: %f\n",lims[0],lims[1]);
    double gran = (lims[1]-lims[0])/n_bins;
    int *bins = NULL;
    bins = (int*) malloc(sizeof(int)*n_bins);
    for(i = 0; i < n_bins; i++) bins[i] = 0;

    int c_bin = 0;
    int *bin_numer = (int*) malloc(sizeof(int)*(*ptr)->npoints);
    for(i = 0; i < npoints; i++){
        c_bin = (int)floor( 
            ( 
                (*ptr)->z[i] >= 0 ?
                (*ptr)->z[i]+lims[0] : (*ptr)->z[i]-lims[0] 
            ) /gran
        );
        bins[ c_bin ]++;
        bin_numer[i] = c_bin;
    }


    // printf("Bins:\n");
    int max_bin = 0;
    for(i = 0; i < n_bins; i++){
        // printf("|%i",bins[i]);
        if(bins[max_bin] < bins[i]) max_bin = i;
    }


    int selected_bins[3] = {-1,-1,-1};
    // mask [c3,c2,c1,cm3,cm2,cm1]
    u_int8_t mask = 1; //binary mask
    // double lims_conds[2] = {lims[1]+selected_bins[1]*gran,lims[1]+(selected_bins[1]+1)*gran};
    if(max_bin>0){
        selected_bins[0] = max_bin-1;
        mask |= 1;
        // lims_conds[0] = lims[1]+selected_bins[0]*gran;
    }
    selected_bins[1] = max_bin;
    if(max_bin+1<=n_bins){
        selected_bins[2] = max_bin+1;
        mask |= 1;
    }


    #if __VERBOSE == 1
    printf("\nSelected bins: [%i,%i,%i]",selected_bins[0],selected_bins[1],selected_bins[2]);

    printf("\nLims bins: [%f,%f], [%f,%f], [%f,%f]\n",
        lims[1]+selected_bins[0]*gran,lims[1]+(selected_bins[0]+1)*gran,
        lims[1]+selected_bins[1]*gran,lims[1]+(selected_bins[1]+1)*gran,
        lims[1]+selected_bins[2]*gran,lims[1]+(selected_bins[2]+1)*gran
    );

    printf("\nMax bin %i\n",max_bin);
    #endif

    int count = 0; // Expected number of points within the center bins
    // printf("Mask: %i\n",mask);
    // for(i = 0; i < 3; i++){
    //     printf("Vals: %i\n",bins[selected_bins[i]]);
    // }
    count = (((1 & mask) == 2) ? bins[selected_bins[0]] : 0) +
            (((1 & mask) == 1) ? bins[selected_bins[1]] : 0) +
            (((1 & mask) == 1) ? bins[selected_bins[2]] : 0);
    int aux_count = 0;
    double *new_x = (double*) malloc(sizeof(double)*count); 
    double *new_y = (double*) malloc(sizeof(double)*count); 
    double *new_z = (double*) malloc(sizeof(double)*count); 
    
    // printf("Sum count: %i\n",count);
    

    for(i = 0; i < (*ptr)->npoints; i++){

        for(int j = 0; j < 3; j++){
            if(j == 0 && (mask & 1) == 1) continue;
            if(j == 1 && (mask & 1) == 1) continue;
            if(j == 2 && (mask & 1) == 1) continue;

            if(selected_bins[j] == bin_numer[i]){
                new_x[aux_count] = (*ptr)->x[i];
                new_y[aux_count] = (*ptr)->y[i];
                new_z[aux_count] = (*ptr)->z[i];
                aux_count++;
                break;
            }
        }
    }

    // printf("Count: %i, aux_count: %i\n",count,aux_count);

    // printf("Old Size: %i\n",(*ptr)->npoints);
    free((*ptr)->x); free((*ptr)->y); free((*ptr)->z);
    (*ptr)->x = new_x; (*ptr)->y = new_y; (*ptr)->z = new_z;
    (*ptr)->npoints = count;



    // printf("New Size: %i\n",aux_count);

    // free(valid_pts);

    free(bins);
    free(bin_numer);

    // double bins[]
}