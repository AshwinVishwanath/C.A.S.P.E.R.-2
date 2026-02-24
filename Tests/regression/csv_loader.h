#ifndef CSV_LOADER_H
#define CSV_LOADER_H

typedef struct {
    int col_count;
    char **col_names;
    int row_count;
    float **data;
} csv_table_t;

int csv_load(const char *path, csv_table_t *table);
int csv_find_col(const csv_table_t *table, const char *name);
void csv_free(csv_table_t *table);

#endif
