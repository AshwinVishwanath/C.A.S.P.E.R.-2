/**
 * @file csv_loader.c
 * @brief CSV file loader for regression tests.
 *
 * Handles BOM (0xEF 0xBB 0xBF), \r\n line endings, trailing whitespace
 * in column names, and dynamic allocation for arbitrary-sized CSVs.
 */

#include "csv_loader.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define INITIAL_ROW_CAP 1024
#define LINE_BUF_SIZE   8192

/* Trim trailing whitespace in-place, return pointer to same buffer */
static char *trim_trailing(char *s)
{
    int len = (int)strlen(s);
    while (len > 0 && isspace((unsigned char)s[len - 1]))
        s[--len] = '\0';
    return s;
}

/* Trim leading whitespace, return pointer into same buffer */
static char *trim_leading(char *s)
{
    while (*s && isspace((unsigned char)*s))
        s++;
    return s;
}

/* Trim both ends. Returns pointer into original buffer (may be offset).
 * Modifies trailing end in place. */
static char *trim(char *s)
{
    s = trim_leading(s);
    trim_trailing(s);
    return s;
}

/* Count comma-separated fields in a line */
static int count_fields(const char *line)
{
    int count = 1;
    for (const char *p = line; *p; p++) {
        if (*p == ',')
            count++;
    }
    return count;
}

/* Parse header line into column names. Returns column count. */
static int parse_header(char *line, char ***names_out)
{
    int n = count_fields(line);
    char **names = (char **)malloc(n * sizeof(char *));
    if (!names)
        return -1;

    char *tok = line;
    for (int i = 0; i < n; i++) {
        char *comma = strchr(tok, ',');
        if (comma)
            *comma = '\0';

        char *trimmed = trim(tok);
        names[i] = strdup(trimmed);
        if (!names[i]) {
            for (int j = 0; j < i; j++)
                free(names[j]);
            free(names);
            return -1;
        }

        tok = comma ? comma + 1 : tok + strlen(tok);
    }

    *names_out = names;
    return n;
}

/* Parse a data row into float array. Returns 0 on success. */
static int parse_row(char *line, int col_count, float *row)
{
    char *tok = line;
    for (int i = 0; i < col_count; i++) {
        char *comma = strchr(tok, ',');
        if (comma)
            *comma = '\0';

        char *trimmed = trim(tok);
        char *end;
        row[i] = strtof(trimmed, &end);
        /* Allow empty or non-numeric fields to default to 0 */
        if (end == trimmed)
            row[i] = 0.0f;

        tok = comma ? comma + 1 : tok + strlen(tok);
    }
    return 0;
}

int csv_load(const char *path, csv_table_t *table)
{
    FILE *fp = fopen(path, "rb");
    if (!fp)
        return -1;

    memset(table, 0, sizeof(*table));

    char line[LINE_BUF_SIZE];

    /* Read first line (header), skip BOM if present */
    if (!fgets(line, sizeof(line), fp)) {
        fclose(fp);
        return -1;
    }

    /* Strip UTF-8 BOM (0xEF 0xBB 0xBF) */
    char *header = line;
    if ((unsigned char)header[0] == 0xEF &&
        (unsigned char)header[1] == 0xBB &&
        (unsigned char)header[2] == 0xBF) {
        header += 3;
    }

    /* Strip trailing \r\n */
    trim_trailing(header);

    /* Parse header */
    table->col_count = parse_header(header, &table->col_names);
    if (table->col_count <= 0) {
        fclose(fp);
        return -1;
    }

    /* Allocate row storage */
    int row_cap = INITIAL_ROW_CAP;
    table->data = (float **)malloc(row_cap * sizeof(float *));
    if (!table->data) {
        fclose(fp);
        return -1;
    }
    table->row_count = 0;

    /* Read data rows */
    while (fgets(line, sizeof(line), fp)) {
        trim_trailing(line);

        /* Skip empty lines */
        if (line[0] == '\0' || line[0] == '\r' || line[0] == '\n')
            continue;

        /* Grow row array if needed */
        if (table->row_count >= row_cap) {
            row_cap *= 2;
            float **new_data = (float **)realloc(table->data,
                                                  row_cap * sizeof(float *));
            if (!new_data) {
                fclose(fp);
                return -1;
            }
            table->data = new_data;
        }

        float *row = (float *)malloc(table->col_count * sizeof(float));
        if (!row) {
            fclose(fp);
            return -1;
        }

        parse_row(line, table->col_count, row);
        table->data[table->row_count++] = row;
    }

    fclose(fp);
    return 0;
}

int csv_find_col(const csv_table_t *table, const char *name)
{
    if (!table || !table->col_names || !name)
        return -1;

    /* Build a trimmed copy of the search name */
    char search[256];
    strncpy(search, name, sizeof(search) - 1);
    search[sizeof(search) - 1] = '\0';
    char *trimmed_search = trim(search);

    for (int i = 0; i < table->col_count; i++) {
        /* Column names are already trimmed at load time */
        if (strcmp(table->col_names[i], trimmed_search) == 0)
            return i;
    }
    return -1;
}

void csv_free(csv_table_t *table)
{
    if (!table)
        return;

    if (table->col_names) {
        for (int i = 0; i < table->col_count; i++)
            free(table->col_names[i]);
        free(table->col_names);
    }

    if (table->data) {
        for (int i = 0; i < table->row_count; i++)
            free(table->data[i]);
        free(table->data);
    }

    memset(table, 0, sizeof(*table));
}
