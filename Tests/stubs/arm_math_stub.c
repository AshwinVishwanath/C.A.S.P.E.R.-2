/* CMSIS-DSP stub implementation — real math, plain C loops */
#include "arm_math_stub.h"
#include <string.h>

void arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows,
                      uint16_t nCols, float *pData)
{
    S->numRows = nRows;
    S->numCols = nCols;
    S->pData   = pData;
}

arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst)
{
    if (pSrcA->numCols != pSrcB->numRows)
        return ARM_MATH_SIZE_MISMATCH;

    uint16_t m = pSrcA->numRows;
    uint16_t n = pSrcB->numCols;
    uint16_t k = pSrcA->numCols;

    for (uint16_t i = 0; i < m; i++) {
        for (uint16_t j = 0; j < n; j++) {
            float sum = 0.0f;
            for (uint16_t p = 0; p < k; p++) {
                sum += pSrcA->pData[i * k + p] * pSrcB->pData[p * n + j];
            }
            pDst->pData[i * n + j] = sum;
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    uint32_t total = (uint32_t)pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < total; i++) {
        pDst->pData[i] = pSrcA->pData[i] + pSrcB->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst)
{
    if (pSrcA->numRows != pSrcB->numRows || pSrcA->numCols != pSrcB->numCols)
        return ARM_MATH_SIZE_MISMATCH;

    uint32_t total = (uint32_t)pSrcA->numRows * pSrcA->numCols;
    for (uint32_t i = 0; i < total; i++) {
        pDst->pData[i] = pSrcA->pData[i] - pSrcB->pData[i];
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst)
{
    uint16_t r = pSrc->numRows;
    uint16_t c = pSrc->numCols;

    for (uint16_t i = 0; i < r; i++) {
        for (uint16_t j = 0; j < c; j++) {
            pDst->pData[j * r + i] = pSrc->pData[i * c + j];
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst)
{
    /* Only supports up to 4x4 for EKF usage */
    uint16_t n = pSrc->numRows;
    if (n != pSrc->numCols || n > 4)
        return ARM_MATH_SIZE_MISMATCH;

    /* Gauss-Jordan elimination */
    float aug[4][8];
    memset(aug, 0, sizeof(aug));

    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            aug[i][j] = pSrc->pData[i * n + j];
        }
        aug[i][n + i] = 1.0f;
    }

    for (uint16_t i = 0; i < n; i++) {
        /* Find pivot */
        float max_val = 0.0f;
        uint16_t max_row = i;
        for (uint16_t k = i; k < n; k++) {
            float av = aug[k][i];
            if (av < 0) av = -av;
            if (av > max_val) {
                max_val = av;
                max_row = k;
            }
        }
        if (max_val < 1e-12f)
            return ARM_MATH_SINGULAR;

        /* Swap rows */
        if (max_row != i) {
            for (uint16_t j = 0; j < 2 * n; j++) {
                float tmp = aug[i][j];
                aug[i][j] = aug[max_row][j];
                aug[max_row][j] = tmp;
            }
        }

        /* Scale pivot row */
        float pivot = aug[i][i];
        for (uint16_t j = 0; j < 2 * n; j++) {
            aug[i][j] /= pivot;
        }

        /* Eliminate column */
        for (uint16_t k = 0; k < n; k++) {
            if (k == i) continue;
            float factor = aug[k][i];
            for (uint16_t j = 0; j < 2 * n; j++) {
                aug[k][j] -= factor * aug[i][j];
            }
        }
    }

    /* Copy result */
    for (uint16_t i = 0; i < n; i++) {
        for (uint16_t j = 0; j < n; j++) {
            pDst->pData[i * n + j] = aug[i][n + j];
        }
    }
    return ARM_MATH_SUCCESS;
}

arm_status arm_mat_scale_f32(const arm_matrix_instance_f32 *pSrc,
                             float scale, arm_matrix_instance_f32 *pDst)
{
    uint32_t total = (uint32_t)pSrc->numRows * pSrc->numCols;
    for (uint32_t i = 0; i < total; i++) {
        pDst->pData[i] = pSrc->pData[i] * scale;
    }
    return ARM_MATH_SUCCESS;
}
