/* CMSIS-DSP stub for host-side testing
 * Provides real math in plain C (no SIMD optimization) */
#ifndef ARM_MATH_STUB_H
#define ARM_MATH_STUB_H

#include <stdint.h>

/* ---- Status ---- */
typedef enum {
    ARM_MATH_SUCCESS        = 0,
    ARM_MATH_ARGUMENT_ERROR = -1,
    ARM_MATH_LENGTH_ERROR   = -2,
    ARM_MATH_SIZE_MISMATCH  = -3,
    ARM_MATH_NANINF         = -4,
    ARM_MATH_SINGULAR       = -5,
    ARM_MATH_TEST_FAILURE   = -6
} arm_status;

/* ---- Matrix instance ---- */
typedef struct {
    uint16_t numRows;
    uint16_t numCols;
    float    *pData;
} arm_matrix_instance_f32;

/* ---- Matrix functions ---- */
void       arm_mat_init_f32(arm_matrix_instance_f32 *S, uint16_t nRows,
                            uint16_t nCols, float *pData);
arm_status arm_mat_mult_f32(const arm_matrix_instance_f32 *pSrcA,
                            const arm_matrix_instance_f32 *pSrcB,
                            arm_matrix_instance_f32 *pDst);
arm_status arm_mat_add_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);
arm_status arm_mat_sub_f32(const arm_matrix_instance_f32 *pSrcA,
                           const arm_matrix_instance_f32 *pSrcB,
                           arm_matrix_instance_f32 *pDst);
arm_status arm_mat_trans_f32(const arm_matrix_instance_f32 *pSrc,
                             arm_matrix_instance_f32 *pDst);
arm_status arm_mat_inverse_f32(const arm_matrix_instance_f32 *pSrc,
                               arm_matrix_instance_f32 *pDst);
arm_status arm_mat_scale_f32(const arm_matrix_instance_f32 *pSrc,
                             float scale, arm_matrix_instance_f32 *pDst);

#endif /* ARM_MATH_STUB_H */
