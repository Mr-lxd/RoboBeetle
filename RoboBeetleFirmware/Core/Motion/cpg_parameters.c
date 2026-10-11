#include "cpg_parameters.h"
#include <math.h>
#include <string.h>
#include <float.h>
_Static_assert(sizeof(double)==8 && DBL_MANT_DIG==53 && DBL_MAX_EXP==1024,
               "CPG wire format requires IEEE754 binary64");
void cpg_parameters_default(cpg_parameters_t *p) {
    *p=(cpg_parameters_t){10.0,10.0,2.5162,.75,0.0,0.0,2.0,0x3c};
}
cpg_parameters_result_t cpg_parameters_validate(const cpg_parameters_t *p) {
    if (!p || (p->mask & 0xc0)) return CPG_PARAMETERS_INVALID_PAYLOAD;
    const double v[]={p->front_amp,p->rear_amp,p->period,p->beta,p->F,p->L,p->w};
    const double lo[]={0,0,.5,.1,-180,-180,0}, hi[]={28,30,10,.9,180,180,5};
    for (unsigned i=0;i<7;++i)
        if (!isfinite(v[i]) || v[i]<lo[i] || v[i]>hi[i]) return CPG_PARAMETERS_OUT_OF_RANGE;
    return CPG_PARAMETERS_OK;
}
size_t cpg_parameters_encode(const cpg_parameters_t *p,uint8_t *wire,size_t capacity) {
    if (!p || !wire || capacity<58) return 0;
    const double v[]={p->front_amp,p->rear_amp,p->period,p->beta,p->F,p->L,p->w};
    wire[0]=1; wire[1]=p->mask;
    for(unsigned i=0;i<7;++i) { uint64_t bits; memcpy(&bits,&v[i],8);
        for(unsigned j=0;j<8;++j) wire[2+8*i+j]=(uint8_t)(bits>>(8*j)); }
    return 58;
}
cpg_parameters_result_t cpg_parameters_decode(const uint8_t *wire,size_t length,cpg_parameters_t *p) {
    if (!wire || !p || length!=58 || wire[0]!=1 || (wire[1]&0xc0)) return CPG_PARAMETERS_INVALID_PAYLOAD;
    cpg_parameters_t candidate={0};
    double *v[]={&candidate.front_amp,&candidate.rear_amp,&candidate.period,&candidate.beta,&candidate.F,&candidate.L,&candidate.w};
    candidate.mask=wire[1];
    for(unsigned i=0;i<7;++i) { uint64_t bits=0;
        for(unsigned j=0;j<8;++j) bits|=(uint64_t)wire[2+8*i+j]<<(8*j);
        memcpy(v[i],&bits,8); }
    cpg_parameters_result_t result=cpg_parameters_validate(&candidate);
    if(result==CPG_PARAMETERS_OK) *p=candidate;
    return result;
}
bool cpg_parameters_equal(const cpg_parameters_t *a,const cpg_parameters_t *b) {
    uint8_t wa[58],wb[58];
    return cpg_parameters_encode(a,wa,58)==58 && cpg_parameters_encode(b,wb,58)==58 && memcmp(wa,wb,58)==0;
}
