/* Aligned CSS symbol DSP only: Fs=BW, no packet sync, coding, or RF assumed. */
#pragma once
#include <math.h>
#define CSS_MAX_N 512u
#define CSS_PI 3.14159265358979323846f
typedef struct { float re, im; } css_complex_t;
static css_complex_t css_work[CSS_MAX_N];

static float css_phase(unsigned k, unsigned n) {
    return CSS_PI * ((float)k * (float)k / (float)n - (float)k);
}

static void css_modulate(int8_t *out, unsigned sf, unsigned symbol) {
    unsigned n=1u<<sf;
    for(unsigned k=0;k<n;k++) {
        unsigned shifted=(k+symbol)&(n-1u);
        float phase=css_phase(shifted,n);
        out[2*k]=(int8_t)lrintf(100.0f*cosf(phase));
        out[2*k+1]=(int8_t)lrintf(100.0f*sinf(phase));
    }
}

static unsigned css_demodulate(const int8_t *iq, unsigned sf, float *peak_fraction) {
    unsigned n=1u<<sf;
    for(unsigned k=0;k<n;k++) {
        float phase=css_phase(k,n),c=cosf(phase),s=sinf(phase);
        float re=iq[2*k],im=iq[2*k+1];
        css_work[k]=(css_complex_t){re*c+im*s,im*c-re*s};
    }
    for(unsigned i=1,j=0;i<n;i++) {
        unsigned bit=n>>1;
        for(;j&bit;bit>>=1)j^=bit;
        j^=bit;
        if(i<j){css_complex_t tmp=css_work[i];css_work[i]=css_work[j];css_work[j]=tmp;}
    }
    for(unsigned len=2;len<=n;len<<=1) {
        float angle=-2.0f*CSS_PI/(float)len;
        css_complex_t delta={cosf(angle),sinf(angle)};
        for(unsigned i=0;i<n;i+=len) {
            css_complex_t w={1,0};
            for(unsigned j=0;j<len/2;j++) {
                css_complex_t a=css_work[i+j],b=css_work[i+j+len/2];
                css_complex_t v={b.re*w.re-b.im*w.im,b.re*w.im+b.im*w.re};
                css_work[i+j]=(css_complex_t){a.re+v.re,a.im+v.im};
                css_work[i+j+len/2]=(css_complex_t){a.re-v.re,a.im-v.im};
                w=(css_complex_t){w.re*delta.re-w.im*delta.im,w.re*delta.im+w.im*delta.re};
            }
        }
    }
    unsigned best=0;float peak=0,total=0;
    for(unsigned i=0;i<n;i++) {
        float p=css_work[i].re*css_work[i].re+css_work[i].im*css_work[i].im;
        total+=p;if(p>peak){peak=p;best=i;}
    }
    *peak_fraction=total>0?peak/total:0;
    return best;
}
