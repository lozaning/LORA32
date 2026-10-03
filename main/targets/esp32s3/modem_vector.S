/* ESP32-S3 PIE IQ10 unpack and complex multiply. Unaligned-load pattern
 * adapted from eSpDR/ESPARGOS s3_unpack.S (GPL-3.0-or-later). */
    .section .iram1,"ax",@progbits
    .align 4
    .global modem_dechirp_sc16
    .type modem_dechirp_sc16,@function
/* aligned input, output, positive-phase chirp table, groups of 4 complex */
modem_dechirp_sc16:
    entry a1,32
    movi a8,15
    wsr.sar a8
    loopnez a5,.Ldechirp_end
    ee.vld.128.ip q0,a2,16
    ee.vld.128.ip q5,a4,16
    ee.cmul.s16 q1,q5,q0,2
    ee.cmul.s16 q1,q5,q0,3
    ee.vst.128.ip q1,a3,16
.Ldechirp_end:
    retw.n
    .size modem_dechirp_sc16,.-modem_dechirp_sc16

    .align 4
    .global lora_rotate_pack
    .type lora_rotate_pack,@function
/* dst IQ10 words, source sc16, repeated phase rotation sc16, groups of4
 * (caller supplies an even count). Process8 samples per SAR change. */
lora_rotate_pack:
    entry a1,32
    ee.vld.128.ip q5,a4,0
    movi a8,15
    movi a9,6
    movi a10,1023
    movi a11,1023
    slli a11,a11,10
    ee.movi.32.q q6,a10,0
    ee.movi.32.q q6,a10,1
    ee.movi.32.q q6,a10,2
    ee.movi.32.q q6,a10,3
    ee.movi.32.q q7,a11,0
    ee.movi.32.q q7,a11,1
    ee.movi.32.q q7,a11,2
    ee.movi.32.q q7,a11,3
    srli a5,a5,1
    loopnez a5,.Llora_pack_end
    ee.vld.128.ip q0,a3,16
    wsr.sar a8
    ee.cmul.s16 q1,q5,q0,0
    ee.cmul.s16 q1,q5,q0,1
    ee.vld.128.ip q4,a3,16
    ee.cmul.s16 q2,q5,q4,0
    ee.cmul.s16 q2,q5,q4,1
    ee.andq q3,q1,q6
    ee.andq q0,q2,q6
    wsr.sar a9
    ee.vsr.32 q4,q1
    ee.andq q4,q4,q7
    ee.orq q3,q3,q4
    ee.vsr.32 q1,q2
    ee.andq q1,q1,q7
    ee.orq q0,q0,q1
    ee.vst.128.ip q3,a2,16
    ee.vst.128.ip q0,a2,16
.Llora_pack_end:
    retw.n
    .size lora_rotate_pack,.-lora_rotate_pack

    .align 4
    .global modem_mix_iq10
    .type modem_mix_iq10,@function
/* src words, aligned output, 8-complex positive-phase mixer, groups of 8, conjugate output flag.
 * Reads at most 16 bytes beyond the last word; caller stays within the bank.
 * Output is decimated by four with a boxcar. IQ10 mixing is scaled by 8
 * so four-term int16 sums cannot saturate; the result is divided by 32. */
modem_mix_iq10:
    entry a1,32
    mov.n a11,a6
    movi a6,1023
    slli a6,a6,6
    slli a7,a6,16
    ee.movi.32.q q6,a6,0
    ee.movi.32.q q6,a6,1
    ee.movi.32.q q6,a6,2
    ee.movi.32.q q6,a6,3
    ee.movi.32.q q7,a7,0
    ee.movi.32.q q7,a7,1
    ee.movi.32.q q7,a7,2
    ee.movi.32.q q7,a7,3
    ee.vld.128.ip q5,a4,0
    ee.ld.128.usar.ip q0,a2,16
    loopnez a5,.Lmix_end
    ee.ld.128.usar.ip q1,a2,16
    ee.src.q q2,q0,q1
    ssai 6
    ee.vsl.32 q3,q2
    ssai 12
    ee.vsl.32 q4,q2
    ee.andq q3,q3,q6
    ee.andq q4,q4,q7
    ee.orq q3,q3,q4
    ssai 18
    ee.cmul.s16 q4,q5,q3,2
    ee.cmul.s16 q4,q5,q3,3
    movi a10,8
    wur.sar_byte a10
    ee.src.q q2,q4,q4
    ee.vadds.s16 q4,q4,q2
    movi a10,4
    wur.sar_byte a10
    ee.src.q q2,q4,q4
    ee.vadds.s16 q4,q4,q2
    ee.movi.32.a q4,a8,0
    srai a9,a8,21
    neg a10,a9
    movnez a9,a10,a11
    slli a8,a8,16
    srai a8,a8,21
    s16i a8,a3,0
    s16i a9,a3,2
    addi a3,a3,4
    ee.ld.128.usar.ip q0,a2,16
    ee.src.q q2,q1,q0
    ssai 6
    ee.vsl.32 q3,q2
    ssai 12
    ee.vsl.32 q4,q2
    ee.andq q3,q3,q6
    ee.andq q4,q4,q7
    ee.orq q3,q3,q4
    ssai 18
    ee.cmul.s16 q4,q5,q3,2
    ee.cmul.s16 q4,q5,q3,3
    movi a10,8
    wur.sar_byte a10
    ee.src.q q2,q4,q4
    ee.vadds.s16 q4,q4,q2
    movi a10,4
    wur.sar_byte a10
    ee.src.q q2,q4,q4
    ee.vadds.s16 q4,q4,q2
    ee.movi.32.a q4,a8,0
    srai a9,a8,21
    neg a10,a9
    moveqz a9,a10,a11
    slli a8,a8,16
    srai a8,a8,21
    neg a8,a8
    s16i a8,a3,0
    s16i a9,a3,2
    addi a3,a3,4
.Lmix_end:
    retw.n
    .size modem_mix_iq10,.-modem_mix_iq10
