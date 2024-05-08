#ifndef GOLD_CODE_H
#define GOLD_CODE_H

#include <stdint.h>

typedef struct gold_code_struct {
    int16_t* data;
    int16_t length;
} gold_code_struct_t;

int generate_one_gold_sequence(
    uint8_t n, 
    gold_code_struct_t* output_code, 
    uint16_t sequence_number);

int generate_all_gold_sequences(
    uint8_t n, 
    gold_code_struct_t* output_codes);

int16_t sample_gold_code_with_idle_time(
    gold_code_struct_t *sampled_gold_code, 
    gold_code_struct_t *code, 
    double sample_rate, 
    double chip_duration, 
    double zeros_duration);

int16_t pad_then_sample_gold_code_with_idle_time( 
    gold_code_struct_t* padded_sampled_code,
    gold_code_struct_t* unpadded_code, 
    int pad_length, 
    double sample_rate, 
    double chip_duration, 
    double zeros_duration);

int16_t pad_gold_code_with_ones(
    gold_code_struct_t* padded_code,
    gold_code_struct_t* unpadded_code, 
    int target_length);

#endif // GOLD_CODE_H
