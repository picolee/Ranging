/*
 * gold_code.c
 *
 *  Created on: Apr 20, 2024
 *      Author: LeeLemay
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "inc/gold_code.h"

// Defined out to 11 bits - use the first N
int16_t seed[]    = {0, 1, 1, 0, 0, 0, 1, 0, 1, 0, 1};

// Tap definitions for the Linear Feedback Shift Registers
typedef struct {
    int16_t* taps1;
    uint8_t taps1_length;
    int16_t* taps2;
    uint8_t taps2_length;
} TapConfig;

// Definitions for N = 5
int16_t taps_for_5_1[] = {2};
int16_t taps_for_5_2[] = {1, 2, 3};
TapConfig taps_for_five = {
    .taps1 = taps_for_5_1,
    .taps1_length = 1,
    .taps2 = taps_for_5_2,
    .taps2_length = 3
};

// Definitions for N = 6
int16_t taps_for_6_1[] = {5};
int16_t taps_for_6_2[] = {1, 4, 5};
TapConfig taps_for_six = {
    .taps1 = taps_for_6_1,
    .taps1_length = 1,
    .taps2 = taps_for_6_2,
    .taps2_length = 3
};

// Definitions for N = 7
int16_t taps_for_7_1[] = {4};
int16_t taps_for_7_2[] = {4, 5, 6};
TapConfig taps_for_seven = {
    .taps1 = taps_for_7_1,
    .taps1_length = 1,
    .taps2 = taps_for_7_2,
    .taps2_length = 3
};

// Definitions for N = 8
int16_t taps_for_8_1[] = {1, 2, 3, 6, 7};
int16_t taps_for_8_2[] = {1, 2, 7};
TapConfig taps_for_eight = {
    .taps1 = taps_for_8_1,
    .taps1_length = 5,
    .taps2 = taps_for_8_2,
    .taps2_length = 3
};

// Definitions for N = 9
int16_t taps_for_9_1[] = {5};
int16_t taps_for_9_2[] = {3, 5, 6};
TapConfig taps_for_nine = {
    .taps1 = taps_for_9_1,
    .taps1_length = 1,
    .taps2 = taps_for_9_2,
    .taps2_length = 3
};

// Definitions for N = 10
int16_t taps_for_10_1[] = {2, 5, 9};
int16_t taps_for_10_2[] = {3, 4, 6, 8, 9};
TapConfig taps_for_ten = {
    .taps1 = taps_for_10_1,
    .taps1_length = 3,
    .taps2 = taps_for_10_2,
    .taps2_length = 5
};

// Definitions for N = 11
int16_t taps_for_11_1[] = {9};
int16_t taps_for_11_2[] = {3, 6, 9};
TapConfig taps_for_eleven = {
    .taps1 = taps_for_11_1,
    .taps1_length = 1,
    .taps2 = taps_for_11_2,
    .taps2_length = 3
};


// Global array holding configurations for different sequence lengths
TapConfig* preferred_pairs[] = 
{
    &taps_for_five,  // Corresponds to N = 5
    &taps_for_six,   // Corresponds to N = 6
    &taps_for_seven, // Corresponds to N = 7
    &taps_for_eight, // Corresponds to N = 8
    &taps_for_nine,  // Corresponds to N = 9
    &taps_for_ten,   // Corresponds to N = 10
    &taps_for_eleven // Corresponds to N = 11
};

void xor_sequences(int16_t* seq1, int16_t* seq2, int16_t* out, size_t n) 
{
    uint32_t i;
    for ( i = 0; i < n; i++)
    {
        out[i] = seq1[i] ^ seq2[i];
    }
}

void roll_sequence_left(int16_t* input, int16_t* output, int n, int shift) 
{
    uint32_t i;
    for ( i = 0; i < n; i++)
    {
        output[i] = input[(i + shift) % n];
    }
}

void roll_sequence_right(int16_t* input, int16_t* output, int n, int shift) 
{
    uint32_t i;
    for ( i = 0; i < n; i++)
    {
        // Calculate new index for left shift
        int new_index = (i - shift + n) % n;
        output[i] = input[new_index];
    }
}

void roll_sequence(int16_t* input, int16_t* output, int n, int shift) 
{
    if(shift < 0) 
    {
        roll_sequence_left(input, output, n, -shift);
    } 
    else 
    {
        roll_sequence_right(input, output, n, shift);
    }
}

int16_t linear_feedback_shift_register(
    int16_t* taps, 
    uint8_t taps_length, 
    int16_t* buf, 
    uint8_t nbits, 
    int16_t* out)
{
    uint32_t index;
    uint8_t taps_index;
    uint8_t roll_index;
    int16_t feedback;
    int16_t* shift_register = (int16_t*)malloc(nbits * sizeof(int16_t));

    if (shift_register == NULL) {
        return 1;
    }
    
    // Copy the buffer into the shift register
    for (roll_index = 0; roll_index < nbits; roll_index++)
    {
        shift_register[roll_index] = buf[roll_index];
    }

    for (index = 0; index < (1 << nbits) - 1; index++)
    {
        feedback = shift_register[0];
        out[index] = feedback;
        for (taps_index = 0; taps_index < taps_length; taps_index++)
        {
            feedback ^= shift_register[taps[taps_index]];
        }

        // Shift the register by one
        for (roll_index = 0; roll_index < nbits - 1; roll_index++)
        {
            shift_register[roll_index] = shift_register[roll_index + 1];
        }

        // Set the last bit to the feedback
        shift_register[nbits - 1] = feedback;
    }

    free(shift_register);
    return 0;
}

void convert_zeros_to_minus_ones(int16_t* code, int n)
{
    uint32_t i;
    int8_t low_bit_value = -1;
    for ( i = 0; i < n; i++)
    {
        if (code[i] == 0)
        {
            code[i] = low_bit_value;
        }
        else
        {
            code[i] = 1;
        }
    }
}

int generate_one_gold_sequence(uint8_t n, gold_code_struct_t* output_code, uint16_t sequence_number)
{
    TapConfig *taps = preferred_pairs[n-5];
    int8_t sequence_length = (1 << n) - 1;
    int16_t* seq1 = malloc(sequence_length * sizeof(int16_t));
    int16_t* seq2 = malloc(sequence_length * sizeof(int16_t));
    if (!seq1 || !seq2)
    {
        free(seq1);
        free(seq2);
        return 1; // Memory allocation failed
    }

    // Generate seq1
    if (linear_feedback_shift_register(taps->taps1, taps->taps1_length, &seed[0], n, seq1) != 0)    
    {
        free(seq1);
        free(seq2);
        return 2; // LFSR failure
    }

    // Generate seq2
    if (linear_feedback_shift_register(taps->taps2, taps->taps2_length, &seed[0], n, seq2) != 0)
    {
        free(seq1);
        free(seq2);
        return 2; // LFSR failure
    }

    // Allocate interim memory for the gold sequences
    output_code->length = sequence_length;
    if(sequence_number == 0)
    {
        output_code->data = seq1;
        convert_zeros_to_minus_ones(output_code->data, sequence_length);
        free(seq2);
        return 0;
    }
    else if (sequence_number == 1)
    {
        output_code->data = seq2;
        convert_zeros_to_minus_ones(output_code->data, sequence_length);
        free(seq1);
        return 0;
    }
    else
    {
        output_code->data = malloc(sequence_length * sizeof(int16_t));
        if (!output_code->data)
        {
            free(seq1);
            free(seq2);
            return 1; // Memory allocation failed
        }
        roll_sequence(seq2, output_code->data, sequence_length, - (sequence_number - 2));
        xor_sequences(seq1, output_code->data, output_code->data, sequence_length);    
        convert_zeros_to_minus_ones(output_code->data, sequence_length);
    }
    free(seq1);
    free(seq2);

    return 0; // Success
}

int generate_all_gold_sequences(uint8_t n, gold_code_struct_t* output_codes)
{
    uint32_t i;
    uint32_t j;
    TapConfig *taps = preferred_pairs[n-5];
    int8_t sequence_length = (1 << n) - 1;
    int8_t num_codes = (1 << n) + 1;
    gold_code_struct_t* gold_sequences;
    int16_t* seq1 = malloc(sequence_length * sizeof(int16_t));
    int16_t* seq2 = malloc(sequence_length * sizeof(int16_t));
    if (!seq1 || !seq2)
    {
        free(seq1);
        free(seq2);
        return 1; // Memory allocation failed
    }

    // Generate seq1
    if (linear_feedback_shift_register(taps->taps1, taps->taps1_length, &seed[0], n, seq1) != 0)    
    {
        free(seq1);
        free(seq2);
        return 2; // LFSR failure
    }

    // Generate seq2
    if (linear_feedback_shift_register(taps->taps2, taps->taps2_length, &seed[0], n, seq2) != 0)
    {
        free(seq1);
        free(seq2);
        return 2; // LFSR failure
    }

    // Allocate interim memory for the gold sequences
    gold_sequences = malloc(num_codes * sizeof(gold_code_struct_t));
    if (!gold_sequences)
    {
        free(seq1);
        free(seq2);
        return 1; // Memory allocation failed
    }

    gold_sequences[0].data = seq1;
    gold_sequences[1].data = seq2;
    for ( i = 2; i < num_codes; i++)
    {
        gold_sequences[i].data = malloc(sequence_length * sizeof(int16_t));
        gold_sequences[i].length = sequence_length;
        if (!gold_sequences[i].data)
        {
            // Free all allocated memory in case of failure
            for ( j = 0; j < i; j++)
            {
                free(gold_sequences[j].data);
            }
            free(gold_sequences);
            return 1; // Memory allocation failed
        }
        roll_sequence(seq2, gold_sequences[i].data, sequence_length, - (i - 2));
        xor_sequences(seq1, gold_sequences[i].data, gold_sequences[i].data, sequence_length);
    }

    // Allocate memory for the output gold sequences
    if (!output_codes)
    {
        for ( i = 0; i < num_codes; i++)
        {
            free(gold_sequences[i].data);
        }
        free(gold_sequences);
        return 1; // Memory allocation failed
    }
    for ( i = 0; i < num_codes; i++)
    {
        output_codes[i].data = malloc(sequence_length * sizeof(int16_t));
        output_codes[i].length = sequence_length;
        if (!(output_codes)[i].data)
        {
            // Free all allocated memory in case of failure
            for ( j = 0; j <= i; j++)
            {
                free((output_codes)[j].data);
            }
            free(output_codes);
            for ( j = 0; j < num_codes; j++)
            {
                free(gold_sequences[j].data);
            }
            free(gold_sequences);
            return 1; // Memory allocation failed
        }

        for ( j = 0; j < (1 << n) - 1; j++)
        {
            (output_codes)[i].data[j] = gold_sequences[i].data[j];
        }

        convert_zeros_to_minus_ones((output_codes)[i].data, sequence_length);
    }

    // Cleanup
    for ( i = 0; i < num_codes; i++) free(gold_sequences[i].data);
    free(gold_sequences);

    return 0; // Success
}

// Function to sample a gold code with idle time between chips
int16_t sample_gold_code_with_idle_time(
    gold_code_struct_t *sampled_gold_code, 
    gold_code_struct_t *code, 
    double sample_rate, 
    double chip_duration, 
    double zeros_duration) 
{
    uint32_t i;
    double sample_time_step = 1.0 / sample_rate;
    double total_bit_duration = chip_duration + zeros_duration;
    double total_duration = code->length * total_bit_duration;
    uint32_t number_of_samples = (int)(total_duration * sample_rate);
    sampled_gold_code->length = number_of_samples;
    sampled_gold_code->data = (int16_t*)calloc(number_of_samples, sizeof(int16_t));
    if (sampled_gold_code->data == NULL) 
    {
        return -1; // Memory allocation failed
    }

    for ( i = 0; i < number_of_samples; i++)
    {
        double time_step = i * sample_time_step;
        int bit_index = (int)(time_step / total_bit_duration) % code->length;
        double within_bit_time = fmod(time_step, total_bit_duration);
        if (within_bit_time < chip_duration) 
        {
            sampled_gold_code->data[i] = code->data[bit_index];
        }
    }

    return 0;
}

// Function to pad a gold code with ones out to a specified number of bits
int16_t pad_gold_code_with_ones(
    gold_code_struct_t* padded_code,
    gold_code_struct_t* unpadded_code, 
    int target_length) 
{
    padded_code->length      = target_length;
    padded_code->data        = (int16_t*)malloc(target_length * sizeof(int16_t));
    if (padded_code->data == NULL) 
    {
        return -1; // Memory allocation failed
    }

    int i;
    for (i = 0; i < unpadded_code->length; i++) 
    {
        padded_code->data[i] = unpadded_code->data[i];
    }
    for (; i < padded_code->length; i++) 
    {
        padded_code->data[i] = 1; // Padding with ones
    }

    return 0;
}

// Wrapper function to pad the gold code and then sample it with idle times
int16_t pad_then_sample_gold_code_with_idle_time( 
    gold_code_struct_t* padded_sampled_code,
    gold_code_struct_t* unpadded_code, 
    int pad_length, 
    double sample_rate, 
    double chip_duration, 
    double zeros_duration) 
{
    // Pad the gold code first
    gold_code_struct_t padded_code;
    if (pad_gold_code_with_ones(&padded_code, unpadded_code, pad_length) ||
        padded_code.data == NULL) 
    {
        return -1;
    }

    // Sample the padded gold code
    if(sample_gold_code_with_idle_time(padded_sampled_code, &padded_code, sample_rate, chip_duration, zeros_duration)) 
    {
        if(padded_code.data != NULL)
        {
            free(padded_code.data);
        }
        return -1;
    }
    free(padded_code.data);

    return 0;
}

// Demo Code to test the Gold code generation and sampling functions
/*
int main() 
{
    int n = 6;                  // N = 6
    gold_code_struct_t* output_codes;     // Pointer to hold the generated Gold codes
    int num_codes = (1 << n) + 1;   // Number of Gold codes generated will be 2^n
    output_codes = malloc(num_codes * sizeof(gold_code_struct_t));

    int result = generate_all_gold_sequences(n, output_codes);
    if (result != 0) 
    {
        printf("Failed to generate Gold codes, error code: %d\n", result);
        free(output_codes);
        return result;
    }

    // Test the roll_sequence function
    int16_t python_rolls_negative[6][6] = {{0, 1, 2, 3, 4, 5}, {1, 2, 3, 4, 5, 0}, {2, 3, 4, 5, 0, 1}, {3, 4, 5, 0, 1, 2}, {4, 5, 0, 1, 2, 3}, {5, 0, 1, 2, 3, 4}};    
    int16_t python_rolls_positive[6][6] = {{0, 1, 2, 3, 4, 5}, {5, 0, 1, 2, 3, 4}, {4, 5, 0, 1, 2, 3}, {3, 4, 5, 0, 1, 2}, {2, 3, 4, 5, 0, 1}, {1, 2, 3, 4, 5, 0}};

    printf("Roll outputs:\n");
    int16_t rolls[] = {0, 1, 2, 3, 4, 5};
    int16_t output[6];
    int16_t amount_to_roll = -1;

    // Negative rolls
    for(int i = 0; i < 6; i++)
    {
        amount_to_roll = -1*i;
        roll_sequence(rolls, output, 6, amount_to_roll);
        printf("roll: %d\n", amount_to_roll);
        for (int j = 0; j < 6; j++) 
        {
            printf("%d ", output[j]);
        }
        printf(" C\n");
        for (int j = 0; j < 6; j++) 
        {
            printf("%d ", python_rolls_negative[i][j]);
        }
        printf(" Python\n");
    }

    // Positive rolls
    for(int i = 0; i < 6; i++)
    {
        amount_to_roll = i;
        roll_sequence(rolls, output, 6, amount_to_roll);
        printf("roll: %d\n", amount_to_roll);
        for (int j = 0; j < 6; j++) 
        {
            printf("%d ", output[j]);
        }
        printf(" C\n");
        for (int j = 0; j < 6; j++) 
        {
            printf("%d ", python_rolls_positive[i][j]);
        }
        printf(" Python\n");
    }

    // Print the generated Gold codes
    printf("Gold codes for N = 6:\n");
    for (int i = 3; i < 4; i++) 
    {
        printf("Code %d: ", i);
        for (int j = 0; j < (1 << n) - 1; j++) {
            printf("%i ", output_codes[i].data[j]);
        }
        printf("\n");
    }

    // Generate the sampled gold codes with idle times
    // One sampled gold code for each generated gold code
    double sample_rate = 4e6;  // 1 MHz
    double chip_duration = 6e-6;  // 1 us
    double zeros_duration = 3e-6;  // 1 us
    gold_code_struct_t sampled_gold_code_array[num_codes];

    for(int i = 0; i < num_codes; i++) 
    {
        sample_gold_code_with_idle_time(&sampled_gold_code_array[i], &(output_codes[i]), sample_rate, chip_duration, zeros_duration);
    }

    // Print the sampled gold codes
    printf("Sampled Gold codes for N = 6:\n");
    for (int i = 3; i < 4; i++) 
    {
        printf("Sampled Code %d: ", i);
        for (int j = 0; j < (int)(sample_rate * ((1 << n) - 1) * (chip_duration + zeros_duration)); j++) 
        {
            printf("%i ", sampled_gold_code_array[i].data[j]);
        }
        printf("\n");
    }


    // Test the padding function
    int16_t num_bits_padding = (1<<n) + 5;
    gold_code_struct_t padded_sampled_gold_code_array;
    if(pad_then_sample_gold_code_with_idle_time(
        &padded_sampled_gold_code_array,
        &(output_codes[3]),
        num_bits_padding, 
        sample_rate, 
        chip_duration, 
        zeros_duration))
    {
        printf("Error while padding gold code\n");
    }
    
    free(padded_sampled_gold_code_array.data);

    // Free memory allocated for sampled_gold_codes
    for (int i = 0; i < num_codes; i++) 
    {
        free(sampled_gold_code_array[i].data);
    }

    // Free memory allocated for output_codes
    for (int i = 0; i < num_codes; i++) {
        free(output_codes[i].data);
    }
    free(output_codes);

    return 0;
}
*/
