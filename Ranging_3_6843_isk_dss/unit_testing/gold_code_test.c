#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "../../shared/gold_code.h"

#define N 6
#define MAX_CODE_LENGTH 16384

// Function to compare two arrays
int compare_arrays(int16_t* arr1, int16_t* arr2, int length) 
{
    uint32_t i;
    for ( i = 0; i < length; i++)
    {
        if (arr1[i] != arr2[i]) 
        {
            return 0; // arrays are not equal
        }
    }
    return 1; // arrays are equal
}

// Function to read gold code data from a file
int read_truth_data(const char* filename, int16_t* data, int* length) 
{
    FILE* file = fopen(filename, "r");
    if (!file) 
    {
        printf("Failed to open file: %s\n", filename);
        return -1;
    }

    int i = 0;
    while (fscanf(file, "%d", &data[i]) != EOF && i < MAX_CODE_LENGTH) 
    {
        i++;
    }
    *length = i;
    fclose(file);
    return 0;
}


void test_generate_one_gold_sequence_function() 
{
    int expected_length;
    int16_t expected_data[MAX_CODE_LENGTH];
    char filename[64];

    gold_code_struct_t output_code;         // Pointer to hold the generated Gold codes
    int num_codes = (1 << N) + 1;           // Number of Gold codes generated will be 2^n + 1

    int result;
    int passes = 0;
    int fails = 0;

    for(int i = 0; i < num_codes; i++)
    {
        result = generate_one_gold_sequence(N, &output_code, i);
        if (result == 0)
        {
            sprintf(filename, "truth_data\\gold_truth_data_%d.txt", i);
            read_truth_data(filename, expected_data, &expected_length);
            if(compare_arrays(output_code.data, expected_data, expected_length))
            {
                passes += 1;
            }
            else 
            {
                printf("Test one gold sequence function %d: Failed\n", i);
                fails += 1;
            }
        }
        else 
        {
            printf("Test one gold sequence function: Failed\n");
            fails += 1;
        }
        if(output_code.data != NULL)
        {
            free(output_code.data); 
        }
    }
    printf("Test one gold sequence function passes: %d\n", passes);
    printf("Test one gold sequence function fails:  %d\n", fails);
}

void test_generate_all_gold_sequences_function() 
{
    int expected_length;
    int16_t expected_data[MAX_CODE_LENGTH];
    char filename[64];

    gold_code_struct_t* output_codes;       // Pointer to hold the generated Gold codes
    int num_codes = (1 << N) + 1;           // Number of Gold codes generated will be 2^n + 1
    output_codes = malloc(num_codes * sizeof(gold_code_struct_t));

    int result = generate_all_gold_sequences(N, output_codes);
    int passes = 0;
    int fails = 0;

    if (result == 0)
    {
        for(int i = 0; i < num_codes; i++)
        {
            sprintf(filename, "truth_data\\gold_truth_data_%d.txt", i);
            read_truth_data(filename, expected_data, &expected_length);
            if(compare_arrays(output_codes[i].data, expected_data, expected_length))
            {
                passes += 1;
            }
            else 
            {
                printf("Test gold function %d: Failed\n", i);
                fails += 1;
            }
        }
    }
    else 
    {
        printf("Test gold function: Failed\n");
        fails += 1;
    }
    printf("Test gold function passes: %d\n", passes);
    printf("Test gold function fails:  %d\n", fails);

    // Clean up
    for(int i = 0; i < num_codes; i++)
    {
        free(output_codes[i].data);
    }
    free(output_codes);
}

void test_pad_then_sample_gold_code_with_idle_time() 
{
    int base_length;
    int16_t expected_data[MAX_CODE_LENGTH];
    char filename[64];
    int num_codes = (1 << N) + 1;           // Number of Gold codes generated will be 2^n + 1
    uint16_t padding_length = 20;
    uint16_t total_length;

    double sample_rate       = 4000000;
    double chip_duration     = 0.000006;
    double zeros_duration    = 0.000003;
    int expected_length;

    gold_code_struct_t sampled_gold_code;
    gold_code_struct_t padded_code;
    gold_code_struct_t* output_codes;       // Pointer to hold the generated Gold codes
    output_codes = malloc(num_codes * sizeof(gold_code_struct_t));

    int result = generate_all_gold_sequences(N, output_codes);

    int passes = 0;
    int fails = 0;
    if (result == 0)
    {
        for(int i = 0; i < num_codes; i++)
        {
            total_length = output_codes[i].length + padding_length;

            // Pad the test data
            pad_gold_code_with_ones(&padded_code, &output_codes[i], total_length);

            // Generate test data
            if(sample_gold_code_with_idle_time(
                &sampled_gold_code, 
                &padded_code, 
                sample_rate, 
                chip_duration, 
                zeros_duration) == 0)
            {
                // Load truth data
                sprintf(filename, "truth_data\\sampled_padded_gold_with_idle_times_truth_data_%d.txt", i);
                read_truth_data(filename, expected_data, &expected_length);

                // Compare truth data with test data
                if(sampled_gold_code.length == expected_length)
                {
                    if(compare_arrays(sampled_gold_code.data, expected_data, expected_length))
                    {
                        passes += 1;
                    }
                    else 
                    {
                        printf("Test sampled padded gold with idle function %d: Failed\n", i);
                        fails += 1;
                    }
                }
                else
                {
                    printf("Test sampled padded gold with idle function %d: Failed\n", i);
                    fails += 1;
                }
            }

            // Cleanup
            free(sampled_gold_code.data);
            free(padded_code.data);
        }
    }
    else 
    {
        printf("Test sampled padded gold with idle function: Failed\n");
        fails += 1;
    }
    printf("Test sampled padded gold with idle function passes: %d\n", passes);
    printf("Test sampled padded gold with idle function fails:  %d\n", fails);

    // Clean up
    for(int i = 0; i < num_codes; i++)
    {
        free(output_codes[i].data);
    }
    free(output_codes);
}

void test_sample_gold_code_with_idle_time() 
{
    double sample_rate       = 4000000;
    double chip_duration     = 0.000006;
    double zeros_duration    = 0.000003;
    int expected_length;
    int16_t expected_data[MAX_CODE_LENGTH];
    char filename[64];

    gold_code_struct_t* output_codes;       // Pointer to hold the generated Gold codes
    int num_codes = (1 << N) + 1;           // Number of Gold codes generated will be 2^n + 1
    output_codes = malloc(num_codes * sizeof(gold_code_struct_t));

    int result = generate_all_gold_sequences(N, output_codes);
    int passes = 0;
    int fails = 0;
    
    gold_code_struct_t sampled_gold_code;

    if (result == 0)
    {
        for(int i = 0; i < num_codes; i++)
        {
            // Generate test data
            if(sample_gold_code_with_idle_time(
                &sampled_gold_code, 
                &output_codes[i], 
                sample_rate, 
                chip_duration, 
                zeros_duration) == 0)
            {
                // Load truth data
                sprintf(filename, "truth_data\\sampled_gold_with_idle_times_truth_data_%d.txt", i);
                read_truth_data(filename, expected_data, &expected_length);

                // Compare truth data with test data
                if(sampled_gold_code.length == expected_length)
                {
                    if(compare_arrays(sampled_gold_code.data, expected_data, expected_length))
                    {
                        passes += 1;
                    }
                    else 
                    {
                        printf("Test sample gold with idle function %d: Failed\n", i);
                        fails += 1;
                    }
                }
                else
                {
                    printf("Test sample gold with idle function %d: Failed\n", i);
                    fails += 1;
                }
            }

            // Cleanup
            free(sampled_gold_code.data);
        }
    }
    else 
    {
        printf("Test sample gold with idle function: Failed\n");
        fails += 1;
    }
    printf("Test sample gold with idle function passes: %d\n", passes);
    printf("Test sample gold with idle function fails:  %d\n", fails);

    // Clean up
    for(int i = 0; i < num_codes; i++)
    {
        free(output_codes[i].data);
    }
    free(output_codes);
}

void test_sample_gold_code_with_idle_time_preallocated_memory()
{
    
    double sample_rate       = 4000000;
    double chip_duration     = 0.000006;
    double zeros_duration    = 0.000003;
    int expected_length;
    int16_t expected_data[MAX_CODE_LENGTH];
    char filename[64];

    gold_code_struct_t* output_codes;       // Pointer to hold the generated Gold codes
    int num_codes = (1 << N) + 1;           // Number of Gold codes generated will be 2^n + 1
    output_codes = malloc(num_codes * sizeof(gold_code_struct_t));

    int result = generate_all_gold_sequences(N, output_codes);
    int passes = 0;
    int fails = 0;

    gold_code_struct_t sampled_gold_code;
    int16_t preallocated_memory[MAX_CODE_LENGTH];

    if (result == 0)
    {
        for (int i = 0; i < num_codes; i++)
        {
            // Generate test data
            if (sample_gold_code_with_idle_time_preallocated_memory(
                    &sampled_gold_code,
                    &output_codes[i],
                    sample_rate,
                    chip_duration,
                    zeros_duration,
                    preallocated_memory,
                    sizeof(preallocated_memory) / sizeof(int16_t)) == 0)
            {
                // Load truth data
                sprintf(filename, "truth_data\\sampled_gold_with_idle_times_truth_data_%d.txt", i);
                read_truth_data(filename, expected_data, &expected_length);

                // Compare truth data with test data
                if (sampled_gold_code.length == expected_length)
                {
                    if (compare_arrays(sampled_gold_code.data, expected_data, expected_length))
                    {
                        passes += 1;
                    }
                    else
                    {
                        printf("Test sample gold with idle function (preallocated memory) %d: Failed\n", i);
                        fails += 1;
                    }
                }
                else
                {
                    printf("Test sample gold with idle function (preallocated memory) %d: Failed\n", i);
                    fails += 1;
                }
            }
        }
    }
    else
    {
        printf("Test sample gold with idle function (preallocated memory): Failed\n");
        fails += 1;
    }
    printf("Test sample gold with idle function (preallocated memory) passes: %d\n", passes);
    printf("Test sample gold with idle function (preallocated memory) fails:  %d\n", fails);

    // Clean up
    for (int i = 0; i < num_codes; i++)
    {
        free(output_codes[i].data);
    }
    free(output_codes);
}


void test_pad_gold_code_with_ones() 
{
    int base_length;
    int16_t base_data[MAX_CODE_LENGTH];
    int16_t expected_data[MAX_CODE_LENGTH];
    char filename[64];
    int num_codes = (1 << N) + 1;           // Number of Gold codes generated will be 2^n + 1
    uint16_t j;
    gold_code_struct_t padded_code;
    gold_code_struct_t unpadded_code;
    uint16_t padding_length;
    uint16_t total_length;
    int passes = 0;
    int fails = 0;

    for(int i = 0; i < num_codes; i++)
    {
        // Generate a random padding length
        padding_length = rand() % 512;

        // Load truth data
        sprintf(filename, "truth_data\\gold_truth_data_%d.txt", i);
        read_truth_data(filename, base_data, &base_length);
        unpadded_code.data = calloc(MAX_CODE_LENGTH, sizeof(int16_t));  
        unpadded_code.length = base_length;
        total_length = base_length + padding_length;
        for(j = 0; j < base_length; j++)
        {
            expected_data[j] = base_data[j];
            unpadded_code.data[j] = base_data[j];
        }
        for(; j < total_length; j++)
        {
            expected_data[j] = 1;
        }

        // Generate test data
        pad_gold_code_with_ones(&padded_code, &unpadded_code, total_length);

        // Compare truth data with test data
        if(compare_arrays(padded_code.data, expected_data, total_length))
        {
            passes += 1;
        }
        else 
        {
            printf("Test padding function %d: Failed\n", i);
            fails += 1;
        }
        free(unpadded_code.data);
        free(padded_code.data);
    }
    printf("Test padding function passes: %d\n", passes);
    printf("Test padding function fails:  %d\n", fails);
}

uint8_t compare_floats(float a, float b, float epsilon) 
{
    if ((a - b) < epsilon && (b - a) < epsilon)
    {
        return 1;
    }
    return 0;
}

/*
void test_linear_regression_function() 
{
    int passes = 0;
    int fails = 0;
    // noise free data
    uint16_t noise_free_x[] = {1, 2, 3, 4, 5};
    uint16_t noise_free_y[] = {2, 4, 6, 8, 10};
    int n = sizeof(noise_free_x) / sizeof(noise_free_x[0]);

    float noise_free_slope_result, noise_free_intercept_result;
    float noise_free_slope_truth = 2.0;
    float noise_free_intercept_truth = 0.0;
    int result = linear_regression(n, &noise_free_x[0], &noise_free_y[0], &noise_free_slope_result, &noise_free_intercept_result);

    if (result == 0) 
    {
        if(noise_free_slope_result == noise_free_slope_truth && noise_free_intercept_result == noise_free_intercept_truth)
        {
            passes += 1;
        } 
        else 
        {
            printf("Failed noise-free linear regression test.\n");
            fails += 1;
        }
    } 
    else 
    {
        printf("Failed noise-free linear regression test.\n");
        fails += 1;
    }

    // noisy data
    uint16_t noisy_x[] = {10,  20,  30,  40,  50,  60,  70,  80,  90, 100};
    uint16_t noisy_y[] = {25,  41,  65,  85, 102, 122, 142, 162, 179, 201};
    n = sizeof(noisy_x) / sizeof(noisy_x[0]);

    float noisy_slope_result, noisy_intercept_result;
    // Computed using numpy/scipy stats.linregress on this data
    float noisy_slope_truth = 1.9551;
    float noisy_intercept_truth = 4.8666;
    float epsilon = 0.0001;
    result = linear_regression(n, &noisy_x[0], &noisy_y[0], &noisy_slope_result, &noisy_intercept_result);

    if (result == 0) 
    {
        
        if(compare_floats(noisy_slope_result, noisy_slope_truth, epsilon) && 
        compare_floats(noisy_intercept_result, noisy_intercept_truth, epsilon))
        {
            passes += 1;
        } 
        else 
        {
            printf("Failed noisy linear regression test.\n");
            fails += 1;
        }
    } 
    else 
    {
        printf("Failed noisy linear regression test.\n");
        fails += 1;
    }

    // Purposefully failing test
    uint16_t failing_x[] = {1, 2, 3, 4, 5};
    uint16_t failing_y[] = {3, 6, 9, 12, 15};
    n = sizeof(failing_x) / sizeof(failing_x[0]);

    float failing_slope_result, failing_intercept_result;
    float failing_slope_truth = 3.1;
    float failing_intercept_truth = 0.0;
    result = linear_regression(n, &failing_x[0], &failing_y[0], &failing_slope_result, &failing_intercept_result);

    if (result == 0) 
    {
        if(failing_slope_result == failing_slope_truth && failing_intercept_result == failing_intercept_truth)
        {
            printf("Failed purposeful failure linear regression test.\n");
            fails += 1;
        } 
        else 
        {
            passes += 1;
        }
    } 
    else 
    {
        printf("Failed purposeful failure linear regression test.\n");
        fails += 1;
    }

    printf("Test linear regression passes: %d\n", passes);
    printf("Test linear regression fails:  %d\n", fails);

}
*/

int main() {
    test_generate_all_gold_sequences_function();

    test_pad_gold_code_with_ones();

    test_sample_gold_code_with_idle_time();

    test_sample_gold_code_with_idle_time_preallocated_memory();

    test_pad_then_sample_gold_code_with_idle_time();

    test_generate_one_gold_sequence_function();

    //test_linear_regression_function();
    
    return 0;
}
