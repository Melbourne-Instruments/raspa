/*
 * Copyright 2018-2020 Modern Ancient Instruments Networked AB, dba Elk
 * RASPA is free software: you can redistribute it and/or modify it under the terms
 * of the GNU General Public License as published by the Free Software Foundation,
 * either version 3 of the License, or (at your option) any later version.
 *
 * RASPA is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
 * PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with RASPA.
 * If not, see http://www.gnu.org/licenses/ .
 */

/**
 * @brief Header file which deals converting the samples from integer to floating point
 * @copyright 2020-2023 Melbourne Instruments, Australia
 */

#ifndef SAMPLE_CONVERSION_H_
#define SAMPLE_CONVERSION_H_

#include <memory>
#include <fstream>
#include <vector>
#include <iomanip>
#include <chrono>
#include <thread>
#include "driver_config.h"

// Define this to output a test samples pattern for each channel
//#define RASPA_TEST_OUTPUT_SAMPLES1      0
//#define RASPA_TEST_OUTPUT_SAMPLES2      0

namespace {
constexpr float FLOAT_TO_INT24_SCALING_FACTOR = 8388607.0f;       // 2**23 - 1
constexpr float INT24_TO_FLOAT_SCALING_FACTOR = 1.19209304e-07f;  // 1.0 / (2**23 - 1)
constexpr float INT32_TO_FLOAT_SCALING_FACTOR = 4.656612873e-10f; // 1.0 / (2**31 - 1)
constexpr float FLOAT_TO_CODEC_SCALING_FACTOR = FLOAT_TO_INT24_SCALING_FACTOR;
constexpr float CODEC_TO_FLOAT_SCALING_FACTOR = INT32_TO_FLOAT_SCALING_FACTOR;
}

namespace raspa {

constexpr auto DEFAULT_CODEC_FORMAT = RaspaCodecFormat::INT24_LJ;
constexpr int MIN_NUM_CHANNELS = 38;
constexpr int MAX_NUM_CHANNELS = 38;
constexpr int NUM_FPGA_STATUS_REGS = 48;
constexpr int MIN_BUFFER_SIZE = 8;
constexpr int MAX_BUFFER_SIZE = 1024;
constexpr int BUFFER_SIZE     = 128;
#if defined(RASPA_SPI_DATA_CHECKSUMED) && defined(RASPA_LOG_SPI_DATA_CHECKSUM_ERRORS)
constexpr int reserve_size_skips = 1024;
constexpr uint NUM_SKIP_SAMPLES_BEFORE_CHECKSUM_CHECKS = 10000;
#endif

#if defined(RASPA_TEST_OUTPUT_SAMPLES1) || defined(RASPA_TEST_OUTPUT_SAMPLES2)
static float sample = -1.0;
#endif

/**
 * @brief Interface class for sample conversion
 */
class BaseSampleConverter
{
public:
    BaseSampleConverter() = default;
    ~BaseSampleConverter() = default;

    /**
     * @brief deinterleaves samples and converts it from the native codec format
     *        to float32
     * @param dst The destination buffer which holds the float samples
     * @param src The source buffer which holds samples in native codec format
     */
    virtual void codec_format_to_float32n(float* dst, int32_t* src) = 0;

    /**
     * @brief Interleaves samples and converts it from float32 to the codec's
     *        native format.
     * @param dst The destination buffer which holds the samples in native codec
     *        format
     * @param src The source buffer which holds samples in float32 format
     */
    virtual void float32n_to_codec_format(int32_t* dst, float* src) = 0;
    virtual void start_spi_error_logging() = 0;
    virtual void stop_spi_error_logging() = 0;
    virtual void _process_spi_error_logging() = 0;
    virtual void _write_spi_error_log() = 0;
};


/**
 * @brief Templated class which performs optimized sample conversion. The
 *        optimization comes from the fact that the template parameters control
 *        the inner and outer loops.
 * @tparam codec_format The codec format.
 * @tparam buffer_size_in_frames The buffer size in frames
 * @tparam num_channels The number of channels
 */
int miso_errors = 0;
int mosi_errors_1 = 0;
int mosi_errors_2 = 0;


template<RaspaCodecFormat codec_format, int buffer_size_in_frames, int num_channels>
class SampleConverter : public BaseSampleConverter
{
public:
#if defined(RASPA_SPI_DATA_CHECKSUMED) && defined(RASPA_LOG_SPI_DATA_CHECKSUM_ERRORS)
// Counter so we can keep track of how many buffers have passed
uint64_t buffer_counter = 0;

// Thread to log any SPI errors
std::thread *spi_error_logging_thread = nullptr;
bool exit_spi_error_logging_thread = false;
uint spi_error_log_buffer_index = 0;

// Vector to store the buffer when an error occured
std::vector<uint64_t > checksum_buffer_vector;

// Store the checksum errors that occured
std::vector<uint32_t> checksum_values_vector; 
std::chrono::_V2::system_clock::time_point start_of_day;
    SampleConverter() : _samples_skiped(0) 
    {
        // Log the startup time
        start_of_day = std::chrono::system_clock::now();

        // To avoid allocations, reseve vector size. 2 values are written to values 
        // vector for each error, so reserve double. 
        checksum_buffer_vector.reserve(reserve_size_skips);
        checksum_values_vector.reserve(reserve_size_skips*2);
    }
    ~SampleConverter() {
        if (spi_error_logging_thread)
            delete spi_error_logging_thread;
    }
#else
    SampleConverter() = default;
    ~SampleConverter() = default;
#endif
#if defined(RASPA_SPI_DATA_CHECKSUMED) && defined(RASPA_LOG_SPI_DATA_CHECKSUM_ERRORS)
    void start_spi_error_logging() override
    {
        // Open the SPI error log file
        _spi_error_log_file.open("/udata/spi.log", std::ios_base::app);

        // Start the SPI error logging thread
        spi_error_logging_thread = new std::thread(&SampleConverter::_process_spi_error_logging, this); 
    }

    void stop_spi_error_logging() override
    {
        // Stop the SPI error logging thread
        exit_spi_error_logging_thread = true;
    }

    void _process_spi_error_logging() override
    {
        // Process forever until stopped
        while (!exit_spi_error_logging_thread)
        {
            // Write the log if there are any entries
            _write_spi_error_log();

            // Sleep for 10s and check again
            std::this_thread::sleep_for(std::chrono::seconds(10));
        }

        // Thread exited, write any entries not processed yet and close the log
        _write_spi_error_log();
        _spi_error_log_file.close();
    }

    void _write_spi_error_log() override
    {     
        // Are there any buffer entries to process?
        uint buffer_size = checksum_buffer_vector.size();
        if (spi_error_log_buffer_index < buffer_size)
        {
            // Log the entries
            auto in_time_t = std::chrono::system_clock::to_time_t(start_of_day);
            std::stringstream datetime;
            datetime << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d_%X");
            _spi_error_log_file << datetime.str() << " write triggered\n";

            for (; spi_error_log_buffer_index < buffer_size; spi_error_log_buffer_index++) {
                auto timestamp = checksum_buffer_vector.at(spi_error_log_buffer_index);
                auto value_1 = checksum_values_vector.at(2 * spi_error_log_buffer_index);
                auto value_2 = checksum_values_vector.at((2 * spi_error_log_buffer_index) + 1);

                // MOSI error case:
                // This method of detecting a mosi error can be fooled if the miso checksum 
                // happened to be 0 or 1, but this seems unlikely. we could add another array 
                // to indicate the error type.
                if (((value_1 == 0) || (value_1 == 1)) &&
                    ((value_2 == 0) || (value_2 == 1))) {
                    _spi_error_log_file << datetime.str() << " " << timestamp
                                        << " MOSI checksum error: "
                                        << ((value_1 == 0) ? "SPI0 NOK," : "SPI0 OK,")
                                        << ((value_2 == 0) ? " SPI4 NOK" : "SPI4 OK") << "\n";
                } 
                // MISO error case:
                else {
                    _spi_error_log_file << datetime.str() << " " << timestamp
                                        << " MISO checksum error: " << std::hex << value_1 << " "
                                        << std::hex << value_2 << "\n";
                }
            }
            _spi_error_log_file.flush();                
        }
    }
#else
    void start_spi_error_logging() override {};
    void stop_spi_error_logging() override {};
    void _process_spi_error_logging() override {};
    void _write_spi_error_log() override {};
#endif

#if defined(RASPA_SPI_DATA_CHECKSUMED) && defined(RASPA_LOG_SPI_DATA_CHECKSUM_ERRORS)
    /**
     * @brief Number of samples to skip at the start of processing before
     *        checking the received data checksums
     */
    uint _samples_skiped;
    std::ofstream _spi_error_log_file;
#endif

    /**
     * @brief deinterleaves samples and converts it from the native codec format
     *        to float32
     * @param dst The destination buffer which holds the float samples
     * @param src The source buffer which holds samples in native codec format
     */
    void codec_format_to_float32n(float* dst, int32_t* src) override
    {
        int32_t buffer[9*BUFFER_SIZE];
#if defined(RASPA_SPI_DATA_CHECKSUMED) && defined(RASPA_LOG_SPI_DATA_CHECKSUM_ERRORS)
        uint32_t sent_checksum = 0;
        uint32_t calc_checksum = 0;
#endif

        /*
         ** Note 1: These are the following input channels (all 96kHz):
         ** - 2 x FX audio in channels
         ** - 1 x Mic in channel
         ** - 4 x CV in channels
         ** - 1 x Diagnostic channel
         ** - 1 x FPGA Status Registers channel
         ** Note 1: Other than the FPGA Status Registers, the input channels are received 
         ** as 24-bit values in a 32-bit word. They are not packed for performance reasons.
         ** Note 2: The FPGA Status Registers are simply a block of 32-bit words, put into
         ** a channel for ease of processing by the VST plug-in
         ** Note 3: It should be noted that the number of input and output
         ** channels in Sushi must be always symmetric. The unused input channels
         ** can be ignored.
         */

        /* This is a new buffer, so increment the buffer count */
#if defined(RASPA_SPI_DATA_CHECKSUMED) && defined(RASPA_LOG_SPI_DATA_CHECKSUM_ERRORS)
        buffer_counter++;
#endif
        
        /* Initialise the destination buffer to all zeros */
        memset(dst, 0, (9*BUFFER_SIZE*sizeof(int32_t)));

        /* 
         ** Copy the samples from the source buffer we are interested in
         ** It is (a little) faster to process this buffer than directly from the
         ** source DMA buffer
         ** Note: The first received sample is thrown away
         */
        memcpy(buffer, (src + 1), (9*BUFFER_SIZE*sizeof(int32_t)));

        /* Loop through setting up each channel, other than the FPGA Status Registers channel */
        for (int c=0; c<8; c++) {
            for (int n=0; n<BUFFER_SIZE; n++) {
                /* Audio channel - get the sample */
                *dst++ = buffer[c + (n<<3)] * CODEC_TO_FLOAT_SCALING_FACTOR;
            }
        }

        /* 
         ** Starting from the last audio channel sample, copy each FPGA status ** reg to the buffers while also converting from 24 bit int to **normalised float 
         ** Any unused bytes in this channel can be ignored
         */
        for(int n =0; n < NUM_FPGA_STATUS_REGS; n++)
        {
            *dst++ = buffer[8*BUFFER_SIZE + n] * INT24_TO_FLOAT_SCALING_FACTOR;
        }
#if defined(RASPA_SPI_DATA_CHECKSUMED) && defined(RASPA_LOG_SPI_DATA_CHECKSUM_ERRORS)
        // Only process the checksums after n samples have been skipped at the start
        // of processing
        if (_samples_skiped >= NUM_SKIP_SAMPLES_BEFORE_CHECKSUM_CHECKS)
        {
            // Get the sent checksum
            sent_checksum = buffer[(8*BUFFER_SIZE) + 48];

            // Calculate the expected checksum
            uint32_t *buf = (uint32_t *)buffer;
            for (uint i=0; i<((8*BUFFER_SIZE) + NUM_FPGA_STATUS_REGS); i++)
            {
                calc_checksum += *buf++;
            }

            // Check the values, print an error if not the same
            if (sent_checksum != calc_checksum)
            {
                miso_errors++;
                
                // Get the timestamp, write the checksums 
                checksum_buffer_vector.push_back(buffer_counter);
                checksum_values_vector.push_back(sent_checksum);
                checksum_values_vector.push_back(calc_checksum);

            }

            // Also check the MOSI checksums (and zero words between each checksum)
            if ((buffer[(8*BUFFER_SIZE) + 48 + 2] == 0) || (buffer[(8*BUFFER_SIZE) + 48 + 4] == 0) )
            {
                mosi_errors_1++;

                // Get the timestamp & write the error status of each FPGA
                checksum_buffer_vector.push_back(buffer_counter);
                checksum_values_vector.push_back((buffer[(8*BUFFER_SIZE) + 48 + 2]));
                checksum_values_vector.push_back(buffer[(8*BUFFER_SIZE) + 48 + 4]);
            }
        }
        else
        {
            // Increment the number of skipped samples
            _samples_skiped++;
        }      
#endif
    }

    /**
     * @brief Interleaves samples and converts it from float32 to the codec's
     *        native format.
     * @param dst The destination buffer which holds the samples in native codec
     *        format
     * @param src The source buffer which holds samples in float32 format
     */
    void float32n_to_codec_format(int32_t* dst, float* src) override
    {
        int32_t dst_unpacked[(20*BUFFER_SIZE)*2];
        float   *src_channel_ptrs[20*2];
        int32_t *dst_unpacked_ptr = dst_unpacked;
#ifdef RASPA_SPI_DATA_CHECKSUMED        
        uint32_t checksum = 0;
#endif

#ifdef RASPA_TEST_OUTPUT_SAMPLES1
        // Create a test pattern for each channel - ramp up for each sample block
        // HQ Channel 1
        float *src_data = src;
        float sc = sample;
        for (int n=0; n<64; n++)
        {
            *src_data++ = sc;
            sc += 0.000005;
        }
        // HQ Channel 2
        sc = sample;
        for (int n=0; n<64; n++)
        {
            *src_data++ = sc;
            sc += 0.000005;
        }
        // Voice Channels 1-12
        for (int c=2; c<(2+(12*3)); c+=3)
        {
            // Voice Channel: HQ
            sc = sample;
            for (int n=0; n<64; n++)
            {
                *src_data++ = sc;
                sc += 0.000005;
            }
            // Voice Channel: Control channels 1-8
            for (int n=0; n<64; n+=8)
            {
                sc = sample;
                for (int i=0; i<8; i++) {
                    *src_data++ = sc;
                    sc += (0.000005 * 8);
                }
            }
            // Voice Channel: Control channels 9-16
            for (int n=0; n<64; n+=8)
            {
                sc = sample;
                for (int i=0; i<8; i++) {
                    *src_data++ = sc;
                    sc += (0.000005 * 8);
                }
            }
        }

        // Set the next sample count (max 18 bits)
        if ((sample + (0.000005 * 64)) < 1.0)
            sample += (0.000005 * 64);
        else
            sample = -1.0;
#elif RASPA_TEST_OUTPUT_SAMPLES2
            // Create a test pattern for each channel - ramp up for each sample block
            // HQ Channel 1
            float *src_data = src;
            [[maybe_unused]]float sc = sample;
            for (int n = 0; n < 64; n++)
            {
                int val =0x0ccccc;
                *src_data++ = (float)val * CODEC_TO_FLOAT_SCALING_FACTOR;
                
            }
            // HQ Channel 2
            sc = sample;
            for (int n = 0; n < 64; n++)
            {
                int val = 0x033333;
                *src_data++ = (float)val * CODEC_TO_FLOAT_SCALING_FACTOR;
            }
            // Voice Channels 1-12
            for (int c = 2; c < (2 + (12 * 3)); c += 3)
            {
                // Voice Channel: HQ
                sc = sample;
                for (int n = 0; n < 64; n++)
                {
                   int val = ((c/3) << 16) + 0xccc;

                    *src_data++ = (float)val * CODEC_TO_FLOAT_SCALING_FACTOR;
                }
                // Voice Channel: Control channels 1-8
                for (int n = 0; n < 64; n += 8)
                {
                    sc = sample;
                    for (int i = 0; i < 8; i++)
                    {
                        int val;
                        val = ((c/3) << 16) + ((n /8) << 12) + 0x555;
                        *src_data++ = ((float)val)*CODEC_TO_FLOAT_SCALING_FACTOR;
                    }
                }
                // Voice Channel: Control channels 9-16
                for (int n = 0; n < 64; n += 8)
                {
                    sc = sample;
                    for (int i = 0; i < 8; i++)
                    {
                        int val;
                        val = ((c/3) << 16) + (((n /8) + 8) << 12) + 0xAAA;
                        *src_data++ = (float)val*CODEC_TO_FLOAT_SCALING_FACTOR;
                    }
                }
            }

            // Set the next sample count (max 18 bits)
            if ((sample + (0.000005 * 64)) < 1.0)
                sample += (0.000005 * 64);
            else
                sample = -1.0;
#endif

        /* Initialise the unpacked buffer to all zeros */
        memset(dst_unpacked, 0, sizeof(dst_unpacked));

        /* 
         ** Setup the pointers to each source channel
         ** There are two blocks of pointers, one for each
         ** FPGA
         ** Note the HQ audio channels are sent in both blocks, but only
         ** processed by one FPGA at this stage
         */
        src_channel_ptrs[0] = src;
        src_channel_ptrs[20] = src_channel_ptrs[0];
        src += BUFFER_SIZE;
        src_channel_ptrs[1] = src;
        src_channel_ptrs[21] = src_channel_ptrs[1];
        for (int i=2; i<20; i++) {
            src += BUFFER_SIZE;
            src_channel_ptrs[i] = src; 
        }
        for (int i=22; i<40; i++) {
            src += BUFFER_SIZE;
            src_channel_ptrs[i] = src; 
        }

        /* Process each block for the two FPGAs */
        float **src_channel_ptrs_block = src_channel_ptrs;
        for (int f=0; f<2; f++)
        {
            /* Process each sample block */
            for (int n=0; n<BUFFER_SIZE; n+=2) {
                /* Send the two HQ audio channel samples (two samples for each) */
                *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[0]++);
                *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[0]++);
                *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[1]++);
                *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[1]++);

                // Send the voice samples - in each block we:
                // - Send two voice HQ channel samples, and four of the 16 control
                //   channel samples
                // Note for each voice there are:
                // - 1 x HQ channel @96k
                // - 16 x control channels @12k
                // This results in 3 x 96k channels per voice
                // Note: 6 voices are sent per FPGA block
                for (int c=2; c<(2+(6*3)); c+=3) {
                    *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c]++);
                    *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c]++);

                    // Send four voice control samples
                    // Note we send 16 from the first 96k channel, followed 16 from the second 96k channel
                    // Also note the control channels are already interleaved in the 96k channel
                    if ((n & 0x4) == 0) {
                        // Get four samples from the first 96k channel
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+1]++);
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+1]++);
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+1]++);
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+1]++);
                    }
                    else {
                        // Get four samples from the second 96k channel
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+2]++);
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+2]++);
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+2]++);
                        *dst_unpacked_ptr++ = _float32n_sample_to_codec_format(src_channel_ptrs_block[c+2]++); 
                    }                 
                }
            }
            src_channel_ptrs_block += 20;
        }

        /* 
         ** We now have the unpacked output buffer, ready to be packed
         ** Pack the samples into 24 bits, 4 samples at a time
         ** Do this for each FPGA block in turn
         */
        dst_unpacked_ptr = dst_unpacked;
        int32_t *dst_ptr = dst;  
        for (int i=0; i<(20*BUFFER_SIZE); i+= 4) {
            /*
             ** Note this function call increments the source pointer
             ** by 4, and destination pointer by 3
             */
            _pack_4_samples(dst_ptr, dst_unpacked_ptr);
#ifdef RASPA_SPI_DATA_CHECKSUMED            
            for (int i=0; i<4; i++) {

                uint32_t tmp = *(uint32_t *)dst_unpacked_ptr++;
                checksum += 0x00ffffffu & tmp;
            }
#else
            dst_unpacked_ptr += 4;
#endif
            dst_ptr += 3;
        }

#ifdef RASPA_SPI_DATA_CHECKSUMED
        // Set the checksum and skip past this and the next zero word
        // to process the second FPGA block
        *dst_ptr++ = checksum;
        dst_ptr++;
        checksum=0;
#else
        // Skip past the two words at the end of the packed data
        dst_ptr += 2;
#endif

        /* 
         ** Pack the samples into 24 bits, 4 samples at a time
         ** for the second FPGA
         */      
        for (int i=0; i<(20*BUFFER_SIZE); i+= 4) {
            /*
             ** Note this function call increments the source pointer
             ** by 4, and destination pointer by 3
             */
            _pack_4_samples(dst_ptr, dst_unpacked_ptr);
#ifdef RASPA_SPI_DATA_CHECKSUMED            
            for (int i=0; i<4; i++) {

                checksum +=0x00ffffffu&( *(uint32_t *)dst_unpacked_ptr++);
            }
#else
            dst_unpacked_ptr += 4;
#endif
            dst_ptr += 3;
        }

#ifdef RASPA_SPI_DATA_CHECKSUMED
        // Set the checksum
        *dst_ptr = checksum;
#endif
    }

private:
    int32_t _float32n_sample_to_codec_format(float *src)
    {
        float x = *src;
        if (x < -1.0f)
        {
            x = -1.0f;
        }
        else if (x > 1.0f)
        {
            x = 1.0f;
        }
        return (int32_t) (x * FLOAT_TO_CODEC_SCALING_FACTOR) & 0x00FFFFFF;
    }

    void _pack_4_samples(int32_t *dst, int32_t *src)
    {
        int32_t packed_word;

        /* 
         ** 24-bit samples are packed such that 4 samples
         ** fit into 3 x 32 bit words
         */

        /* Packed word 1 */
        packed_word = *src++ << 8;
        packed_word += *src >> 16;
        _reverse_packed_word_byte_order(dst++, packed_word);

        /* Packed word 2 */
        packed_word = *src++ << 16;
        packed_word += *src >> 8;
        _reverse_packed_word_byte_order(dst++, packed_word);

        /* Packed word 3 */
        packed_word = *src++ << 24;
        packed_word += *src;
        _reverse_packed_word_byte_order(dst++, packed_word);
    }

    void _reverse_packed_word_byte_order(int32_t *dst, int32_t packed_word)
    {
        int32_t dst_word;

        /*
         ** Swap the packed word byte order so that the LS byte starts from
         ** the MS byte position
         */
        dst_word = (packed_word << 24) & 0xFF000000;
        dst_word += (packed_word << 8) & 0x00FF0000;
        dst_word += (packed_word >> 8) & 0x0000FF00;
        dst_word += (packed_word >> 24) & 0x000000FF;

#ifdef RASPA_SPI_LOOPBACK_TESTING
        // Mask the LS bit in each byte
        dst_word &=  0xFEFEFEFE;
#endif

        *dst = dst_word; 
    }

    /**
     * @brief Converts samples in native codec format to int32rj
     * @param sample The sample in native codec format
     * @return The sample in int32_rj format
     */
    int32_t _codec_format_to_int32rj(int32_t sample)
    {
        if constexpr (codec_format == RaspaCodecFormat::INT24_LJ)
        {
            return sample >> 8;
        }

        else if constexpr (codec_format == RaspaCodecFormat::INT24_I2S)
        {
            /**
             * This format does not have the sign info in the first bit.
             * So we need to manually extend the sign bits to convert it to
             * int32_rj. Fastest way is to use two shifts.
             */
            sample = sample << 1;
            return sample >> 8;
        }
        else if constexpr (codec_format == RaspaCodecFormat::INT24_RJ)
        {
            /**
             * This format does not have the sign info in the first 8 bits.
             * So we need to manually extend the sign bits to convert it to
             * int32_rj. Fastest way is to use two shifts.
             */

            sample = sample << 8;
            return sample >> 8;
        }
        else
        {
            // samples already in int32_rj format
            return sample;
        }
    }

    /**
     * @brief Converts sample in int32_rj format to native codec format.
     * @param sample The sample in int32_rj format
     * @return The sample in native codec format
     */
    int32_t _int32rj_to_codec_format(int32_t sample)
    {
        if constexpr (codec_format == RaspaCodecFormat::INT24_LJ)
        {
            return sample << 8;
        }
        else if constexpr (codec_format == RaspaCodecFormat::INT24_I2S)
        {
            return (sample << 7) & 0x7FFFFF00;
        }
        else if constexpr (codec_format == RaspaCodecFormat::INT24_RJ)
        {
            return sample & 0x00FFFFFF;
        }
        else
        {
            // sample already in codec format
            return sample;
        }
    }
};

/**
 * @brief Gets the next supported buffer size
 * @param buffer_size the current buffer size
 * @return {true, next buffer size} if current buffer size is not equal to the
 *         maximum allowed buffer size.
 *         {false, current buffer size} otherwise
 */
constexpr std::pair<bool, int> get_next_buffer_size(int buffer_size)
{
    if (buffer_size != MAX_BUFFER_SIZE)
    {
        return {true, buffer_size * 2};
    }

    return {false, buffer_size};
}

/**
 * @brief Gets the next supported number of channels
 * @param num_channels the current number of channels
 * @return {true, next number of channels} if current number of channels is not
 *         equal to the maximum allowed number of channels.
 *         {false, current number of channels} otherwise
 */
constexpr std::pair<bool, int> get_next_num_channels(int num_channels)
{
    if (num_channels != MAX_NUM_CHANNELS)
    {
        return {true, num_channels + 2};
    }

    return {false, num_channels};
}

/**
 * @brief Gets the next supported codec format
 * @param codec_format the current codec format
 * @return {true, next codec format} if current number codec format is not
 *         equal to the last possible RaspaCodecFormat.
 *         {false, current codec format} otherwise
 */
constexpr std::pair<bool, RaspaCodecFormat>
get_next_codec_format(RaspaCodecFormat codec_format)
{
    if (codec_format != RaspaCodecFormat::INT32_RJ)
    {
        return {true,
                static_cast<RaspaCodecFormat>(static_cast<int>(codec_format) +
                                              1)};
    }

    return {false, codec_format};
}

/**
 * @brief Get a pointer to an instance of a BaseSampleConvertor object. This
 *        function deduces the template arguments for SampleConverter and
 *        instantiates it for the  following argument values:
 *            - buffer sizes : 8, 16, 32, 64, 128, 256, 512, 1024
 *            - number of channels :  2, 4, 6, 8
 *            - Codec formats : INT24_LJ, INT24_I2S, INT24_RJ, INT32_RJ
 * @param codec_format The codec format
 * @param buffer_size_in_frames The buffer size in frames
 * @param num_channels The number of channels.
 * @return A SampleConverter instance if buffer size and num channels
 *         is supported, empty unique_ptr otherwise.
 */
template<RaspaCodecFormat expected_format = DEFAULT_CODEC_FORMAT,
         int expected_buffer_size = MIN_BUFFER_SIZE,
         int expected_num_chans = MIN_NUM_CHANNELS>
std::unique_ptr<BaseSampleConverter> get_sample_converter(RaspaCodecFormat codec_format,
                                                          int buffer_size_in_frames,
                                                          int num_channels)
{
    if (codec_format != expected_format)
    {
        constexpr auto next_format = get_next_codec_format(expected_format);
        if constexpr (next_format.first)
        {
            return get_sample_converter<next_format.second>(codec_format,
                    buffer_size_in_frames, num_channels);
        }

        return std::unique_ptr<BaseSampleConverter>(nullptr);
    }

    if (buffer_size_in_frames != expected_buffer_size)
    {
        constexpr auto next_buffer_size = get_next_buffer_size(expected_buffer_size);
        if constexpr (next_buffer_size.first)
        {
            return get_sample_converter<expected_format, next_buffer_size.second>
                    (codec_format, buffer_size_in_frames, num_channels);
        }

        return std::unique_ptr<BaseSampleConverter>(nullptr);
    }

    if (num_channels != expected_num_chans)
    {
        constexpr auto next_num_chans = get_next_num_channels(expected_num_chans);
        if constexpr (next_num_chans.first)
        {
            return get_sample_converter<expected_format,
                                        expected_buffer_size,
                                        next_num_chans.second>(codec_format,
                                                buffer_size_in_frames,
                                                num_channels);
        }

        return std::unique_ptr<BaseSampleConverter>(nullptr);
    }

    return std::make_unique<SampleConverter<static_cast<RaspaCodecFormat>
            (expected_format), expected_buffer_size, expected_num_chans>>();
}

} // namespace raspa

#endif // SAMPLE_CONVERSION_H_