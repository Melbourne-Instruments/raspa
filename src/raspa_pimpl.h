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
 * @brief Definition of class RaspaPimpl, which abstracts low-level access
 *        to a RTDM Audio device from userspace in scenarios where the driver
 *        directly interfaces with the CODECS. Provides access to RT driver
 *        through a typical callback registration service. This class provides a
 *        private implementation of the api found in raspa.h
 * @copyright 2020-2023 Melbourne Instruments, Australia
 */
#ifndef RASPA_PIMPL_H_
#define RASPA_PIMPL_H_

#include <sched.h>
#include <sys/mman.h>
#include <sys/sysinfo.h>
#include <errno.h>

#pragma GCC diagnostic ignored "-Wunused-parameter"

#include <rtdm/rtdm.h>
#include <xenomai/init.h>
#include <cobalt/pthread.h>
#include <cobalt/time.h>
#include <cobalt/sys/ioctl.h>

#pragma GCC diagnostic pop

#include <cstdlib>
#include <cstdint>

#include "raspa/raspa.h"
#include "driver_config.h"
#include "raspa_error_codes.h"
#include "sample_conversion.h"

#ifdef RASPA_DEBUG_PRINT
#include <stdio.h>
#endif

/*
 * This variable is defined by xenomai init. It is used to index the number
 * of command line arguments passed to xenomai. Since these arguments are passed
 * manually, this variable is incremented in the init() function.
 */
extern int optind;

namespace {

// Delay in milliseconds to wait for audio driver to close and stop its thread.
constexpr int CLOSE_DELAY_US = 500000;

// Delay in microseconds for a stop request to be processed
constexpr int STOP_REQUEST_DELAY_US = 10000;

constexpr int THREAD_CREATE_DELAY_US = 10000;

// Number of kernel memory pages raspa allocates
constexpr int NUM_PAGES_KERNEL_MEM = 16;

// Num of audio buffers.
constexpr int NUM_BUFFERS = 2;

// Driver parameter definitions
constexpr int DRIVER_PARAM_PATH_LEN = 100;
constexpr int DRIVER_PARAM_VAL_STR_LEN = 25;

/**
 * driver versions
 */
constexpr int REQUIRED_DRIVER_VERSION_MAJ = 0;
constexpr int REQUIRED_DRIVER_VERSION_MIN = 2;

// Manually passed "commandline args" to xenomai
constexpr char XENOMAI_ARG_APP_NAME[]     = "raspa";
constexpr char XENOMAI_ARG_CPU_AFFINITY[] = "--cpu-affinity=3";

// SPI Audio and Control buffer sizes in bytes
// Note: The size of the SPI buffer is dictated by the output data, as this is significantly
// larger than the input data
constexpr int RTDM_NUM_OUTPUT_CHANNELS_PER_FPGA = 20;
constexpr int RTDM_BUFFER_SIZE = 128;
constexpr int SPI_BUFFER_OUTPUT_AUDIO_SAMPLES_SIZE = (RTDM_NUM_OUTPUT_CHANNELS_PER_FPGA * RTDM_BUFFER_SIZE * 24 / 8);
constexpr int SPI_BUFFER_TRANSFER_SIZE = (SPI_BUFFER_OUTPUT_AUDIO_SAMPLES_SIZE + (2*4));

// Constants related to the GPIO timing pin
#ifdef RASPA_PROCESSING_GPIO_TIMING_PIN
constexpr char MEM_DEV_NAME[]                   = "/dev/mem";
constexpr uint BCM2711_PI4_PERIPHERAL_BASE      = 0xFE000000;
constexpr uint GPIO_REGISTER_BASE               = 0x200000;
constexpr uint GPIO_SET_OFFSET                  = 0x1C;
constexpr uint GPIO_CLR_OFFSET                  = 0x28;
constexpr uint GPIO_TIMING_PIN                  = 16;
constexpr uint GPIO_TIMING_INITIAL_BUFFER_SKIPS = 45454;
#endif
}

namespace raspa {

/**
 * @brief Entry point for the real time thread
 * @param data Contains pointer to an instance of RaspaPimpl
 * @return nullptr
 */
static void* raspa_pimpl_task_entry(void* data);

/**
 * @brief Interface to a audio rtdm driver that directly interfaces with a
 *        coded. It handles all low level access and is responsible for querying
 *        various audio parameters from the driver, starting and stopping of
 *        real time audio callbacks, interleaving/deinterleaving and conversion
 *        of audio samples. All public functions follow the same api as defined
 *        raspa.h
 */
class RaspaPimpl
{
public:
    RaspaPimpl() : _driver_buffer(nullptr),
                   _driver_buffer_audio_in{nullptr, nullptr},
                   _driver_buffer_audio_out{nullptr, nullptr},
                   _driver_cv_in{nullptr},
                   _driver_cv_out{nullptr},
                   _kernel_buffer_mem_size(0),
                   _user_audio_in{nullptr},
                   _user_audio_out{nullptr},
                   _device_handle(-1),
                   _interrupts_counter(0),
                   _buf_idx(0),
                   _stop_request_flag(false),
                   _break_on_mode_sw(false),
                   _sample_rate(0),
                   _num_codec_chans(0),
                   _num_input_chans(0),
                   _num_output_chans(0),
                   _buffer_size_in_frames(0),
                   _buffer_size_in_samples(0),
                   _codec_format(RaspaCodecFormat::INT24_LJ),
                   _device_opened(false),
                   _user_buffers_allocated(false),
                   _mmap_initialized(false),
                   _task_started(false),
                   _res_get_audio_info(0),
                   _user_data(nullptr),
                   _user_callback(nullptr)
    {}

    ~RaspaPimpl()
    {
        _cleanup();
    }

    int init()
    {
        /*
         * Fake command line arguments to pass to xenomai_init(). For some
         * obscure reasons, xenomai_init() crashes if argv is allocated here on
         * the stack, so we alloc it beforehand.
         */
        int argc = 2;
        auto argv = new char* [argc + 1];
        for (int i = 0; i < argc; i++)
        {
            argv[i] = new char[32];
        }
        argv[argc] = nullptr;

        std::snprintf(argv[0],
                      sizeof(XENOMAI_ARG_APP_NAME),
                      XENOMAI_ARG_APP_NAME);

        std::snprintf(argv[1],
                      sizeof(XENOMAI_ARG_CPU_AFFINITY),
                      XENOMAI_ARG_CPU_AFFINITY);

        optind = 1;

        xenomai_init(&argc, (char* const**) &argv);
        _kernel_buffer_mem_size = NUM_PAGES_KERNEL_MEM * getpagesize();

        for (int i = 0; i < argc; i++)
        {
            free(argv[i]);
        }
        free(argv);


        auto res = mlockall(MCL_CURRENT | MCL_FUTURE);
        if (res < 0)
        {
            return res;
        }

        _res_get_audio_info = _get_audio_info_from_driver();

#ifdef RASPA_PROCESSING_GPIO_TIMING_PIN
        // Get a pointer to the GPIO registers
        uint32_t *gpio_port = reinterpret_cast<uint32_t *>(_mmap_bcm_register_base(GPIO_REGISTER_BASE));
        if (gpio_port)
        {
            // Set the pin as an input first, and then as an output
            // Note: Commented out as this should already be an output pin as set by the FPGA Config utility
            //*(gpio_port + (GPIO_TIMING_PIN / 10)) &= ~(7 << ((GPIO_TIMING_PIN % 10) * 3));
            //usleep(1000);
            //*(gpio_port + (GPIO_TIMING_PIN / 10)) |= (1 << ((GPIO_TIMING_PIN % 10) * 3));
            //usleep(1000);

            // Set the set/clr registers pointers
            gpio_set_reg = gpio_port + (GPIO_SET_OFFSET / sizeof(uint32_t));
            gpio_clr_reg = gpio_port + (GPIO_CLR_OFFSET / sizeof(uint32_t));
        }
#endif
        return RASPA_SUCCESS;
    }

    int open(int buffer_size,
             RaspaProcessCallback process_callback,
             void* user_data, unsigned int debug_flags)
    {
        _buffer_size_in_frames = buffer_size;

        auto res = _check_driver_compatibility();
        if (res < 0)
        {
            return res;
        }

        if (_res_get_audio_info < 0)
        {
            return _res_get_audio_info;
        }

        _init_sample_converter();
        if(!_sample_converter)
        {
            return -RASPA_EINVALID_BUFFSIZE;
        }
        _sample_converter.get()->start_spi_error_logging();

        if (debug_flags == 1 && RASPA_DEBUG_SIGNAL_ON_MODE_SW == 1)
        {
            _break_on_mode_sw = true;
        }

        res = _open_device();
        if (res < 0)
        {
            return res;
        }

        res = _get_driver_buffers();
        if (res < 0)
        {
            _cleanup();
            return res;
        }

        _init_driver_buffers();

        res = _init_user_buffers();
        if (res < 0)
        {
            _cleanup();
            return res;
        }

        _user_data = user_data;
        _interrupts_counter = 0;
        _buf_idx = 0;
        _user_callback = process_callback;

        return RASPA_SUCCESS;
    }

    int start_realtime()
    {
        // Initialize RT task
        _task_started = false;
        struct sched_param rt_params = {.sched_priority = RASPA_PROCESSING_TASK_PRIO};
        pthread_attr_t task_attributes;
        __cobalt_pthread_attr_init(&task_attributes);

        pthread_attr_setdetachstate(&task_attributes, PTHREAD_CREATE_JOINABLE);
        pthread_attr_setinheritsched(&task_attributes, PTHREAD_EXPLICIT_SCHED);
        pthread_attr_setschedpolicy(&task_attributes, SCHED_FIFO);
        pthread_attr_setschedparam(&task_attributes, &rt_params);

        // Force affinity on first thread
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(3, &cpuset);
        auto res = pthread_attr_setaffinity_np(&task_attributes,
                                               sizeof(cpu_set_t), &cpuset);
        if (res < 0)
        {
            _cleanup();
            _raspa_error_code.set_error_val(RASPA_ETASK_AFFINITY, res);
            return -RASPA_ETASK_AFFINITY;
        }

        // Create rt thread
        res = __cobalt_pthread_create(&_processing_task, &task_attributes,
                                      &raspa_pimpl_task_entry, this);
        if (res < 0)
        {
            _cleanup();
            _raspa_error_code.set_error_val(RASPA_ETASK_CREATE, res);
            return -RASPA_ETASK_CREATE;
        }

        _task_started = true;
        usleep(THREAD_CREATE_DELAY_US);

        /* After Xenomai init + RT thread creation, all non-RT threads have the
         * affinity restricted to one single core. This reverts back to the
         * default of using all cores */
        CPU_ZERO(&cpuset);
        for (int i = 0; i < 3; i++)
        {
            CPU_SET(i, &cpuset);
        }
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

        res = __cobalt_ioctl(_device_handle, RASPA_PROC_START);
        if (res < 0)
        {
            _raspa_error_code.set_error_val(RASPA_ETASK_START, res);
            return -RASPA_ETASK_START;
        }

        return RASPA_SUCCESS;
    }

    /**
     * @brief The main rt audio loop
     */
    void rt_loop()
    {
#ifdef RASPA_PROCESSING_GPIO_TIMING_PIN
        uint gpio_timing_initial_buffer_skips = 0;
#endif
#ifdef RASPA_SPI_LOOPBACK_TESTING
        int count = 0;
        int num_errors = 0;
        int count_error[300];
        int32_t in_errors[300];
        int32_t out_errors[300];     
#endif

        while (true)
        {
            auto res = __cobalt_ioctl(_device_handle, RASPA_IRQ_WAIT);
            if (res < 0)
            {
                // TODO: think how to handle this. Error here means something *bad*,
                // so we might want to de-register the driver, cleanup and signal
                // user someway.            
                // Melbourne Instruments: Just wait again for the next IRQ, don't process the data
                continue;
            }

#ifdef RASPA_PROCESSING_GPIO_TIMING_PIN
            // Don't use the GPIO timing pin for an intial number of buffers
            if (gpio_timing_initial_buffer_skips >= GPIO_TIMING_INITIAL_BUFFER_SKIPS) {
                *gpio_set_reg = (1 << GPIO_TIMING_PIN);
            }         
#endif

            _buf_idx = res;
            if (_break_on_mode_sw && _interrupts_counter > 1)
            {
                pthread_setmode_np(0, PTHREAD_WARNSW, NULL);
                _break_on_mode_sw = 0;
            }

            // clear driver buffers if stop is requested
            if (_stop_request_flag)
            {
                _clear_driver_buffers();
            }
            else
            {       
#ifdef RASPA_SPI_LOOPBACK_TESTING
                // Check that the received data is the same as the transmitted
                // Check the first two words, and the last
                int32_t *pin = _driver_buffer_audio_in[_buf_idx];
                int32_t *pout = _driver_buffer_audio_out[_buf_idx];
                int32_t in1 = _left_rotate_int32(*pin);
                int32_t in2 = _left_rotate_int32(*(pin + 1));
                int32_t in3 = _left_rotate_int32(*(pin + (SPI_BUFFER_OUTPUT_AUDIO_SAMPLES_SIZE/4) - 1));
                int32_t out1 = *pout;
                int32_t out2 = *(pout + 1);
                int32_t out3 = *(pout + (SPI_BUFFER_OUTPUT_AUDIO_SAMPLES_SIZE/4) - 1);
                if (in1 != out1) {
                    // Save the error
                    count_error[num_errors] = count;
                    in_errors[num_errors] = in1;
                    out_errors[num_errors] = out1;
                    num_errors++;
                }
                if (in2 != out2) {
                    // Save the error
                    count_error[num_errors] = count;
                    in_errors[num_errors] = in2;
                    out_errors[num_errors] = out2;
                    num_errors++;
                }
                if (in3 != out3) {
                    // Save the error
                    count_error[num_errors] = count;
                    in_errors[num_errors] = in3;
                    out_errors[num_errors] = out3;
                    num_errors++;
                }
#endif
                _sample_converter->codec_format_to_float32n(_user_audio_in,
                                                            _driver_buffer_audio_in[_buf_idx]);
                _user_callback(_user_audio_in, _user_audio_out, _user_data);
                _sample_converter->float32n_to_codec_format(
                        _driver_buffer_audio_out[_buf_idx], _user_audio_out);

#ifdef RASPA_SPI_LOOPBACK_TESTING
                count++;
                if ((count >= 100) && num_errors) {
                    printf("num_errors %d\n", num_errors);
                    for (int i=0; i<num_errors; i++) {
                        printf("COUNT: %d\n", count_error[i]);
                        printf("OUT_ERR: %X\n", out_errors[i]);
                        printf("IN_ERR: %X\n", in_errors[i]);
                    }
                    break;
                }
#endif
            }
#ifdef RASPA_PROCESSING_GPIO_TIMING_PIN
            // Don't use the GPIO timing pin for an intial number of buffers
            if (gpio_timing_initial_buffer_skips >= GPIO_TIMING_INITIAL_BUFFER_SKIPS) {
                *gpio_clr_reg = (1 << GPIO_TIMING_PIN);
            }
            else
               gpio_timing_initial_buffer_skips++;            
#endif
            _interrupts_counter++;
        }
        pthread_exit(nullptr);
    }

#ifdef RASPA_SPI_LOOPBACK_TESTING 
    int32_t _left_rotate_int32(int32_t val) 
    {
        return (val << 1) | ((val >> 31) & 0x01);
    } 
#endif

    float get_sampling_rate()
    {
        return (float) _sample_rate;
    }

    int get_num_input_channels()
    {
        return _num_input_chans;
    }

    int get_num_output_channels()
    {
        return _num_output_chans;
    }

    const char* get_error_msg(int code)
    {
        return _raspa_error_code.get_error_text(code);
    }

    uint32_t get_gate_values()
    {
        return *_driver_cv_in;
    }

    void set_gate_values(uint32_t cv_gates_out)
    {
        *_driver_cv_out = cv_gates_out;
    }

    RaspaMicroSec get_time()
    {
        RaspaMicroSec time = 0;
        struct timespec tp;
        auto res = __cobalt_clock_gettime(CLOCK_MONOTONIC, &tp);
        if (res == 0)
        {
            time = (RaspaMicroSec) tp.tv_sec * 1000000 + tp.tv_nsec / 1000;
        }

        return time;
    }

    int64_t get_samplecount()
    {
        return _interrupts_counter * _buffer_size_in_frames;
    }

    RaspaMicroSec get_output_latency()
    {
        // TODO - really crude approximation
        if (_sample_rate > 0)
        {
            return (_buffer_size_in_samples * 1000000) / _sample_rate;
        }

        return 0;
    }

    int close()
    {
        _sample_converter.get()->stop_spi_error_logging();
        printf("\nmiso errors %d   mosi errors %d\n", miso_errors, mosi_errors_1 );

        _stop_request_flag = true;

        // Wait sometime for periodic task to send mute command to device
        usleep(STOP_REQUEST_DELAY_US);

        auto res = __cobalt_ioctl(_device_handle, RASPA_PROC_STOP);

        // Wait for driver to stop current transfers.
        usleep(CLOSE_DELAY_US);
        
        if (res < 0)
        {
            _cleanup();
            _raspa_error_code.set_error_val(RASPA_ETASK_STOP, res);
            return -RASPA_ETASK_STOP;
        }

        return _cleanup();
    }


protected:
    /**
     * @brief Read driver parameter
     * @param param The parameter to read
     * @return integer value of the parameter upon success, linux error code
     *         otherwise.
     */
    int _read_driver_param(const char* param)
    {
        int rtdm_file;
        char path[DRIVER_PARAM_PATH_LEN];
        char value[DRIVER_PARAM_VAL_STR_LEN];

        std::snprintf(path, DRIVER_PARAM_PATH_LEN,
                      RASPA_MODULE_PARAMETERS_PATH"/%s", param);

        rtdm_file = __cobalt_open(path, O_RDONLY);

        if (rtdm_file < 0)
        {
            return rtdm_file;
        }

        if (read(rtdm_file, value, DRIVER_PARAM_VAL_STR_LEN) == -1)
        {
            return rtdm_file;
        }

        __cobalt_close(rtdm_file);

        return atoi(value);
    }

    /**
     * @brief Checks compatibility of raspa configuration with that of the
     *        driver.
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _check_driver_compatibility()
    {
        auto buffer_size = _read_driver_param("audio_buffer_size");
        auto major_version = _read_driver_param("audio_ver_maj");
        auto minor_version = _read_driver_param("audio_ver_min");

        if (buffer_size < 0 || major_version < 0 || minor_version < 0)
        {
            return -RASPA_EPARAM;
        }

        if (_buffer_size_in_frames != buffer_size)
        {
            return -RASPA_EBUFFSIZE;
        }

        if (REQUIRED_DRIVER_VERSION_MAJ != major_version)
        {
            return -RASPA_EVERSION;
        }


        if (REQUIRED_DRIVER_VERSION_MIN != minor_version)
        {
            return -RASPA_EVERSION;
        }

        return RASPA_SUCCESS;
    }

    /**
     * @brief Get the various info from the drivers parameter
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _get_audio_info_from_driver()
    {
        _sample_rate = _read_driver_param("audio_sampling_rate");
        _num_input_chans = _read_driver_param("audio_input_channels");
        _num_output_chans = _read_driver_param("audio_output_channels");
        auto codec_format = _read_driver_param("audio_format");

        if (_sample_rate < 0 || _num_output_chans < 0
            || _num_output_chans < 0 || codec_format < 0)
        {
            return -RASPA_EPARAM;
        }

        if(codec_format < static_cast<int>(RaspaCodecFormat::INT24_LJ)
        || codec_format >= static_cast<int>(RaspaCodecFormat::NUM_CODEC_FORMATS))
        {
            _raspa_error_code.set_error_val(RASPA_ECODEC_FORMAT, codec_format);
            return -RASPA_ECODEC_FORMAT;
        }
        _codec_format = static_cast<RaspaCodecFormat>(codec_format);

        _num_codec_chans = (_num_input_chans > _num_output_chans)
                           ? _num_input_chans : _num_output_chans;

        return RASPA_SUCCESS;
    }

    /**
     * @brief Open the rtdm device.
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _open_device()
    {
        _device_opened = false;
        _device_handle = __cobalt_open(RASPA_DEVICE_NAME, O_RDWR);
        if (_device_handle < 0)
        {
            _raspa_error_code.set_error_val(RASPA_EDEVICE_OPEN, _device_handle);
            return -RASPA_EDEVICE_OPEN;
        }

        _device_opened = true;
        return RASPA_SUCCESS;
    }

    /**
     * @brief Close the rtdm device
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _close_device()
    {
        if (_device_opened)
        {
            auto res = __cobalt_close(_device_handle);
            _device_opened = false;

            if (res < 0)
            {
                _raspa_error_code.set_error_val(RASPA_EDEVICE_CLOSE, res);
                return -RASPA_EDEVICE_CLOSE;
            }
        }

        return RASPA_SUCCESS;
    }

    /**
     * @brief Get audio buffers from the driver using mmap.
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _get_driver_buffers()
    {
        _mmap_initialized = false;
        _driver_buffer = (int32_t*) __cobalt_mmap(NULL, _kernel_buffer_mem_size,
                                                  PROT_READ | PROT_WRITE,
                                                  MAP_SHARED, _device_handle,
                                                  0);
        if (_driver_buffer == MAP_FAILED)
        {
            _raspa_error_code.set_error_val(RASPA_ENOMEM, errno);
            return -RASPA_ENOMEM;
        }

        _mmap_initialized = true;
        return RASPA_SUCCESS;
    }

    /**
     * @brief Unmaps the acquired audio buffers from the driver.
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _release_driver_buffers()
    {
        if (_mmap_initialized)
        {
            auto res = munmap(_driver_buffer, _kernel_buffer_mem_size);
            _mmap_initialized = false;
            if (res < 0)
            {
                _raspa_error_code.set_error_val(RASPA_EUNMAP, res);
                return -RASPA_EUNMAP;
            }
        }

        return RASPA_SUCCESS;
    }

    /**
     * @brief Initialize the input and output double buffers from the driver.
     */
    void _init_driver_buffers()
    {
        _buffer_size_in_samples = _buffer_size_in_frames * _num_codec_chans;

        _driver_buffer_audio_out[0] = _driver_buffer;
        _driver_buffer_audio_in[0] = _driver_buffer_audio_out[0] + ((SPI_BUFFER_TRANSFER_SIZE*2)/4);

        _driver_buffer_audio_out[1] = _driver_buffer_audio_in[0] + ((SPI_BUFFER_TRANSFER_SIZE*2)/4);
        _driver_buffer_audio_in[1] = _driver_buffer_audio_out[1] + ((SPI_BUFFER_TRANSFER_SIZE*2)/4);

        _driver_cv_out = (uint32_t*) _driver_buffer_audio_in[1] + ((SPI_BUFFER_TRANSFER_SIZE*2)/4);
        _driver_cv_in = _driver_cv_out + 1;
    }

    /**
     * @brief Create audio buffers for the user.
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _init_user_buffers()
    {
        _user_buffers_allocated = false;
        int res = posix_memalign((void**) &_user_audio_in, 16,
                                 _buffer_size_in_samples * sizeof(float))
                  || posix_memalign((void**) &_user_audio_out, 16,
                                    _buffer_size_in_samples * sizeof(float));

        std::fill_n(_user_audio_in, _buffer_size_in_samples, 0);
        std::fill_n(_user_audio_out, _buffer_size_in_samples, 0);

        if (res < 0)
        {
            _raspa_error_code.set_error_val(RASPA_EUSER_BUFFERS, res);
            return -RASPA_EUSER_BUFFERS;
        }

        _user_buffers_allocated = true;
        return RASPA_SUCCESS;
    }

    /**
     * @brief Free up the allocated user buffers.
     */
    void _free_user_buffers()
    {
        if (_user_buffers_allocated)
        {
            free(_user_audio_in);
            free(_user_audio_out);
            _user_buffers_allocated = false;
        }
    }

    /**
     * @brief Initialize the sample converter instance.
     */
    void _init_sample_converter()
    {
        _sample_converter = get_sample_converter(_codec_format,
                                                 _buffer_size_in_frames,
                                                 _num_codec_chans);
    }

    /**
     * @brief De init the sample converter instance.
     */
    void _deinit_sample_converter()
    {
        _sample_converter.reset();
    }

    /**
     * @brief Stops the real time task.
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _stop_rt_task()
    {
        if (_task_started)
        {
            auto res = pthread_cancel(_processing_task);
            res |= __cobalt_pthread_join(_processing_task, NULL);
            _task_started = false;
            if (res < 0)
            {
                _raspa_error_code.set_error_val(RASPA_ETASK_CANCEL, res);
                return -RASPA_ETASK_CANCEL;
            }
        }

        return RASPA_SUCCESS;
    }

    /**
     * @brief Free up memory, delete instances and stops the rt thread
     * @return RASPA_SUCCESS upon success, different raspa error code otherwise
     */
    int _cleanup()
    {
        // The order is very important. Its the reverse order of instantiation,
        auto res = _stop_rt_task();
        _free_user_buffers();
        res |= _release_driver_buffers();
        res |= _close_device();

        _deinit_sample_converter();
        return res;
    }

    /**
     * @brief Clear the driver buffers
     */
    void _clear_driver_buffers()
    {
        if (!_mmap_initialized)
        {
            return;
        }

        for (int i = 0; i < ((SPI_BUFFER_TRANSFER_SIZE*2)/4); i++)
        {
            _driver_buffer_audio_out[0][i] = 0;
            _driver_buffer_audio_out[1][i] = 0;
            _driver_buffer_audio_in[0][i] = 0;
            _driver_buffer_audio_in[1][i] = 0;
        }
    }

#ifdef RASPA_PROCESSING_GPIO_TIMING_PIN
    void *_mmap_bcm_register_base(off_t register_base)
    {
        uint32_t *addr = nullptr;
        int mem_fd;

        // Open the memory device
        mem_fd = ::open(MEM_DEV_NAME, O_RDWR|O_SYNC);
        if (mem_fd < 0)
        {
            // Error opening the device
            return nullptr;
        }

        // Map the register base
        addr = reinterpret_cast<uint32_t *>(::mmap(NULL, 4096, (PROT_READ|PROT_WRITE),
                                    MAP_SHARED, mem_fd,
                                    (BCM2711_PI4_PERIPHERAL_BASE + register_base)));
        ::close(mem_fd);

        // Was the map successful?
        if (addr == MAP_FAILED)
        {
            // Error mapping the register base
            return nullptr;
        }
        return addr;
    }    
#endif

    // Pointers for driver data
    int32_t* _driver_buffer;
    int32_t* _driver_buffer_audio_in[NUM_BUFFERS];
    int32_t* _driver_buffer_audio_out[NUM_BUFFERS];
    uint32_t* _driver_cv_in;
    uint32_t* _driver_cv_out;
    size_t _kernel_buffer_mem_size;

    // User buffers for audio
    float* _user_audio_in;
    float* _user_audio_out;

    // device handle identifier
    int _device_handle = -1;

    // counter to count the number of interrupts
    int _interrupts_counter;

    // flag to denote which buffer is being used
    int _buf_idx;

    // flag to denote that a stop has been requested
    bool _stop_request_flag;

    // flag to break on mode switch occurence
    bool _break_on_mode_sw;

    // audio buffer parameters
    int _sample_rate;
    int _num_codec_chans;
    int _num_input_chans;
    int _num_output_chans;
    int _buffer_size_in_frames;
    int _buffer_size_in_samples;
    RaspaCodecFormat _codec_format;
    std::unique_ptr<BaseSampleConverter> _sample_converter;

    // initialization phases
    bool _device_opened;
    bool _user_buffers_allocated;
    bool _mmap_initialized;
    bool _task_started;

    // result of get_audio_info_from_driver()
    int _res_get_audio_info;

    // rt task data
    void* _user_data;
    RaspaProcessCallback _user_callback;
    pthread_t _processing_task;

    // Error code helper class
    RaspaErrorCode _raspa_error_code;

    // GPIO registers
#ifdef RASPA_PROCESSING_GPIO_TIMING_PIN
    volatile uint32_t *gpio_set_reg;
    volatile uint32_t *gpio_clr_reg;
#endif
};

static void* raspa_pimpl_task_entry(void* data)
{
    auto raspa_pimpl = static_cast<RaspaPimpl*>(data);
    raspa_pimpl->rt_loop();

    // To suppress warnings
    return nullptr;
}

} // namespace raspa

#endif // RASPA_PIMPL_H
