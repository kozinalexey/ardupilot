

#ifndef __AP_HAL_F4BY_STORAGE_H__
#define __AP_HAL_F4BY_STORAGE_H__

#include <AP_HAL.h>
#include "AP_HAL_F4BY_Namespace.h"
#include <systemlib/perf_counter.h>

#define F4BY_STORAGE_SIZE 4096
#define F4BY_STORAGE_MAX_WRITE 512
#define F4BY_STORAGE_LINE_SHIFT 9
#define F4BY_STORAGE_LINE_SIZE (1<<F4BY_STORAGE_LINE_SHIFT)
#define F4BY_STORAGE_NUM_LINES (F4BY_STORAGE_SIZE/F4BY_STORAGE_LINE_SIZE)

class F4BY::F4BYStorage : public AP_HAL::Storage {
public:
    F4BYStorage();

    void init(void* machtnichts) {}
    uint8_t  read_byte(uint16_t loc);
    uint16_t read_word(uint16_t loc);
    uint32_t read_dword(uint16_t loc);
    void     read_block(void *dst, uint16_t src, size_t n);

    void write_byte(uint16_t loc, uint8_t value);
    void write_word(uint16_t loc, uint16_t value);
    void write_dword(uint16_t loc, uint32_t value);
    void write_block(uint16_t dst, const void* src, size_t n);

    void _timer_tick(void);

private:
    int _fd;
    volatile bool _initialised;
    void _storage_open(void);
    void _mark_dirty(uint16_t loc, uint16_t length);
    uint8_t _buffer[F4BY_STORAGE_SIZE] __attribute__((aligned(4)));
    volatile uint32_t _dirty_mask;
    perf_counter_t  _perf_storage;
    perf_counter_t  _perf_errors;
    const char* _mtd_file_path;
    void _mtd_write_signature(void);
};

#endif // __AP_HAL_F4BY_STORAGE_H__
