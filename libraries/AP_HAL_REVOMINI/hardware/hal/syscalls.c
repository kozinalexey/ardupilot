/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @file syscalls.c
 * @brief Low level system routines used by Newlib for basic I/O and
 * memory allocation.
 */

#include "stm32f4xx.h"
#include <reent.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <syscalls.h>

/* _end is set in the linker command file */
extern caddr_t _end;


int _kill(int pid, int sig)
{
	pid = pid; sig = sig; /* avoid warnings */
	errno = EINVAL;
	return -1;
}

void _exit(int status)
{
	status = status; /* avoid warnings */
	while(1) {;}
}

int _getpid(void)
{
	return 1;
}

char* get_stack_top(void)
{
	return (char*) __get_MSP();
}

/*
 * sbrk -- changes heap size size. Get nbytes more
 *         RAM. We just increment a pointer in what's
 *         left of memory on the board.
 */
caddr_t _sbrk(int nbytes) {
    static caddr_t heap_ptr = NULL;
    caddr_t        base;

    if (heap_ptr == NULL) {
        heap_ptr = (caddr_t)&_end;
    }

    uint32_t top = (unsigned int)get_stack_top();

    if ( top > (unsigned int)heap_ptr // there is place in stack
        || top < 0x20000000 ) //       or stack not in RAM
    {
        base = heap_ptr;
        heap_ptr += nbytes;
        return (base);
    } else {
        return ((caddr_t)-1);
    }
}

int _open(const char *path, int flags, ...) {
    return 1;
}

int _close(int fd) {
    return 0;
}

int _fstat(int fd, struct stat *st) {
    st->st_mode = S_IFCHR;
    return 0;
}

int _isatty(int fd) {
    return 1;
}

int isatty(int fd) {
    return 1;
}

int _lseek(int fd, off_t pos, int whence) {
    return -1;
}

unsigned char getch(void) {
    return 0;
}


int _read(int fd, char *buf, size_t cnt) {
    *buf = getch();

    return 1;
}

void putch(unsigned char c) {
}

void cgets(char *s, int bufsize) {
    char *p;
    int c;
    int i;

    for (i = 0; i < bufsize; i++) {
        *(s+i) = 0;
    }

    p = s;

    for (p = s; p < s + bufsize-1;) {
        c = getch();
        switch (c) {
        case '\r' :
        case '\n' :
            putch('\r');
            putch('\n');
            *p = '\n';
            return;

        case '\b' :
            if (p > s) {
                *p-- = 0;
                putch('\b');
                putch(' ');
                putch('\b');
            }
            break;

        default :
            putch(c);
            *p++ = c;
            break;
        }
    }
    return;
}

int _write(int fd, const char *buf, size_t cnt) {
    size_t i;

    for (i = 0; i < cnt; i++)
        putch(buf[i]);

    return cnt;
}

char *fgets(char *s, int bufsize, void *f) {
    cgets(s, bufsize);
    return s;
}

void clock_gettime(uint32_t a1, void *a2) { return; } 

int val_read(void *dest, volatile const void *src, int bytes)
{

        int i;

        for (i = 0; i < bytes / 4; i++) {
                *(((volatile unsigned *)dest) + i) = *(((volatile unsigned *)src) + i);
        }

        return i * 4;
}


#define UDID_START              0x1FFF7A10
int get_board_serial(uint8_t *serialid)
{
        const volatile uint32_t *udid_ptr = (const uint32_t *)UDID_START;
        union udid id;
        val_read((uint32_t *)&id, udid_ptr, sizeof(id));


        /* Copy the serial from the chips non-write memory and swap endianess */
        serialid[0] = id.data[3];   serialid[1] = id.data[2];  serialid[2] = id.data[1];  serialid[3] = id.data[0];
        serialid[4] = id.data[7];   serialid[5] = id.data[6];  serialid[6] = id.data[5];  serialid[7] = id.data[4];
        serialid[8] = id.data[11];   serialid[9] = id.data[10];  serialid[10] = id.data[9];  serialid[11] = id.data[8];

        return 0;
}
