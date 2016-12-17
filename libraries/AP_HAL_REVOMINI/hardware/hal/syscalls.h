int _kill(int pid, int sig);

void _exit(int status);
int _getpid(void);
char* get_stack_top(void);
caddr_t _sbrk(int nbytes);
int _open(const char *path, int flags, ...);
int _close(int fd);
int _fstat(int fd, struct stat *st);
int _isatty(int fd);
int isatty(int fd);
int _lseek(int fd, off_t pos, int whence);
unsigned char getch(void);
int _read(int fd, char *buf, size_t cnt);
void putch(unsigned char c);

void cgets(char *s, int bufsize);
int _write(int fd, const char *buf, size_t cnt);
char *fgets(char *s, int bufsize, void *f);
void clock_gettime(uint32_t a1, void *a2);


#pragma pack(push, 1)
union udid {
        uint32_t        serial[3];
        uint8_t  data[12];
};
#pragma pack(pop)

int get_board_serial(uint8_t *serialid);
int val_read(void *dest, volatile const void *src, int bytes);

