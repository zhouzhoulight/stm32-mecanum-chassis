/* Minimal newlib syscall stubs: no heap, no stdio redirection to USART1.
 * The binary host protocol owns USART1; printf must never be mixed into it. */
#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/stat.h>

void *_sbrk(ptrdiff_t increment)
{
    (void)increment;
    errno = ENOMEM;
    return (void *)-1;
}

int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; errno = EINVAL; return -1; }
int _close(int fd) { (void)fd; errno = EBADF; return -1; }
int _read(int fd, char *p, int n) { (void)fd; (void)p; (void)n; errno = EBADF; return -1; }
int _write(int fd, const char *p, int n) { (void)fd; (void)p; (void)n; errno = EBADF; return -1; }
int _lseek(int fd, int off, int whence) { (void)fd; (void)off; (void)whence; errno = ESPIPE; return -1; }
int _fstat(int fd, struct stat *s) { (void)fd; s->st_mode = S_IFCHR; return 0; }
int _isatty(int fd) { (void)fd; return 1; }
