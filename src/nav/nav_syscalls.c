/* Bounded libc support for integer scanf/printf only. Algorithms have no heap.
 * No syscalls touch a UART: the explicit asynchronous ring owns all logging. */
#ifdef __arm__
#include <stddef.h>
#include <stdint.h>
#include <errno.h>
#include <sys/stat.h>
static uint64_t libc_heap[1024];
static ptrdiff_t libc_used;
void *_sbrk(ptrdiff_t increment)
{
 if(increment< -libc_used || increment>(ptrdiff_t)sizeof libc_heap-libc_used){errno=ENOMEM;return (void *)-1;}
 void *p=(char *)libc_heap+libc_used;libc_used+=increment;return p;
}
int _getpid(void){return 1;}
int _kill(int pid,int sig){(void)pid;(void)sig;errno=EINVAL;return -1;}
int _close(int fd){(void)fd;errno=EBADF;return -1;}
int _read(int fd,char *p,int n){(void)fd;(void)p;(void)n;errno=EBADF;return -1;}
int _write(int fd,const char *p,int n){(void)fd;(void)p;(void)n;errno=EBADF;return -1;}
int _lseek(int fd,int offset,int whence){(void)fd;(void)offset;(void)whence;errno=ESPIPE;return -1;}
int _fstat(int fd,struct stat *s){(void)fd;s->st_mode=S_IFCHR;return 0;}
int _isatty(int fd){(void)fd;return 1;}
#endif
