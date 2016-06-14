#include <types.h>
#include <kern/errno.h>
#include <kern/unistd.h>
#include <kern/wait.h>
#include <lib.h>
#include <syscall.h>
#include <current.h>
#include <proc.h>
#include <thread.h>
#include <addrspace.h>
#include <copyinout.h>
#include <mips/trapframe.h>


int sys_fork(struct trapframe *tf, pid_t *retval) {
  struct proc *fork_proc = proc_create_runprogram(curproc->p_name);
  DEBUG(DB_SYSCALL,"SysFork: fork_proc'd\n");
  if(fork_proc == NULL) {
    return ENPROC;
  }

  fork_proc->p_pid = curproc->pid;
  as_copy(curproc_getas(), &(fork_proc->p_addrspace));
  if(fork_proc->p_addrspace == NULL) {
    proc_destroy(fork_proc);
    return ENOMEM;
  }
  struct trapframe *forked_tf = kmalloc(sizeof(struct trapframe));
  if(forked_tf == NULL) {
    proc_destroy(fork_proc);
    return ENOMEM;
  }

  memcpy(forked_tf, tf, sizeof(struct trapframe));
  int errno = thread_fork(curthread->t_name, fork_proc, (void *)enter_forked_process, forked_tf, 0);
  if (errno) {
    proc_destroy(fork_proc);
    kfree(forked_tf);
    forked_tf = NULL;
    return errno;
  }

  *retval = fork_proc->pid;

  return 0;
}


  /* this implementation of sys__exit does not do anything with the exit code */
  /* this needs to be fixed to get exit() and waitpid() working properly */

void sys__exit(int exitcode) {

  struct addrspace *as;
  struct proc *p = curproc;
  /* for now, just include this to keep the compiler from complaining about
     an unused variable */
  (void)exitcode;

  DEBUG(DB_SYSCALL,"Syscall: _exit(%d)\n",exitcode);

  KASSERT(curproc->p_addrspace != NULL);
  as_deactivate();
  /*
   * clear p_addrspace before calling as_destroy. Otherwise if
   * as_destroy sleeps (which is quite possible) when we
   * come back we'll be calling as_activate on a
   * half-destroyed address space. This tends to be
   * messily fatal.
   */
   as = curproc_setas(NULL);
   as_destroy(as);

  /* detach this thread from its process */
  /* note: curproc cannot be used after this call */
   proc_remthread(curthread);

  /* if this is the last user process in the system, proc_destroy()
     will wake up the kernel menu thread */
   proc_destroy(p);

   thread_exit();
  /* thread_exit() does not return, so we should never get here */
   panic("return from thread_exit in sys_exit\n");
 }


/* stub handler for getpid() system call                */
 int
 sys_getpid(pid_t *retval)
 {
  *retval = curproc->pid;
  return(curproc->pid);
}

/* stub handler for waitpid() system call                */

int
sys_waitpid(pid_t pid,
 userptr_t status,
 int options,
 pid_t *retval)
{
  int exitstatus;
  int result;

  /* this is just a stub implementation that always reports an
     exit status of 0, regardless of the actual exit status of
     the specified process.   
     In fact, this will return 0 even if the specified process
     is still running, and even if it never existed in the first place.

     Fix this!
  */

     if (options != 0) {
      return(EINVAL);
    }
  /* for now, just pretend the exitstatus is 0 */
    exitstatus = 0;
    result = copyout((void *)&exitstatus,status,sizeof(int));
    if (result) {
      return(result);
    }
    *retval = pid;
    return(0);
  }

