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
#include <synch.h>
#include <kern/fcntl.h>
#include <vfs.h>
#include <limits.h>

int sys_fork(struct trapframe *tf, pid_t *retval) {
  struct proc *fork_proc = proc_create_runprogram(curproc->p_name);
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
  update_proc_parent_pid(fork_proc->pid, curproc->pid);
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
  save_exit_code(p->pid, exitcode);

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
  return(0);
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
    lock_acquire(all_procs_lock);
    struct proc_status* curr = get_proc_status(pid);
    if(curr->p_pid != curproc->pid) {
      return ECHILD;
    } else if (curr == NULL) {
      return ESRCH;
    }

    while(curr->status != 0) {
      cv_wait(proc_cv, all_procs_lock);
    }
    exitstatus = curr->exit_code;
    lock_release(all_procs_lock);

  /* for now, just pretend the exitstatus is 0 */
    
    result = copyout((void *)&exitstatus,status,sizeof(int));
    if (result) {
      return(result);
    }
    *retval = pid;
    return(0);
  }

  int sys_execv(char *program, char **args) {
    int arg_count = 0;
    struct addrspace *as;
    struct addrspace *curr_addrspace = curproc_getas();
    struct vnode *v;
    vaddr_t entrypoint, stackptr;
    int result;

    if(NULL == program) {
      return EFAULT;
    }

    while(NULL != args[arg_count]) {
      if(1024 < strlen(args[arg_count])) {
        return E2BIG;
      }
      arg_count += 1;
    }

    if(arg_count > ARG_MAX) {
      return E2BIG;
    }

    char *new_prog = kmalloc(sizeof(char*) * (strlen(program) + 1));
    if (NULL == new_prog) {
      return ENOMEM;
    }
    
    char **new_prog_args = kmalloc(sizeof(char) * (arg_count + 1));
    if (NULL == new_prog_args) {
      kfree(new_prog);
      return ENOMEM;
    }

    for(int i = 0; i < arg_count; i++) {
      new_prog_args[i] = kmalloc(sizeof(char) * (strlen(args[i]) + 1));
      if(NULL == new_prog_args[i]) {
        kfree(new_prog);
        for(int x = 0; x < i; x++) {
          kfree(new_prog_args[x]);
        }
        kfree(new_prog_args);
        return ENOMEM;
      }
      int curr_res = copyinstr((userptr_t) args[i], new_prog_args[i], strlen(args[i]) + 1, NULL);
      if(curr_res) {
        kfree(new_prog);
        for(int x = 0; x < i; x++) {
          kfree(new_prog_args[x]);
        }
        kfree(new_prog_args);
        return curr_res;
      }
    }

    new_prog_args[arg_count] = NULL;
    int curr_res = copyinstr((userptr_t) program, new_prog, strlen(program) + 1, NULL);
    if(curr_res != 0) {
      kfree(new_prog);
      for(int i = 0; i < arg_count; i++) {
        kfree(new_prog_args[i]);
      }
      kfree(new_prog_args);
      return ENOEXEC;
    }

    result = vfs_open(new_prog, O_RDONLY, 0, &v);
    if (result) {
      return result;
    }

    as = as_create();
    if (as == NULL) {
      vfs_close(v);
      return ENOMEM;
    }

    curproc_setas(as);
    as_activate();

    result = load_elf(v, &entrypoint);
    if (result) {
      vfs_close(v);
      curproc_setas(curr_addrspace);
      return result;
    }

    vfs_close(v);

    result = as_define_stack(as, &stackptr);
    if (result) {
      curproc_setas(curr_addrspace);
      return result;
    }

    while((stackptr % 8) != 0) {
      stackptr -= 1;
    }
    
    vaddr_t args_pointer[arg_count + 1];
    for(int i = arg_count - 1; i >= 0; i--) {
      stackptr -= strlen(new_prog_args[i]) + 1;

      int curr_res = copyoutstr(new_prog_args[i], (userptr_t) stackptr, strlen(new_prog_args[i]) + 1, NULL);
      if(curr_res != 0) {
        kfree(new_prog);
        for(int i = 0; i < arg_count; i++) {
          kfree(new_prog_args[i]);
        }
        kfree(new_prog_args);
        return curr_res;
      }
      args_pointer[i] = stackptr;
    }

    while((stackptr % 4) != 0) {
      stackptr -= 1;
    }

    args_pointer[arg_count] = 0;
    for(int i = arg_count; i >= 0; i--) {
      //int x = arg_count - i;
      stackptr -= ROUNDUP(sizeof(vaddr_t), 4);
      int curr_res = copyout(&args_pointer[i], (userptr_t) stackptr, sizeof(vaddr_t));
      if(curr_res != 0) {
        kfree(new_prog);
        for(int i = 0; i < arg_count; i++) {
          kfree(new_prog_args[i]);
        }
        kfree(new_prog_args);
        return curr_res;
      }
    }

    as_destroy(curr_addrspace);

  /* Warp to user mode. */
  enter_new_process(arg_count /*argc*/, (userptr_t) stackptr /*userspace addr of argv*/,
    stackptr, entrypoint);
  
  /* enter_new_process does not return. */
  panic("enter_new_process returned\n");
  return EINVAL;
}
