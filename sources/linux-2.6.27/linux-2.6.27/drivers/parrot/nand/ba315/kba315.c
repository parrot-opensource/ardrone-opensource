/*
 *
 * description  BA315 controller operations queue implementation
 *
 *      author  Gregor Boirie <gregor.boirie@parrot.com>
 *        date  01-Sep-2008
 *
 *        $Id: kba315.c,v 1.6 2009-04-21 12:37:22 gboirie Exp $
 *
 *       Copyright (C) 2008 Parrot S.A.
 */

#include <linux/kthread.h>
#include <linux/freezer.h>

#include "kba315.h"

static struct task_struct* kba315d_task;
static LIST_HEAD(kba315_work_queue);
static DECLARE_WAIT_QUEUE_HEAD(kba315_wait);
static DEFINE_SPINLOCK(kba315_lock);

static
int
kba315_queue_empty(void)
{
    int ret;

    spin_lock(&kba315_lock);
    ret = list_empty(&kba315_work_queue);
    spin_unlock(&kba315_lock);

    return ret;
}

#ifdef CONFIG_PM

static
int
kba315d_wait_workset(void)
{
    do {
        int ret = wait_event_interruptible_exclusive(kba315_wait,
                                                     kthread_should_stop() ||
                                                     ! kba315_queue_empty() ||
                                                     freezing(current));
        if (ret && ! freezing(current))
            return ret;
    } while (try_to_freeze());

    return 0;
}

#else    /* CONFIG_PM */

static
int
kba315d_wait_workset(void)
{
    return wait_event_interruptible_exclusive(kba315_wait,
                                              kthread_should_stop() ||
                                              ! kba315_queue_empty());
}

#endif  /* CONFIG_PM */

kba315_op_t* const* kba315_ops;

/**
 * kba315_run_work - perform submitted work unit
 * @running:    the work unit to run
 * @next:       the next unit to run
 *
 * Description:
 * Perform work related actions and mark assigned set as completed if needed.
 *
 * Notes:
 * Following assumptions are made.
 *
 * All work units of the same set follow each others within the queue.
 * This is mandatory since we want to support work sets/units allocated on
 * the stack. As we run in another thread, work units must exist as long as the
 * work queue iterator did not process them. Indeed, waiters may only be woken
 * up after all work units of the same set have been treated (to prevent
 * waiters from freeing them while they are still present in the queue).
 */
static
struct completion*
kba315_run_work(struct kba315_work* running, struct kba315_work* next)
{
    struct kba315_workset*  set;

    BUG_ON(! running);
    BUG_ON(! running->set);

    set = running->set;

    BUG_ON(set->type >= max_op);

    /*
     * if there was no previous execution error for this set, perform work
     */
    if (likely(! set->ret || set->ret == -EUCLEAN))
        set->ret = (*kba315_ops[set->type])(running, next);

    /*
     * decrease work units completion count:
     * if more work to do, bail out
     */
    if (--set->uncomplete)
        return 0;

    /*
     * work units are all completed, tell the caller he should wake up waiter
     */
    return &set->done;
}

#ifdef STAT
#include <mach/hardware.h>
#include<asm/io.h>
static
void
kba315_init_survey(void)
{
    writeb(1, IO_ADDRESS(VERSATILE_GPIO0_BASE) + 0x400);
}
#else
static void kba315_init_survey(void) {}
#endif

static
int
kba315d_thread(void* __unused)
{
	struct sched_param param = { .sched_priority = 1 };

	sched_setscheduler(current, SCHED_FIFO, &param);
    set_freezable();
    kba315_init_survey();

    while (1) {
        int ret = kba315d_wait_workset();

        /* we are required to exit */
        if (unlikely(kthread_should_stop()))
            return 0;

        /* we were interrupted by some signals: flush them and get back to sleep */
        if (unlikely(ret)) {
            /* FIXME: flush needed ? */
            flush_signals(kba315d_task);
            continue;
        }


        /* work units were submitted */
        {
            LIST_HEAD(work_head);
            struct kba315_work* work;
            struct completion*  done = 0;

            /*
             * initialize the private work queue with what is currently
             * registered within the main work queue
             */
            spin_lock(&kba315_lock);

            BUG_ON(list_empty(&kba315_work_queue));

            __list_splice(&kba315_work_queue, &work_head, work_head.next);
            INIT_LIST_HEAD(&kba315_work_queue);

            spin_unlock(&kba315_lock);


            /*
             * process the private work queue
             */
            list_for_each_entry(work, &work_head, node) {
                struct kba315_work* next;

                /*
                 * last iteration caused the completion of a workset:
                 * now that we do not need related work entries anymore,
                 * we can safely wake up attached waiter
                 */
                if (done)
                    complete(done);

                if (list_is_last(&work->node, &work_head)) {
                    /*
                     * we reached the last work unit of our private queue:
                     * let's see if someone else did submit some more work in our
                     * back, and append the additional work at the tail of our
                     * private queue...
                     */
                    spin_lock(&kba315_lock);
                    if (list_empty(&kba315_work_queue))
                        next = 0;
                    else {
                        __list_splice(&kba315_work_queue, work_head.prev, &work_head);
                        INIT_LIST_HEAD(&kba315_work_queue);
                        next = list_entry(work->node.next, struct kba315_work, node);
                    }
                    spin_unlock(&kba315_lock);
                }
                else
                    next = list_entry(work->node.next, struct kba315_work, node);

                /*
                 * perform requested operation
                 */
                done = kba315_run_work(work, next);
            }

            if (done)
                complete(done);
        }
    }
}

/**
 * kba315_wait_complete - wait for completion of a set of work units
 * @set:    units of work related set to wait completion for
 * @units:  units of work to attach to set
 * @count:  number og submitted units of work
 *
 * Description:
 * blocks till all submitted units of work have completed or an error occured
 */
void
kba315_wait_complete(struct kba315_workset* set, struct kba315_work* units, size_t count)
{
    LIST_HEAD(work_head);

    BUG_ON(! set);
    BUG_ON(! set->mtd);
    BUG_ON(set->type >= max_op);
    BUG_ON(! units);
    BUG_ON(! count);

    /* work set failsafe init */
    init_completion(&set->done);
    set->uncomplete = count;

    /*
     * create a chained list of work units with regard to the order they were
     * submitted
     */
    while (count--) {
        units[count].set = set;
        INIT_LIST_HEAD(&units[count].node);
        list_add(&units[count].node, &work_head);
    }

    /*
     * append chained work units to the main workqueue
     * and wake BA315 work queue daemon if needed
     */
    spin_lock(&kba315_lock);
    {
        int do_wait = list_empty(&kba315_work_queue);
        __list_splice(&work_head, &kba315_work_queue, kba315_work_queue.next);

        if (do_wait)
            wake_up_interruptible_sync(&kba315_wait);
    }
    spin_unlock(&kba315_lock);

    /* 
     * wait for all work units to be completed
     */
    wait_for_completion(&set->done);
}

int __init
kba315_init(kba315_op_t* const* ops)
{
    BUG_ON(! ops);
    kba315_ops = ops;

    kba315d_task = kthread_run(kba315d_thread, NULL, "kba315d");
    if (IS_ERR(kba315d_task))
        return PTR_ERR(kba315d_task);

    return 0;
}

void
kba315_fini(void)
{
    kthread_stop(kba315d_task);
}

/* vim:ts=4:sw=4:et:syn=c */
