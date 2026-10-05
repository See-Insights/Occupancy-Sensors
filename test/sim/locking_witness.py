#!/usr/bin/env python3
"""Deterministic wait-graph witness, not an execution of Device OS binaries.
Models v6.4.1's one-slot queue and PMIC lock, using real host threads.
A queued Wakeup is consumed while the application holds PMIC; another Update
fills the queue before the application's thermal ReloadConfig enqueue.
"""
import queue
import threading

def witness(enqueue_under_lock):
    events = queue.Queue(maxsize=1)
    wire = threading.Lock()
    app_holds = threading.Event()
    pm_waiting = threading.Event()
    enqueue_now = threading.Event()
    app_done = threading.Event()
    pm_done = threading.Event()
    events.put('Wakeup')
    def app():
        wire.acquire()
        app_holds.set()
        enqueue_now.wait()
        if not enqueue_under_lock:
            wire.release()
        events.put('ReloadConfig')  # CONCURRENT_WAIT_FOREVER
        if enqueue_under_lock:
            wire.release()
        app_done.set()
    def power_manager():
        app_holds.wait()
        assert events.get() == 'Wakeup'
        pm_waiting.set()
        with wire:  # initDefault()/handleUpdate(): PMIC(true)
            pass
        events.get()  # Only this consumer can free the event queue.
        pm_done.set()
    a = threading.Thread(target=app, daemon=True)
    p = threading.Thread(target=power_manager, daemon=True)
    a.start(); p.start()
    assert pm_waiting.wait(1)
    events.put_nowait('Update')  # ISR update() succeeds while PM is blocked.
    enqueue_now.set()
    completed = app_done.wait(0.1)
    if enqueue_under_lock:
        assert not completed and not pm_done.is_set() and events.full()
        # External teardown only: neither firmware thread can perform this get.
        assert events.get_nowait() == 'Update'
    else:
        assert completed
    a.join(1); p.join(1)
    assert not a.is_alive() and not p.is_alive()

witness(True)
witness(False)
print('locking witness: enqueue under PMIC lock deadlocks; release-before-enqueue control completes')
