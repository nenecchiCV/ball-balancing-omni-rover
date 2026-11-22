# Atomic snapshot of runner/shell/UART state during a stall window.
# Reads every object in a single halted OpenOCD session.

proc rdw {a} {
    set out [capture "mdw [format 0x%08x $a]"]
    regexp {: ([0-9a-fA-F]+)} $out -> v
    return [expr "0x$v"]
}

proc rdb {a} {
    set out [capture "mdb [format 0x%08x $a]"]
    regexp {: ([0-9a-fA-F]+)} $out -> v
    return [expr "0x$v"]
}

proc dump_thread {name base} {
    set pended [rdw [expr $base + 8]]
    set state  [rdb [expr $base + 12]]
    set prio   [rdb [expr $base + 13]]
    set psp    [rdw [expr $base + 0x50]]
    echo [format "%s @0x%08x pended_on=0x%08x state=0x%02x prio=%d psp=0x%08x" $name $base $pended $state $prio $psp]
    echo [format "  stack@psp:"]
    ocd_mdw $psp 16
}

init
halt

echo "=== runner counters ==="
echo [format "active_    =0x%08x" [rdw 0x20000fd0]]
echo [format "phase_     =%d" [rdw 0x20000fcc]]
echo [format "led_phase_ =%d" [rdw 0x20000fc4]]
echo [format "loop_iters_=%d" [rdw 0x20000fc0]]
echo [format "upd_calls_ =%d" [rdw 0x20000fbc]]
echo [format "overruns_  =%d" [rdw 0x20000fc8]]

echo "=== tick_sem @0x20000500 ==="
echo [format "wait_q.head=0x%08x tail=0x%08x count=%d limit=%d" [rdw 0x20000500] [rdw 0x20000504] [rdw 0x20000508] [rdw 0x2000050c]]

echo "=== runner_mutex @0x200004dc ==="
echo [format "wait_q.head=0x%08x tail=0x%08x owner=0x%08x lock_count=%d" [rdw 0x200004dc] [rdw 0x200004e0] [rdw 0x200004e4] [rdw 0x200004e8]]

echo "=== threads ==="
dump_thread "z_main_thread " 0x20000b38
dump_thread "shell_thread  " 0x20000658

echo "=== LPUART1 ==="
echo [format "CR1=0x%08x ISR=0x%08x RDR=0x%08x" [rdw 0x40008000] [rdw 0x4000801c] [rdw 0x40008024]]

echo "=== TIM6 ==="
echo [format "CR1=0x%08x DIER=0x%08x SR=0x%08x CNT=%d" [rdw 0x40001000] [rdw 0x4000100c] [rdw 0x40001010] [rdw 0x40001024]]

echo "=== NVIC ==="
echo [format "ISER1=0x%08x ISPR1=0x%08x IABR1=0x%08x" [rdw 0xe000e104] [rdw 0xe000e204] [rdw 0xe000e304]]

echo "=== shell transport ctx @0x20000960 (rx_ringbuf region) ==="
ocd_mdw 0x20000960 32
echo "=== shell ctx @0x200010b8 (signal_event region) ==="
ocd_mdw 0x200010b8 32

resume
exit
