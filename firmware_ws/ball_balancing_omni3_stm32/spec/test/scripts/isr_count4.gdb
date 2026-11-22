target remote :3333
break _ZN2bb10debug_uart5writeEPKcj
commands
  silent
  printf "DBGWRITE %p %u\n", $r0, $r1
  finish
  printf "DBGWRITE done\n"
  cont
end
break _ZN2bb12control_tick11tick_isr_cbEPKv
commands
  silent
  printf "ISR\n"
  cont
end
cont
