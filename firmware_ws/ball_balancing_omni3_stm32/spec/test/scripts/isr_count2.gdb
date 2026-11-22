target remote :3333
break _ZN2bb12control_tick11tick_isr_cbEPKv
commands
  silent
  printf "ISR\n"
  cont
end
break _ZN2bb16tick_timing_test9update_cbEv
commands
  silent
  printf "UPD\n"
  cont
end
cont
