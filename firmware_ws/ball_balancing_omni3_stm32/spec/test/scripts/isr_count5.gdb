target remote :3333
break z_arm_pendsv
commands
  silent
  printf "PENDSV\n"
  cont
end
break _ZN2bb12control_tick11tick_isr_cbEPKv
commands
  silent
  printf "ISR\n"
  cont
end
cont
