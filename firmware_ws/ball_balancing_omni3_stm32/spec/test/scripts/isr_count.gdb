target remote :3333
break _ZN2bb12control_tick11tick_isr_cbEPKv
commands
  silent
  printf "HIT\n"
  cont
end
cont
