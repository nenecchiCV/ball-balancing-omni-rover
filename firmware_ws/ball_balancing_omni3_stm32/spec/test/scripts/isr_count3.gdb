target remote :3333
break z_impl_k_sem_give
commands
  silent
  printf "GIVE sem=%p\n", $r0
  cont
end
cont
