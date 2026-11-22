target remote :3333
break *0x08003742
commands
  silent
  printf "LOCK\n"
  cont
end
break *0x08003766
commands
  silent
  printf "UPDCALL\n"
  cont
end
cont
