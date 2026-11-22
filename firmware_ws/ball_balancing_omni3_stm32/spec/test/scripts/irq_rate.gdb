set pagination off
set $cnt = 0
break uart_stm32_isr
commands
silent
set $cnt = $cnt + 1
cont
end
# sample for ~3s then report
shell sleep 0
cont
