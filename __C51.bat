@echo off
::This file was created automatically by CrossIDE to compile with C51.
C:
cd "\Users\Michael\Desktop\CrossIDE\AVR\usart\"
"C:\Users\Michael\Desktop\CrossIDE\Call51\Bin\c51.exe" --use-stdout  "C:\Users\Michael\Desktop\CrossIDE\AVR\usart\usart.c"
if not exist hex2mif.exe goto done
if exist usart.ihx hex2mif usart.ihx
if exist usart.hex hex2mif usart.hex
:done
echo done
echo Crosside_Action Set_Hex_File C:\Users\Michael\Desktop\CrossIDE\AVR\usart\usart.hex
