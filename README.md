stm32_isp_gui.exe 窗口

<img width="753" height="607" alt="image" src="https://github.com/user-attachments/assets/8b47d231-dae6-4729-9ea2-acb9bbcbcfa2" />

stm32_isp.ese  命令  
适用于 STM32F103C6T6 的串口 ISP 下载工具  
下载 BIN 程序 

	stm32_isp.exe com7 115200 download led_test.bin
	[*] Open com7 @ 115200
	[*] Enter ISP
	[*] Sync 0x7F
	[*] Global erase
	[+] Erase done
	[*] Writing firmware
	Downloading... 100%
	[+] Download complete
	[+] 下载完成，自动复位运行程序
	[*] 复位并运行程序
	[*] 退出 ISP
		
下载 HEX 程序

	stm32_isp.exe com7 115200 download led_test.hex
	[*] Open com7 @ 115200
	[*] Enter ISP
	[*] Sync 0x7F
	[*] Global erase
	[+] Erase done
	[*] Writing firmware
	Downloading... 100%
	[+] Download complete
	[+] 下载完成，自动复位运行程序
	[*] 复位并运行程序
	[*] 退出 ISP
		
清空程序

	stm32_isp.exe com7 115200 clear
	[*] Open com7 @ 115200
	[*] Enter ISP
	[*] Sync 0x7F
	[*] Global erase
	[+] Erase done
	[*] 退出 ISP
