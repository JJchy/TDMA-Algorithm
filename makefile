setting:
	#gcc -std=c99 setting.c -o setting -lm
	gcc -std=c99 setting_packet.c -o setting_packet -lm

clean:
	rm setting
