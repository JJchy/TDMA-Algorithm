setting:
	gcc -std=c99 setting_packet.c -o setting_packet -lm

new:
	gcc -std=c99 New_setting.c -o New_setting -lm

2-hop:
	gcc -std=c99 2_hop_setting.c -o 2_hop_setting -lm

clean:
	rm setting
