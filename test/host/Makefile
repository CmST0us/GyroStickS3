CC ?= gcc
CFLAGS = -std=c11 -O1 -g -Wall -Wextra -Wshadow -fsanitize=address,undefined -fno-sanitize-recover=undefined \
         -I../../components/gyl/include
GYL = ../../components/gyl

all: test_gyl test_imu_fifo test_buttons

test_gyl: test_gyl.c $(GYL)/gyl_format.c $(GYL)/gyl_codec.c $(GYL)/gyl_decode.c
	$(CC) $(CFLAGS) -o $@ $^ -lm

check: test_gyl test_imu_fifo test_buttons
	./test_gyl
	./test_imu_fifo
	./test_buttons

FIFO = ../../components/imu_fifo

test_imu_fifo: test_imu_fifo.c $(FIFO)/imu_fifo.c
	$(CC) $(CFLAGS) -I$(FIFO)/include -o $@ $^

test_buttons: test_buttons.c ../../main/btn_logic.h
	$(CC) $(CFLAGS) -I../../main -o $@ test_buttons.c

MAIN = ../../main
UIFLAGS = -std=c11 -O1 -g -Wall -Wextra -Istubs -I$(MAIN) -I../../components/gyl/include

ui_preview: ui_preview.c $(MAIN)/ui.c $(MAIN)/lcd_gfx.c $(MAIN)/lcd.h $(MAIN)/ui.h
	$(CC) $(UIFLAGS) -o $@ ui_preview.c $(MAIN)/ui.c $(MAIN)/lcd_gfx.c

previews: ui_preview
	rm -f ui_*.ppm
	./ui_preview ui

clean:
	rm -f test_gyl test_imu_fifo test_buttons ui_preview ui_*.ppm test_pages.bin test_samples.csv
