obj-m := mhz19b.o

KDIR=../output

all:
	make M=$(PWD) -C $(KDIR) modules \
		ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu-

clean:
	make M=$(PWD) -C $(KDIR) clean
	#rm -f *~

.PHONY: clean
