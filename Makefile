KDIR:=/lib/modules/$(shell  uname -r)/build/ 

DEBIAN_VERSION_FILE:=/etc/debian_version
DEBIAN_DISTRO:=$(wildcard $(DEBIAN_VERSION_FILE))
CURRENT=$(shell uname -r)
MAJORVERSION=$(shell uname -r | cut -d '.' -f 1)
MINORVERSION=$(shell uname -r | cut -d '.' -f 2)
SUBLEVEL=$(shell uname -r | cut -d '.' -f 3)

ifeq ($(MAJORVERSION),5)
MDIR=kernel/drivers/tty/serial
else
ifeq ($(MAJORVERSION),4)
MDIR=drivers/tty/serial
else
ifeq ($(MAJORVERSION),3)
MDIR=drivers/tty/serial
else
ifeq ($(MAJORVERSION),2)
ifneq (,$(filter $(SUBLEVEL),38 39))
MDIR=drivers/tty/serial
else
MDIR=drivers/serial
endif
else
MDIR=drivers/serial
endif
endif
endif
endif

RM_PATH:= /lib/modules/$(shell uname -r)/$(MDIR)/ax99100.ko

obj-m +=ax99100.o
obj-m +=parport_pc.o

ax99100-objs := ax99100_spi.o ax99100_sp.o

default:
	$(RM) *.mod.c *.o *.ko .*.cmd *.symvers
	make -C $(KDIR) M=$(PWD) modules
	gcc -pthread select_BR.c -o select_BR
	gcc -pthread advanced_BR.c -o advanced_BR
	gcc -pthread gpio_99100.c -o gpio_99100
	gcc -pthread -Wall 9bit_test.c -o 9bit_test
	gcc -I /usr/include spi_test.c -o spi_test
	$(RM) *.mod.c *.o .*.cmd *.symvers
	rm -rf .tmp_version* *~
	rm -rf Module.markers modules.*	

install: default
ifeq ($(MAJORVERSION),5)
	$(MAKE) -C $(KDIR) M=$(PWD) modules_install
	cp ax99100.ko  /lib/modules/$(shell uname -r)/$(MDIR)
	su -c "/sbin/depmod -a"
	modprobe ax99100
else
	cp ax99100.ko  /lib/modules/$(shell uname -r)/kernel/$(MDIR)
	depmod -A
	chmod +x ax99100
	cp ax99100 /etc/init.d/
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	ln -s /etc/init.d/ax99100 /etc/rcS.d/Sax99100 || true
else
	ln -s /etc/init.d/ax99100 /etc/rc.d/rc3.d/Sax99100 || true  	
	ln -s /etc/init.d/ax99100 /etc/rc.d/rc5.d/Sax99100 || true
endif
	modprobe ax99100
endif

uninstall:
ifeq ($(MAJORVERSION),5)	
ifneq ($(shell lsmod | grep electrum),)
	rmmod ax99100
endif
	rm -rf $(RM_PATH)
	su -c "/sbin/depmod -a"
else
	modprobe -r ax99100
	rm -f /lib/modules/$(shell uname -r)/kernel/$(MDIR)/ax99100.*
	depmod -A
	rm -f /etc/init.d/ax99100
ifeq ($(DEBIAN_DISTRO), $(DEBIAN_VERSION_FILE))
	rm -f /etc/init.d/ax99100 /etc/rcS.d/Sax99100 || true
else
	rm -f /etc/rc.d/rc3.d/Sax99100
	rm -f /etc/rc.d/rc5.d/Sax99100
endif
endif

clean:
	$(RM) *.mod.c *.o *.o.* *.ko .*.cmd *.symvers *.o.ur-safe *.mod
	rm -rf .tmp_version* *~
	rm -rf Module.markers modules.* .cache.*
	rm -f select_BR advanced_BR gpio_99100 spi_test 9bit_test
