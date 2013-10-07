
PWD := $(shell pwd)

INSTALL := install

KERNEL_MOD_DIR := polygator

obj-m := simbank.o

ifeq ($(TARGET_DEVICE), sbg4)

simbank-objs := sbg4-base.o

KERNEL_SRC_DIR := $(KERNEL_SRC)
KERNEL_STG_DIR := $(INSTALL_MOD_PATH)

else

simbank-objs := sbpc-base.o

KERNEL_VERSION := `uname -r`
KERNEL_SRC_DIR := /lib/modules/$(KERNEL_VERSION)/build
KERNEL_STG_DIR := /

endif

CHKCONFIG	:= $(wildcard /sbin/chkconfig)
UPDATE_RCD	:= $(wildcard /usr/sbin/update-rc.d)
ifeq (,$(DESTDIR))
	ifneq (,$(CHKCONFIG))
		SYSVINIT_ADD := $(CHKCONFIG) --add simbank
	else
		ifneq (,$(UPDATE_RCD))
			SYSVINIT_ADD := $(UPDATE_RCD) simbank defaults 20 80
		endif
	endif
endif

all: modules

modules:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) modules

modules_install: install_modules

install: install_modules install_headers

install_modules:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) INSTALL_MOD_PATH=$(KERNEL_STG_DIR) INSTALL_MOD_DIR=$(KERNEL_MOD_DIR) modules_install

install_headers:
	$(INSTALL) -m 755 -d "$(DESTDIR)/usr/include/simbank"
	for header in simbank/*.h ; do \
		$(INSTALL) -m 644 $$header "$(DESTDIR)/usr/include/simbank" ; \
	done

install_sysvinit:
	$(INSTALL) -m 755 simbank.sysvinit $(DESTDIR)/etc/init.d/simbank
ifneq (,$(SYSVINIT_ADD))
	$(SYSVINIT_ADD)
endif


uninstall: uninstall_modules uninstall_headers uninstall_sysvinit

uninstall_modules:
	rm -rvf "$(DESTDIR)/lib/modules/$(KERNEL_VERSION)/$(KERNEL_MOD_DIR)"
	depmod

uninstall_headers:
	rm -rvf "$(DESTDIR)/usr/include/simbank"

uninstall_sysvinit:
	rm -fv $(DESTDIR)/etc/rc0.d/*simbank
	rm -fv $(DESTDIR)/etc/rc1.d/*simbank
	rm -fv $(DESTDIR)/etc/rc2.d/*simbank
	rm -fv $(DESTDIR)/etc/rc3.d/*simbank
	rm -fv $(DESTDIR)/etc/rc4.d/*simbank
	rm -fv $(DESTDIR)/etc/rc5.d/*simbank
	rm -fv $(DESTDIR)/etc/rc6.d/*simbank
	rm -fv $(DESTDIR)/etc/init.d/simbank

clean:
	@make -C $(KERNEL_SRC_DIR) M=$(PWD) clean
	@rm -f *~ simbank/*~
