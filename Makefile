.PHONY: all setup test quick
all setup test quick:
	$(MAKE) -C fish_mujoco $@
