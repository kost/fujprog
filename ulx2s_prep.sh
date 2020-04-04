#!/bin/csh

ft232r_flash -p $1
ujprog -p $1 -e fprog.bin ulx2s_8e.jed
ujprog -p $1 -j flash ulx2s_8e.jed
