# Arm USB/CAN adapter identity — cited source for HOST-04 (2026-10-10)

This file excerpts operator-sourced evidence from the 2026-10-10 bring-up. The source is
`~/rebot_setup/SETUP_LOG.md`, sha256 `c697d9e2c751d07f7d4d97ffe337cd70838688277b101ec879a73170c6ae2bae`.
SETUP_LOG.md was redacted in 2026-11 (USB serial number, sudo note). Original sha256 `c697d9e2c751d07f7d4d97ffe337cd70838688277b101ec879a73170c6ae2bae`, filed copy `9b7d049c27bc9d8501287347b5a1dc61caeeca8e7f5cf86ec0b579b70a04bcb7`.
The full file is filed, unmodified except that redaction, as `evidence/operator/MOT-09/SETUP_LOG.md`. The overnight Claude Code session
filed this excerpt on the operator's instruction and added no measurement of its own.

The operator also states that the adapter is the "HDSC VID:PID 2e88:4603 from lsusb" (**operator statement**,
overnight instructions 2026-10-10).

## Kernel enumeration, session 1 (SETUP_LOG.md lines 29–31, from `journalctl -k`)

```
Oct 10 14:06:10 usb 1-7: New USB device found, idVendor=2e88, idProduct=4603
Oct 10 14:06:10 usb 1-7: Product: CDC Device / Manufacturer: HDSC / SerialNumber: <USB2CAN-serial-redacted>
Oct 10 14:06:10 cdc_acm 1-7:1.0: ttyACM0: USB ACM device
```

## Re-enumeration after the DMTool session, session 2 (SETUP_LOG.md lines 155–156)

```
15:51:48 usb 1-7: New USB device found, idVendor=2e88, idProduct=4603 (HDSC CDC Device, SN <USB2CAN-serial-redacted>)
15:51:48 cdc_acm 1-7:1.0: ttyACM0: USB ACM device
```

## Device node and default permissions (SETUP_LOG.md lines 23–25 and 148–150)

```
crw-rw---- 1 root dialout 166, 0 Oct 10 14:06 /dev/ttyACM0      (no ttyUSB*)
$ ls -l /dev/serial/by-id/
usb-HDSC_CDC_Device_<USB2CAN-serial-redacted>-if00 -> ../../ttyACM0
```

The same device, with the same serial number, came back at 15:51 as `/dev/ttyACM0` with `root:dialout 0660`.
The replug reset the permissions, and the operator ran `sudo chmod 666 /dev/ttyACM0` in both sessions as a
stop-gap.

## Summary used by HOST-04

| Field | Value |
|---|---|
| idVendor | `2e88` |
| idProduct | `4603` |
| manufacturer / product | `HDSC` / `CDC Device` |
| serial | `<USB2CAN-serial-redacted>` |
| kernel driver / node | `cdc_acm` / `/dev/ttyACM0` (tty subsystem) |
| default node permissions | `root:dialout 0660` |
| what it is | Damiao USB2CAN serial bridge (dm-serial, 921600 baud) to the B601-DM CAN bus |
