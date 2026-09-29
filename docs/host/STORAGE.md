# Storage decision — unmounted 1.9 TB NVMe (`K1_Storage`)

## Facts (collected read-only, 2026-09-29)

`lsblk -f`:

```
NAME        FSTYPE   FSVER LABEL            UUID                                 MOUNTPOINTS
nvme1n1
├─nvme1n1p1 vfat     FAT32                  4F78-9D8F                            /boot/efi
└─nvme1n1p2 ext4     1.0                    7d5167a0-872d-4e09-8f72-4d213e3df514  /
nvme0n1     ext4     1.0   K1_Storage       80a8057a-8426-4681-a3d8-2e56b8f9717c  (unmounted)
```

- `nvme0n1` is a whole-disk ext4 filesystem (no partition table shown by `lsblk`), labelled
  `K1_Storage`, UUID `80a8057a-8426-4681-a3d8-2e56b8f9717c`.
- It is **not mounted** anywhere and **not listed in `/etc/fstab`**.
- `/etc/fstab` only references the boot disk (`nvme1n1p1` → `/boot/efi`, `nvme1n1p2` → `/`) and
  `/swapfile`. No entry for `nvme0n1`/`K1_Storage` exists.
- Capacity: the card describes it as ~1.9 TB (device node not directly accessible from this
  sandboxed session to re-measure with `blockdev --getsize64`; size must be confirmed by the
  operator at mount time, e.g. via `sudo blockdev --getsize64 /dev/nvme0n1` or `sudo lsblk -b`).
- The label `K1_Storage` strongly suggests this filesystem was created by/for another project or
  host context ("K1"), not by `rebot_crack_vision`. **Its contents are unknown and were not
  inspected or modified.**
- Root filesystem (`/`, on `nvme1n1p2`): 468G total, 333G used, 112G available, 75% full.
- Current repo-local data footprint: `data/` = 13M, `models/` = 266M (both currently tiny; no
  storage pressure yet from this project alone).

## Options

1. **Do not mount.** Leave `K1_Storage` untouched (default/safe choice, since it may belong to
   another project — mounting or reformatting it here would risk another project's data). Keep
   datasets/rosbags/LLM weights on the root disk (`/`, 112G free) or use it as needed until space
   runs low.
2. **Mount read-only, inspect ownership first.** Operator mounts it manually (outside this
   sandbox) at a scratch path (e.g. `/mnt/k1_storage`) with `-o ro` to inspect contents and confirm
   whether it belongs to another active project before deciding further.
3. **Mount read-write for this project's storage**, via `/etc/fstab` with the `UUID=` (reversible:
   remove the fstab line and `umount` to revert), at a dedicated mountpoint (e.g.
   `/mnt/k1_storage` or `/data/k1_storage`), and point `data/`, `models/`, and rosbag storage at
   subdirectories there via symlinks or config, once ownership is confirmed safe.
4. **Repurpose/reformat.** Only if the operator confirms `K1_Storage` is decommissioned/unused —
   not recommended without explicit confirmation given the ambiguous ownership.

## Operator decision

**Pending.** Recorded here once received: whether to mount `K1_Storage`, where, with what
permissions/fstab options, and what project data (if any) should live there. If mounting is
approved, it will be carried out via the privileged path (a reviewed `privileged_request` using
`mount`/`/etc/fstab` edits with a UUID entry), which is reversible by removing the fstab line and
unmounting.
