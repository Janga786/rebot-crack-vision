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
- Root filesystem (`/`, on `nvme1n1p2`): 468G total, 362G used, 83G available, 82% full
  (re-checked 2026-10-09; usage has grown from 75%→82% since the initial facts above as LLM
  weights and other project data accumulated).
- Current repo-local data footprint: `data/` = 14M, `models/` = 266M (both still tiny).
- LLM weights footprint: `~/models/llm` = 15G (per the LLM-03 card, which set a 40G free-space
  floor on `/` and verified 52G free at the time; current free space, 83G, remains above that
  floor).

## Options considered

1. **Do not mount.** Leave `K1_Storage` untouched (since it may belong to another project —
   mounting or reformatting it here would risk another project's data). Keep
   datasets/rosbags/LLM weights on the root disk.
2. **Mount read-only, inspect ownership first.** Operator mounts it manually (outside this
   sandbox) at a scratch path with `-o ro` to confirm whether it belongs to another active
   project before deciding further.
3. **Mount read-write for this project's storage**, via `/etc/fstab` with the `UUID=` (reversible:
   remove the fstab line and `umount` to revert), and point `data/`, `models/`, and rosbag storage
   at subdirectories there.
4. **Repurpose/reformat.** Only if the operator confirms `K1_Storage` is decommissioned/unused —
   not recommended without explicit confirmation given the ambiguous ownership.

## Operator decision (authoritative, 2026-09-30)

Option **1** was chosen. **Leave `K1_Storage` (`nvme0n1`, ext4, label `K1_Storage`) COMPLETELY
UNTOUCHED.** Do not mount it, modify it, add it to `/etc/fstab`, or use it for datasets, model
weights, or caches. No interaction with it beyond the read-only identification already performed
above. All project data, rosbags, LLM weights (`~/models/llm`), and caches stay on the existing
root filesystem (`/`, on `nvme1n1p2`).

Consequence: capacity planning for this project uses the root filesystem's free space (83G as of
2026-10-09) against the LLM-03 40G floor, not `K1_Storage`. No privileged action (mount, fstab
edit, format) was taken or is pending for `K1_Storage`.
