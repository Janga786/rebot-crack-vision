# Environment lock (HOST-02)

The `crackvision` conda environment (created per TC-002) is pinned to two committed lock
files so it can be recreated exactly and drift can be detected automatically.

## Lock files

- `requirements/crackvision.conda-explicit.txt` — output of `conda list -n crackvision --explicit`
  (every conda-installed package, pinned to an exact build/URL).
- `requirements/crackvision.pip-freeze.txt` — output of `pip freeze` run inside the activated
  `crackvision` env (every pip-installed package, pinned to an exact version).

Both files carry a header comment recording the generation date (UTC) and source command;
`verify_lock.py` and `rebuild_env.sh` skip leading `#` lines when comparing/consuming content.

## Regenerating the lock files

After deliberately changing the environment (e.g. adding a dependency to
`requirements/requirements-core.txt` and installing it), regenerate the locks:

```bash
conda list -n crackvision --explicit > requirements/crackvision.conda-explicit.txt
./env.sh python -m pip freeze > requirements/crackvision.pip-freeze.txt
```

then re-add the header comments (generation date, source) by hand or re-run the same
commands used to create them initially (see git history of this file for the exact header
format), and commit the updated files.

## Checking for drift

```bash
./env.sh python scripts/verify_lock.py
```

- Exit `0` — the live `crackvision` env matches both lock files.
- Exit `1` — a diff between the live env and the lock files is printed.
- Exit `3` — precondition not met (conda not found, `crackvision` env missing, or lock
  files missing).

## Rebuilding the environment from scratch

`scripts/rebuild_env.sh` recreates the environment under a **new** name — it never
modifies the existing `crackvision` env.

```bash
bash scripts/rebuild_env.sh --dry-run          # print the exact commands, run nothing
bash scripts/rebuild_env.sh --name crackvision-rebuild   # actually create it
```

The script prints, and optionally runs:

```bash
conda create --name <NEW_NAME> --file requirements/crackvision.conda-explicit.txt --yes
conda run -n <NEW_NAME> python -m pip install -r requirements/crackvision.pip-freeze.txt
```

To verify the rebuilt env matches, point `verify_lock.py`'s conda env lookup logic is
hard-coded to `crackvision`; compare `<NEW_NAME>` manually with:

```bash
conda list -n <NEW_NAME> --explicit | diff - requirements/crackvision.conda-explicit.txt
conda run -n <NEW_NAME> python -m pip freeze | diff - requirements/crackvision.pip-freeze.txt
```
