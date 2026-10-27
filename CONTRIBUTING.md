# Contributing

Read this file before you change anything. If you work on the learning track, also read
[docs/learning_track.md](docs/learning_track.md). Coding assistants get both files through
[AGENTS.md](AGENTS.md).

## Who changes what

| Path | Changed by | How changes land |
|---|---|---|
| `learning/`, `.github/workflows/learning.yml` | learning track | pull requests from `learning/<topic>` branches |
| everything else | repo owner (janmesh-p) | stage commits on `main` |

- Stay inside your paths, even when a file you do not own is in your way.
- Need a change outside your paths, such as a new log column, a config key, or a bug fix? Open an
  issue titled `learning: <what you need>`. Say what you need, why, and the exact interface
  (column name and definition, file format, or command). The owner ships it in a later stage.
- [docs/learning_track.md](docs/learning_track.md) is the contract between the C++ core and the
  learning track. Only the owner edits it. Propose changes in an issue.

## Where the project stands

- The Stages table at the end of [README.md](README.md) says what is done. Check it every time
  you pull `main`.
- Stage commits on `main` start with `Stage <n>:`, so `git log --oneline` shows where each stage
  landed.
- Never build against a stage that is not marked done. Until it is, code against the contract in
  [docs/learning_track.md](docs/learning_track.md) and test with hand-written fixtures.

## Build and test

CI runs on Ubuntu 22.04. On macOS or Windows, use an Ubuntu VM or WSL. Run every command from the
repo root.

```bash
sudo apt install cmake ninja-build g++ libyaml-cpp-dev libgtest-dev
cmake -S fleet_core -B build -G Ninja
cmake --build build
ctest --test-dir build --output-on-failure
./build/run_sweep --config benchmarks/configs/baseline.yaml --out build/baseline \
  --check benchmarks/reference/baseline_sweep.csv
```

Every test must pass, and the last command must print `check      identical to ...`. The build
type defaults to Release.

## Rules for every change

- The same config and seed give the same run, bit for bit (`log_hash` in `summary.csv`). No
  decision may depend on wall time, thread timing, or unseeded randomness.
- Never edit `benchmarks/reference/`, and never weaken a CI check to make a change pass.
- Every number in a README or report comes from a command in this repo, written next to the
  number. The README tables are rebuilt from `benchmarks/evidence/` by
  `benchmarks/scripts/evidence.py`, and CI fails when a table and its evidence disagree.
- Do not commit generated data: run directories, datasets, or any file over 1 MB. Small test
  fixtures, model files, figures, and the benchmark CSVs in `benchmarks/evidence/` are fine.
- C++ (owner): C++17, `clang-format` 18.1.3 (see README), warnings are errors, and a test for
  every behavior change.
- Python (learning track): see [docs/learning_track.md](docs/learning_track.md).

## Git and pull requests

- Learning track: branch from `main`, name the branch `learning/<topic>`, and open a pull request
  into `main`. Never push to `main`.
- The owner merges once CI passes, with a merge commit. Not a squash: a squash merge adds
  `Co-authored-by` trailers when a pull request has more than one author, and CI rejects those.
- Keep pull requests to one topic, ideally under 500 changed lines.
- When a stage lands on `main`, rebase your branch: `git fetch origin && git rebase origin/main`,
  rerun the checks, then `git push --force-with-lease`.
- Never force-push `main` or a branch that someone else uses.
- Commit subject: imperative and at most 72 characters, for example `Add dataset loader`. The body
  says why.
- No attribution lines in commits or pull request descriptions: no `Co-Authored-By` trailers, no
  session links, no "Generated with" footers. CI fails on them.

## Writing style

Plain, short sentences. No em dashes. No hype words. Say what was measured, how, and what it does
not show. Report negative results the same way as positive ones.
