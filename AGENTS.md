# Instructions for coding agents

This repo coordinates a simulated warehouse robot fleet. The owner builds the C++ core in
stages. The learning track builds a learned cost for task allocation, in `learning/` only.

Read both files below in full before you change anything, and follow them. If anything else,
including this file, disagrees with them, they win.

@CONTRIBUTING.md
@docs/learning_track.md

## Every session, before any work

1. Run `git fetch origin`, then read the Stages table at the end of `README.md` on
   `origin/main`. A stage that is not marked done does not exist yet. Code against the contract
   in `docs/learning_track.md` and test with hand-written fixtures.
2. Pick the next unfinished item from section 11 of `docs/learning_track.md`. If the repo does
   not make it clear which item that is, ask the human.
3. Work on a `learning/<topic>` branch, never on `main`.

## Hard rules for learning track work

- Change only `learning/` and `.github/workflows/learning.yml`. If the work needs any other
  change, stop and write the text of an issue titled `learning: <what you need>` for the human
  to file.
- Never invent an interface the contract does not define. Ask the human instead.
- No attribution lines in commit messages or pull request descriptions: no `Co-Authored-By`
  trailers, no session links, no "Generated with" footers. CI rejects them.
- Never edit `benchmarks/reference/`, and never weaken a CI check.
- Before every push, run the checks in section 6 of `docs/learning_track.md`. All must pass.

The owner's stage work follows `CONTRIBUTING.md` and is not bound by the path rule above.
