# 0090. Commit messages and constant access follow fixed conventions

- Status: accepted
- Date: 2026-09-15

## Context

Without a fixed convention, two things drift. Commit bodies vary in shape, so a
history of thousands of commits cannot be scanned by eye or by script for the
cause and the verification of a change. And shared constants get re-exported
under local aliases, so a grep for the real name misses the call sites and a
change to the shared value looks isolated when it is not.

These were recorded in the per-session memory store and had no home in the repo,
because they are conventions rather than a config value with an `x-journal`.

## Decision

Commit messages are in English and imperative. The body is structured as cause,
key changes, and verification, so each commit states why it was made and how it
was checked. No `Co-Authored-By:` trailers are added (some older commits predate
this and carry them).

Shared constants are imported under their real name at every call site. There is
no local alias (`X = SHARED_X`) and no `as` re-export. A reader searching for the
shared name finds every use.

## Consequences

- A commit history is scannable for cause and verification without opening diffs.
- A shared constant cannot hide behind a local name, so a change to it is
  auditable by grep.

## History

- Memory-only until now: `feedback_commit_style` and `feedback_no_alias_constants`
  in the per-session store. No repo commit introduced them.

## Cross-references

- 0069 owns the config governance these constants live under; 0087 owns the test
  invocation conventions.
