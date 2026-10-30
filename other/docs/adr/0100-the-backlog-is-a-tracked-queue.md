# 0100. The backlog is a tracked queue, and it is the one place a list of open work lives

- Status: accepted (2026-10-30)
- Date: 2026-10-30

## Context

ADR 0097 made tracked prose own only the present it can be checked against, and
called a checkbox list of open work in a tracked file a defect, because it goes
stale with nothing failing when the present changes. The rule held: no tracked
file carries a status list today, and `task docs:check` enforces the path half.

It left the work itself with nowhere to be recorded. The sessions are working
notes under the ignored `other/docs/internal` tree: a fresh clone gets none of
them, and a finding survives only as long as the person who had the session
remembers it. Two sessions of September produced the same list twice.

The queue is the one artifact whose purpose is to be incomplete, so the
stale-status objection does not apply to it in the same way: an item says what
is left, not what is true, and closing it is a move to `done/`.

## Options considered

- (a) Keep the rule absolute: no backlog anywhere, only ADRs and working notes.
- (b) A tracked backlog per repository, with a fixed shape and a monthly close.
- (c) GitHub Issues, with `docs/` for the docs only.

## Decision

(b), and (c) stays open if the owner ever moves the queue there.

The shape follows the docs-architect convention: `backlog/inbox.md` for raw
captures, `backlog/vtitan.md` with `doing`, `next`, `later` and `someday`, and
`backlog/done/YYYY-MM.md` for what closed. `doing` holds one or two items and
`next` three to five, so the file stays a queue and not a pile.

What keeps 0097 intact:

- An item says what is missing and where it came from. It does not claim a state
  of the code. "The rule is untested for jumps" is a task; "the rule works" is
  not.
- A measured number cited by an item already lives in an ADR, and the item links
  the ADR instead of restating the number as if the queue were its source.
- Closing an item is a move to `done/` with one line, not a paragraph of history.
- `task docs:check` keeps applying to every tracked file. The backlog is prose
  and its paths are checked like any other.

## Consequences

- The queue is versioned and a fresh clone sees it. The sessions stay ignored.
- 0097's sentence about checkbox lists now reads as applying to docs that
  describe the present; the backlog describes what is absent, and 0099 plus this
  ADR remain the tracked record of what is true.
- The rest of the docs-architect tree is not adopted by this ADR: frontmatter on
  the 101 ADRs, `guides/` and `methodology/` folders, `templates/` and the
  linter in CI. Moving `adr/` files or rewriting their headers is a separate pass
  that needs its own approval.
- A backlog item is not a commitment. Items in `later` and `someday` are pruned
  monthly rather than archived.
