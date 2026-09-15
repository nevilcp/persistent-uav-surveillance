---
name: commit-message
description: Writes Git commit messages using the Conventional Commits format with strict subject and body rules (50-char capitalized imperative subject, no trailing period, body wrapped at 72 explaining what and why). Use whenever writing, suggesting, reviewing, amending, rewording, or squashing a commit message, or before running git commit, even if the user doesn't mention conventions.
allowed-tools: Bash(git diff *) Bash(git status *) Bash(git log *) Bash(git show *) Bash(python3 ${CLAUDE_SKILL_DIR}/scripts/check_message.py *)
---

# Writing Git Commit Messages

## Why this matters

A commit message is written for someone reading `git log` months later, often a
teammate, often your future self. That reader can't ask what "Tweaked a few things"
meant. A good message lets them:

- Understand what changed and why, without reconstructing it from the diff
- Find and revert a specific change safely
- Build release notes from the history

Treat the history as a roadmap of how the project evolved. Every message should
make sense to someone who wasn't there.

## Ground the message in the actual diff

Before writing a message, look at what's really staged rather than inferring it
from the conversation:

```bash
git status --short
git diff --cached
git log --oneline -15
```

You're pre-approved to run these read-only commands without asking. If nothing is
staged, check unstaged changes (`git diff`) and ask the user what belongs in this
commit before writing a message — don't guess. If the diff mixes unrelated
changes (e.g. a bug fix plus a formatting pass), say so and suggest splitting into
separate commits rather than writing one message that covers both.

## Format

Use the Conventional Commits template:

```
<type>[optional scope]: <Description>

[optional body]

[optional footer(s)]
```

- **type**: one of the types below.
- **scope** (optional): the area of the codebase affected, in parentheses, e.g. `feat(auth): ...`.
- **Description**: the subject text after the colon.
- **body** (optional): explains what changed and why.
- **footer(s)** (optional): trailing metadata, such as issue references.

## Commit types

| Type | Use for |
|---|---|
| `feat` | Adding a new feature |
| `fix` | Fixing a bug |
| `refactor` | Rewriting or restructuring code without fixing a bug or adding a feature |
| `perf` | A refactor whose purpose is improving performance |
| `style` | Changes that don't affect what the code means: formatting, whitespace, missing semicolons |
| `test` | Adding missing tests or correcting existing ones |
| `docs` | Documentation only, such as the README |
| `build` | The build system or build tooling, dependencies, project version |
| `ci` | Continuous integration configuration and scripts |
| `ops` | Operational components: infrastructure, deployment, backup, recovery |
| `chore` | Miscellaneous changes that aren't a fix or feature and don't touch source or test files, e.g. editing `.gitignore` or routine dependency bumps |
| `revert` | Reverting a previous commit |

Choosing between overlapping types:

- `perf` vs `refactor`: if the motivation is speed or resource use, use `perf`.
- `ci` vs `build`: CI pipeline configuration is `ci`; build tooling is `build`.
- `build` vs `chore` for dependencies: both are valid readings of these rules.
  Follow whichever the repository's recent history uses; if there's no precedent,
  use `build` when the dependency change affects how the project builds or ships,
  and `chore` for routine housekeeping.
- If a commit genuinely needs two types, it probably should be two commits. Say so
  to the user rather than forcing one type to cover both.

## Rules

1. **Limit the subject line to 50 characters.** Count the whole first line,
   including type, scope, colon, and space. If it doesn't fit, the subject is
   carrying detail that belongs in the body, or the commit does too much.
2. **Capitalize the description.** The first word after `type(scope): ` starts
   with a capital letter: `feat: Add login rate limiting`.
3. **Don't end the subject line with a period.**
4. **Separate the subject from the body with a blank line.** Git and many tools
   treat the first line as the title, and the blank line is how they tell them apart.
5. **Wrap the body at 72 characters.**
6. **Use the body to explain what and why, not how.** The diff already shows how.
   Describe the problem, the reason for this approach, and any consequences a
   future reader should know about.
7. **Write the subject in the imperative mood,** as a command: "Add", "Fix",
   "Remove", not "Added", "Fixes", or "Removing". A quick test: the subject should
   complete the sentence "If applied, this commit will ___".

The body is optional. Skip it for changes whose subject fully explains them;
add it whenever the why isn't obvious from the subject alone.

## Avoid vague messages

Never write messages like "Tweaked a few things", "Update", "Fixes", "WIP", or
"Changes". Name the specific change instead.

## Committing from the command line

- Subject only: `git commit -m "fix: Handle empty cart at checkout"`
- Subject and body: pass `-m` twice. Git joins each `-m` as a separate paragraph,
  which produces the required blank line:
  `git commit -m "feat(auth): Add login rate limiting" -m "Body text..."`
- For bodies with several paragraphs or careful wrapping, write the message to a
  file and use `git commit -F <file>`.

## Amending

`git commit --amend` modifies the most recent commit, adding staged changes and/or
rewriting its message. Use it freely to clean up local commits before pushing.
Do not amend commits that have already been pushed to a shared repository: that
rewrites public history other people may have built on. If the commit is already
public, make a new commit instead, and warn the user before running any amend on
a pushed commit.

## Before finalizing

Run the checker on your draft, then fix anything it reports. `${CLAUDE_SKILL_DIR}`
resolves correctly regardless of whether this skill is installed at the personal,
project, or plugin level, so use it rather than a hardcoded path:

```bash
python3 ${CLAUDE_SKILL_DIR}/scripts/check_message.py <<'MSG'
<your message>
MSG
```

If the repository enforces its own commit rules (a commitlint config, a
`.gitmessage` template, or a `commit-msg` hook) and they conflict with a rule
above, follow the repository's rules, since commits that fail its hook won't go
through, and tell the user which rule you changed.

## Committing on the user's behalf

When asked to actually run `git commit` (not just draft a message), write the
finished message to a temp file and commit with `-F` rather than `-m`, since `-F`
handles multi-paragraph bodies without fighting shell quoting:

```bash
git commit -F <(cat <<'MSG'
<type>[optional scope]: <Description>

<body>
MSG
)
```

`git commit` is not in this skill's pre-approved commands, so Claude Code will
still ask for confirmation before running it — don't work around that by piping
the commit into an already-approved command. Show the drafted message before
running the command so the user can stop you if the diff was misread.

## Examples

Subject only:

```
fix(cart): Prevent negative item quantities
```

With body:

```
feat(auth): Add rate limiting to login endpoint

Repeated failed logins from one IP could be used to brute-force
passwords. Limit attempts to 5 per minute per IP and return 429
once the limit is reached.

Refs: #142
```

Rewriting bad messages:

| Bad | Good |
|---|---|
| `Tweaked a few things` | `style: Reformat user service with project linter` |
| `fixed bug.` | `fix(api): Return 404 for missing user profiles` |
| `added tests for login` | `test(auth): Add unit tests for user authentication` |
| `Updated README` | `docs: Document local setup steps in README` |
