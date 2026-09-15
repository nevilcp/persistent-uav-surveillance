#!/usr/bin/env python3
"""Check a commit message against the commit-message skill's rules.

Reads the message from stdin (or a file path argument), prints each problem,
and exits 1 if any are found.
"""

import re
import sys

TYPES = (
    "feat",
    "fix",
    "refactor",
    "perf",
    "style",
    "test",
    "docs",
    "build",
    "ci",
    "ops",
    "chore",
    "revert",
)
HEADER = re.compile(r"^(?P<type>[a-z]+)(\((?P<scope>[^()]+)\))?!?: (?P<desc>.+)$")
SUBJECT_MAX = 50
BODY_MAX = 72
NON_IMPERATIVE = re.compile(r"^\w+(ed|ing)\b|^\w+[^s]s\b", re.IGNORECASE)


def check(message: str) -> list[str]:
    lines = [ln for ln in message.splitlines() if not ln.startswith("#")]
    while lines and not lines[-1].strip():
        lines.pop()
    if not lines or not lines[0].strip():
        return ["Message is empty."]

    problems = []
    subject = lines[0]

    if len(subject) > SUBJECT_MAX:
        problems.append(
            f"Subject is {len(subject)} characters; limit is {SUBJECT_MAX}."
        )
    if subject.endswith("."):
        problems.append("Subject ends with a period.")

    match = HEADER.match(subject)
    if not match:
        problems.append("Subject doesn't match '<type>[(scope)]: <Description>'.")
    else:
        if match["type"] not in TYPES:
            problems.append(
                f"Unknown type '{match['type']}'. Use one of: {', '.join(TYPES)}."
            )
        desc = match["desc"]
        if not desc[0].isupper():
            problems.append("Description should start with a capital letter.")
        if NON_IMPERATIVE.match(desc):
            problems.append(
                f"'{desc.split()[0]}' may not be imperative; use a command "
                "form like 'Add' or 'Fix' (ignore if it already is)."
            )

    if len(lines) > 1:
        if lines[1].strip():
            problems.append("Missing blank line between subject and body.")
        for number, line in enumerate(lines[2:], start=3):
            if len(line) > BODY_MAX and not re.match(r"^\S+$", line):
                problems.append(
                    f"Body line {number} is {len(line)} characters; "
                    f"wrap at {BODY_MAX}."
                )

    return problems


def main() -> int:
    if len(sys.argv) > 1:
        with open(sys.argv[1], encoding="utf-8") as handle:
            message = handle.read()
    else:
        message = sys.stdin.read()
    problems = check(message)
    for problem in problems:
        print(f"- {problem}")
    if not problems:
        print("OK")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
