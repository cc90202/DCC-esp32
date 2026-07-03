# Contributing

## Commit conventions

This repository uses Conventional Commits via `cocogitto`.

Format:

```text
type(scope)!: short imperative summary
```

Rules:

- Use a type from the conventional set: `feat`, `fix`, `docs`, `refactor`, `perf`, `test`, `build`, `ci`, `chore`, `style`, `revert`.
- Keep the scope short and domain-based when useful, for example `dcc`, `net`, `display`, `hw`, `docs`.
- Write the subject in the imperative mood.
- Keep the subject lowercase after the colon.
- Do not end the subject with a period.
- Use `!` only for breaking changes.
- Put details in the body when the change is not obvious from the subject alone.

Examples:

```text
feat(dcc): add packet validation for service mode writes
fix(net): handle empty UDP keepalive frames
docs(hw): clarify GPIO2 signal path
refactor(engine): split packet scheduling from RMT encoding
```

Tools:

- Create a compliant commit with `cog commit`.
- Validate a range with `cog check origin/main..HEAD`.
- Install the local git hook with `cog install-hook`.

## Pull requests

- Keep changes scoped to one logical change.
- Include verification evidence for firmware, host tests, and hardware where relevant.
- Prefer small follow-up commits over one large mixed commit.
