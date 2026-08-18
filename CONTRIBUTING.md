# Contributing to nanoPRC

Thanks for your interest in contributing.

## Contribution Process

1. Fork the repository and create a topic branch.
2. Make focused changes and include tests/build fixes when relevant.
3. Open a pull request against `main`.
4. Ensure required CI checks pass.
5. Address maintainer review feedback.

## Review and Merge Policy

- Contributions from non-maintainers must be submitted through pull requests.
- Pull requests require maintainer approval before merge.
- The maintainer may use direct pushes for urgent maintenance.

## Legal Requirements

By submitting a pull request, you confirm that:

1. You are legally entitled to submit the contribution.
2. The contribution is your original work, or you have rights to submit it.
3. You agree to the contributor legal terms in [CLA.md](CLA.md), including copyright assignment to the project owner.

Pull requests that do not accept the contributor legal terms are not eligible for merge.

The repository enforces this via a required GitHub Actions check on pull requests. Contributors must check the legal attestation checkbox in the PR description template.

## Coding Expectations

- Keep changes scoped and readable.
- Preserve existing style unless a file-wide reformat is intentionally part of the change.
- Add or update documentation when behavior changes.
- **Before opening a PR, run the C99 conformance gate:** `python tests/internal/check_c99.py`
  (exit 0 = clean). The core library and the C demos are strict C99, but MSVC's C mode silently
  accepts constructs C99 removed — so a green Windows build is not evidence of conformance, and
  violations surface only in CI on Linux/macOS. The script parses every first-party `.c` file with
  a real Clang/GCC frontend at `-std=c99`; it finds a compiler on PATH, or on Windows uses any
  Visual Studio installation's bundled `clang-tidy` (VS2019 or VS2022, any edition). It needs a
  configured CMake build directory for the generated headers — pass `--build-dir` if yours is not
  `build/`.

## Security

Do not disclose security vulnerabilities in public issues. Contact the maintainer directly for responsible disclosure.
