# Release Checklist

## AGPL Public Release (Tag-Driven)

1. Verify local branch is clean and up to date with `master`.
2. Run full build/tests/regressions and confirm no release blockers.
3. Confirm third-party notices are current in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
4. Choose semantic version `vMAJOR.MINOR.PATCH`.
5. Create and push tag:
   - `git tag -a vX.Y.Z -m "Release vX.Y.Z"`
   - `git push origin vX.Y.Z`
6. Confirm CI passes:
   - build workflow
   - version workflow
   - tag format guard workflow
7. Publish release notes including:
   - tag/version
   - key changes
   - migration notes
   - known issues
8. Archive release artifacts and CI `version.env` metadata.

## Python Package Release (nanoprc-py)

1. Confirm `python/pyproject.toml` version and metadata are correct.
2. Run `.github/workflows/python-wheels.yaml` and verify wheel build/test passes on Linux/macOS/Windows.
3. Run `.github/workflows/python-publish.yaml` with `repository=testpypi`.
4. Run `.github/workflows/python-verify-testpypi.yaml` (optionally pinning `version`).
5. Install from TestPyPI in a clean environment and run example smoke tests.
6. Publish to PyPI:
   - push a `vMAJOR.MINOR.PATCH` tag, or
   - run `.github/workflows/python-publish.yaml` with `repository=pypi`.
7. Confirm `.github/workflows/python-verify-pypi.yaml` passes (auto-triggered after tag publish, or manual run).
8. Confirm PyPI page, wheel availability, and `pip install nanoprc-py` on all supported OSes.
