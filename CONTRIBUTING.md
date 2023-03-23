Contributing to esmini
======================

## User guide

https://esmini.github.io/

## Issue tracker

https://github.com/esmini/esmini/issues

Use issue tracker for reporting bugs or other problems.

### Checklist
Before creating the issue:
* Check version. esmini is constantly growing and fixing small bugs. Make sure the issue is present in the latest version.
* Check if a previous issue has been raised about the same bug.

### Report issue

When creating the issue, please include:

* Clear description of the issue, e.g. expected vs actual result
* Information for reproducing the issue, e.g. code snippet or OpenSCENARIO and OpenDRIVE file

## How to contribute

Contributions should be provided as pull request.

The esmini [license](https://github.com/esmini/esmini/blob/master/LICENSE) will apply to all contributions.

Preferably, especially for larger contributions, an issue can be created in the issue tracker (see above) to discuss the planned contribution. E.g. outlook to be accepted and sorting out potential technical or other considerations.

Code of the pull request will undergo [CI](https://github.com/esmini/esmini/actions) including tests and formatting check. Hence it's recommended to run the formatting checks and test suites locally first, see more info in sections below.

## Build esmini

See [User guide - Build guide](https://esmini.github.io/#_build_guide).

## Formatting

From esmini v2.30.0 the CI will include formatting checks, in addition to static analysis and tests. To check or apply formatting, see [User guide / For esmini developers and contributors / Formatting](https://esmini.github.io/#_formatting).

## Test

There are two sets of tests on different frameworks:
1. Unit tests (white box)
2. Smoke tests (black box)

See [User guide / For esmini developers and contributors / Test](https://esmini.github.io/#_test) for more information and how to run the test suites.

## Branch strategy

 Pull requests should be targeting the `dev` branch, which is used for integrating features and bug fixes.

 See [User guide / For esmini developers and contributors / Formatting](https://esmini.github.io/#_branch_strategy) for more info.
