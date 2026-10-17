# Third-party sources (git submodules)

Pinned commits. After cloning the repository: `git submodule update --init`
(or clone with `--recursive`). `COLCON_IGNORE` keeps colcon from treating them as
packages; `go2_mpc` builds them through `add_subdirectory`.

| Library | Upstream | Commit | Commit date | License |
|---|---|---|---|---|
| BLASFEO | https://github.com/giaf/blasfeo | 6b182bd8c6b5358a8eadc0604dc7b830d39028b1 | 2026-10-12 | BSD-2-Clause |
| HPIPM | https://github.com/giaf/hpipm | 16a8db59f595bda642f460aa0f62e308f4b76db3 | 2026-10-03 | BSD-2-Clause |

BLASFEO's CMake writes `include/blasfeo_target.h` into its source tree at configure
time; BLASFEO's own `.gitignore` covers it, so the submodule stays clean after a build.

To update: `git -C blasfeo fetch && git -C blasfeo checkout <commit>`, rebuild and run
the tests, then commit the new submodule pointer and change the table above.
