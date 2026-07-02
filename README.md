# nanoPRC

[![build](https://github.com/mvrhel/nanoPRC/actions/workflows/build.yaml/badge.svg)](https://github.com/mvrhel/nanoPRC/actions/workflows/build.yaml)

## License

nanoPRC is licensed under the GNU Affero General Public License v3.0.
See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for third-party attribution and license summaries.
See https://nanoprc.org for more details.

## Commercial Licensing
For commercial integrations, closed-source desktop software, or embedded enterprise platform applications where AGPLv3 compliance is not viable, a proprietary B2B OEM Commercial License program is available. Please contact sales@cascadiavoxel.com for custom licensing terms and architecture verification bundles.

## Contributing

- Contribution guide: [CONTRIBUTING.md](CONTRIBUTING.md)
- Contributor legal terms and copyright assignment: [CLA.md](CLA.md)

## Building

Ensure all submodules are checked out:

```bash
git submodule update --init --recursive
```

Then, generate the build system using CMake:

```bash
mkdir build
cd build
cmake ..
```

Finally, build the project with the generated build system. You may specify your desired build system by selecting a generator via the `-G` flag when running `cmake`. For example:

```bash
cmake -G "Visual Studio 16 2019" ..
```

The build process in the case of windows is done by opening the solution file in the build directory and building the projects.

Executables are built to the `bin` directory and libraries to the `lib` directory from within the build directory.

In the case of Linux, your milage may vary. 
See https://wiki.libsdl.org/SDL3/README-linux#build-dependencies for the dependencies needed for the demo viewer and your flavor of Linux.  The dependencies for just the library are very limited. You will need to make sure you have cmake installed. After the cmake .. command above, you will run make in the build directory to compile the code.  The viewer has been succesfully built and run on WSL.

On macOS, the process is the same but you should not need to do anything special with SDL dependencies. You will need to make sure you have cmake and likely the xcode tools. After the cmake .. command above you will run make in the build directory. The viewer has been built and run on Intel and Apple silicon machines.

###  quick_start
nanoPRC includes several source-code demonstrations that showcase how to use the core library.

| Location | Description |
|--------|-------------|
| demos/quick_start | A simple example on how to open, parse, and make use of the PRC contents |
| demos/viewer | A basic 3D interactive viewer to display PRC models |
| demos/json_export | A utility to create a JSON dump from a PRC model |
| demos/stl_export | A utility to create binary STL mesh files from a PRC model |
| demos/obj_export | A utility to create OBJ files from a PRC model with MTL and textures |

### Deterministic Unzipped-Section Fuzzing

For robustness testing of parser error paths, you can fuzz only the unzipped PRC section buffers
(`schema_globals_unzipped`, `tree_unzipped`, `tessellation_unzipped`, `geometry_unzipped`,
`extra_geometry_unzipped`, `model_unzipped`).

Enable the feature at configure time:

```bash
cmake -S . -B build -DPRC_ENABLE_UNZIPPED_FUZZ=ON
```

Runtime controls (environment variables):

- `PRC_FUZZ_SEED`: integer seed for deterministic replay (same input + same seed = same mutations)
- `PRC_FUZZ_RATE`: average mutation spacing in bytes (default `512`; lower = more mutations)
- `PRC_FUZZ_MAX_MUTATIONS`: per-buffer mutation cap (default `64`)
- `PRC_FUZZ_SECTION`: choose which section(s) to mutate. Options: `schema`, `tree`, `tessellation`,
  `geometry`, `extra`, `model`, `all` (default). Multiple values can be comma-separated.
- `PRC_FUZZ_SECTION_MASK`: optional numeric bitmask override (`1=schema`, `2=tree`,
  `4=tessellation`, `8=geometry`, `16=extra`, `32=model`)
- `PRC_FUZZ_LOG`: optional log path (default `prc_unzipped_fuzz.log`)

Example:

```bash
PRC_FUZZ_SEED=12345 PRC_FUZZ_SECTION=model PRC_FUZZ_RATE=1024 PRC_FUZZ_MAX_MUTATIONS=32 ./your_app
```

The log records seed and per-buffer mutation events so failures can be reproduced exactly.

## Versioning (AGPL Repository)

This repository uses Git-tag-driven versioning:

- Create release tags in the format `vMAJOR.MINOR.PATCH` (example: `v1.4.0`).
- CI derives build metadata from `git describe --tags --dirty --always`.
- Local and CI CMake config generates `prc_version.h` with:
	- `PRC_VERSION`
	- `PRC_GIT_DESCRIBE`

If no matching tag is present, the fallback version is `v0.1.0`.

## Release Process

- Checklist: [RELEASE_CHECKLIST.md](RELEASE_CHECKLIST.md)
- CI tag policy guard: [.github/workflows/tag-policy.yaml](.github/workflows/tag-policy.yaml)

## Documentation

Generate API documentation locally with Doxygen:

```bash
doxygen Doxyfile
```

This writes HTML output to `docs/doxygen/html`.

For strict public API doc checks used in CI (parameter-doc focused):

```bash
doxygen Doxyfile.public
```

Warnings are written to `docs/doxygen/warnings-public.log` and fail the strict check when present.
