# uctf

Unmanned Capture the Flag (U-CTF) project.

Please see the [documentation](doc/readme.md) in the `doc` subfolder for more information.

## Contributions to Python code

After modifying any Python files please run the linters and address any warnings:

```sh
nosetests3 test/
```

## Contributions to markdown

After modifying any markdown files please run the linter and address any warnings:

```sh
npm install remark-cli remark-lint
./node_modules/remark-cli/cli.js README.md doc/
```

## References

1. Google group [uctf-internal](https://groups.google.com/a/osrfoundation.org/forum/#!forum/uctf-internal)
