# aerpawlib

This is a python3.7+ package that can be used to write scripts compatible with
our dedicated script runner and environment. By using this library, you're
able to take advantage of a few custom frameworks when building your script.

To install as a local library (this maintains a dynamic link to this dir):
    `pip install -e .`

For specific usage instructions, check out `aerpawlib/__main__.py` or run
    `python -m aerpawlib -h`
in your shell after installing

## Documentation

To generate docs, use `pdoc`

```
pip install pdoc3

pdoc --html aerpawlib
```

Live docs can be viewed using `pdoc --http localhost:8080 aerpawlib`

## Examples

Under `examples/`, there are a few scripts that you can build off of.

* `examples/basic_runner.py` is an example of a script using the `BasicRunner`
runner type, which is the simplest runner possible (only an entry point that
accepts a `Vehicle` has to be defined)
* `examples/preplanned_trajectory.py` is a script useful to demonstrate how to
travel between different predetermined waypoints read from a `.plan` file
(generated by QGroundControl)
* `examples/squareoff_logging.py` is a script that demonstrates the
`StateMachine` runner type and nearly all functionality provided by it
(`@background` functions, `@timed_state`s, and how to handle additional
arguments)
