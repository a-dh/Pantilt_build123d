# pantilt-build123d

Pan-tilt geometry and modeling using build123d.

## History

This model was built as a reaction to the wobbly SG9 servo - based pan and tilt units on commercial robot kits.  It was nice to use the same powder coated aluminim as the body, but the construction practicalities of using just the servo's pivot points for support left a wobbly unbalanced mess.

I initially worked out what I wanted in https://www.tinkercad.com/things/j7cgDCpGaQd-compact-pan-tilt-wip, but tweaking interior dimentions requires un-grouping and re-grouping.

Build123d allows me to use pythonic idioms to reference the interior touch points from other component modeling factories to make alignment explicit and
easy.

## Development

```
python3 -m venv .venv
. .venv/bin/activate
pip install .[dev]
```

