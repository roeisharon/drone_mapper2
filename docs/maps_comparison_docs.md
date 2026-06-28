> ⚠️ **Stale.** This doc predates the June 2026 spec/API updates. Authoritative sources:
> `assignment2_spec.pdf` (CLI = `<origin_map> <target_map> [comparison_config=<path>]`) and
> `docs/IMPLEMENTATION_PLAN.md` → Step 8. The old `resolution_ratio=<r1>/<r2>` argument is
> replaced by `comparison_config=<path>` (YAML).

#MapsComparison CLI Utility

Its API should meet the API as introduced in the reference stub implementation.
No need for any interface, just this class as a standalone.
This utility should also create an additional target (executable), with run instructions:
```
	./maps_comparison <map1> <map2> [comparison_config=<path>]
```

arguments:
`map1` and `map2` will be the .npy file names (with or without path).
Comparison_config is a path to a YAML file with the relevant configuration for executing the comparisons.resolution_ratio is a string of the actual `<res1>/actual <res2>`, if not provided assume the resolution of both maps is the same.
Note: support for the resolution_ratio argument  comparing when the resolutions are different is an optional bonus feature!
The program will print to the standard output just the score number
As a floating point number between 0 and 100 - no additional text!
In case of an error: print to standard output the score -1 and to standard error an descriptive error message of your choice.

The actual comparison algorithm is yours, but we will check that:
Two identical maps return 100.
Two very similar maps return a number close to 100, but not 100.
Two very distinct maps return a number close to 0.
Anything in between returns a reasonable result!

# MapsComparison API